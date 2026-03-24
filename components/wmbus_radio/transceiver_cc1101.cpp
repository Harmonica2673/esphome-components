#include "transceiver_cc1101.h"

#include "esphome/core/log.h"

namespace esphome {

namespace wmbus_radio {

static const char *TAG = "CC1101";

// CC1101 crystal frequency (standard 26 MHz)
#define F_XTAL 26000000

void CC1101::setup() {
  this->common_setup();
  ESP_LOGV(TAG, "Setup");

  // CC1101 has no hardware reset pin — skip hardware reset, use software reset only
  ESP_LOGVV(TAG, "software reset (SRES)");
  this->strobe(CC1101_SRES);
  delay(10);

  ESP_LOGVV(TAG, "checking part number");
  uint8_t partnum = this->read_status_register(CC1101_PARTNUM);
  uint8_t version = this->read_status_register(CC1101_VERSION);
  ESP_LOGVV(TAG, "part: %02X, version: %02X", partnum, version);

  if (partnum != 0x00) {
    ESP_LOGE(TAG, "Invalid part number: %02X (expected 0x00)", partnum);
    return;
  }

  // Configure for wM-Bus Mode C/T at 868.95 MHz, 100 kbps, 2-FSK
  // RF settings based on TI Design Note DN022 and wmbusmeters reference
  ESP_LOGVV(TAG, "configuring GDO pins");
  // GDO2: Sync word sent/received (active high)
  this->write_register(CC1101_IOCFG2, CC1101_GDO_SYNC_WORD);
  // GDO1: High impedance (unused, default)
  this->write_register(CC1101_IOCFG1, CC1101_GDO_HI_Z);
  // FIX: GDO0 = 0x01 — asserts when RX FIFO >= threshold OR end-of-packet reached,
  //      active HIGH (no 0x40 inversion). The original RXFIFO_THR | 0x40 (threshold-only,
  //      active-low) meant frames shorter than 32 bytes never triggered the IRQ at all.
  this->write_register(CC1101_IOCFG0, 0x01);

  ESP_LOGVV(TAG, "configuring FIFO threshold");
  // FIX: Lower threshold from 0x07 (32 bytes) to 0x00 (4 bytes).
  //      Original 32-byte threshold caused any frame shorter than 32 bytes of payload
  //      to never trigger GDO0. 4 bytes ensures the IRQ fires early on all frame sizes.
  //      This matches the threshold used in the author's own wMbus-lib reference code.
  this->write_register(CC1101_FIFOTHR, 0x00);

  ESP_LOGVV(TAG, "configuring sync word");
  // Sync word for wM-Bus: 0x543D (with Manchester encoding: preamble 0x54)
  this->write_register(CC1101_SYNC1, WMBUS_SYNC_WORD_HIGH);
  this->write_register(CC1101_SYNC0, WMBUS_SYNC_WORD_LOW);

  ESP_LOGVV(TAG, "configuring packet control");
  // Max packet length (used as cap; actual framing is software-managed)
  this->write_register(CC1101_PKTLEN, 0xFF);
  // No address check, no append status, no CRC autoflush
  this->write_register(CC1101_PKTCTRL1, 0x00);
  // FIX: Infinite packet length mode (bits[1:0] = 0b10, value 0x02).
  //
  //      The original 0x00 (fixed 255-byte mode) was the primary cause of long-frame
  //      failure: the CC1101 waited for exactly 255 bytes before declaring end-of-packet,
  //      so MARCSTATE never reached IDLE on real wM-Bus frames and every packet hit the
  //      500ms timeout in read_in_task().
  //
  //      Variable length mode (0x01) was tried next but is also wrong: it treats the
  //      first chip byte in the FIFO as the frame length. Since Manchester decoding is
  //      done in software here (MDMCFG2 has Manchester disabled), the FIFO contains raw
  //      Manchester chips, not decoded bytes. The CC1101 read the wrong length and broke
  //      sync detection entirely ("nothing received").
  //
  //      Infinite mode (0x02) is correct: the CC1101 never terminates reception based
  //      on a hardware length field; the FIFO fills continuously and software manages
  //      all frame boundaries using the decoded L-field. This is also exactly what the
  //      author's own wMbus-lib reference implementation uses.
  this->write_register(CC1101_PKTCTRL0, 0x02);
  // No device address filtering
  this->write_register(CC1101_ADDR, 0x00);
  // Channel number 0
  this->write_register(CC1101_CHANNR, 0x00);

  ESP_LOGVV(TAG, "configuring frequency synthesizer");
  // IF frequency: ~152 kHz (typical for 100 kbps)
  this->write_register(CC1101_FSCTRL1, 0x08);
  this->write_register(CC1101_FSCTRL0, 0x00);

  ESP_LOGVV(TAG, "setting radio frequency");
  // Frequency: 868.95 MHz
  // FREQ = (f_carrier * 2^16) / f_xtal = (868950000 * 65536) / 26000000 = 0x216BD0
  const uint32_t frequency = 868950000;
  uint32_t freq_reg = ((uint64_t)frequency << 16) / F_XTAL;
  this->write_register(CC1101_FREQ2, BYTE(freq_reg, 2));
  this->write_register(CC1101_FREQ1, BYTE(freq_reg, 1));
  this->write_register(CC1101_FREQ0, BYTE(freq_reg, 0));

  ESP_LOGVV(TAG, "configuring modem");
  // Modem configuration for 100 kbps, 2-FSK, 50 kHz deviation
  // MDMCFG4: Channel bandwidth and data rate exponent
  this->write_register(CC1101_MDMCFG4, 0x5C);
  // MDMCFG3: Data rate mantissa (100 kbps)
  this->write_register(CC1101_MDMCFG3, 0x04);
  // MDMCFG2: 2-FSK, 16/16 sync word, Manchester OFF (software-decoded)
  this->write_register(CC1101_MDMCFG2, 0x06);
  // MDMCFG1: FEC disabled, preamble 4 bytes
  this->write_register(CC1101_MDMCFG1, 0x22);
  // MDMCFG0: Channel spacing
  this->write_register(CC1101_MDMCFG0, 0xF8);

  ESP_LOGVV(TAG, "configuring deviation");
  this->write_register(CC1101_DEVIATN, 0x44);

  ESP_LOGVV(TAG, "configuring state machine");
  this->write_register(CC1101_MCSM2, 0x07);
  this->write_register(CC1101_MCSM1, 0x00);
  this->write_register(CC1101_MCSM0, 0x18);

  ESP_LOGVV(TAG, "configuring AFC/AGC");
  this->write_register(CC1101_FOCCFG, 0x2E);
  this->write_register(CC1101_BSCFG, 0xBF);
  this->write_register(CC1101_AGCCTRL2, 0x43);
  this->write_register(CC1101_AGCCTRL1, 0x09);
  this->write_register(CC1101_AGCCTRL0, 0xB5);

  ESP_LOGVV(TAG, "configuring WOR");
  this->write_register(CC1101_WOREVT1, 0x87);
  this->write_register(CC1101_WOREVT0, 0x6B);
  this->write_register(CC1101_WORCTRL, 0xFB);

  ESP_LOGVV(TAG, "configuring front end");
  this->write_register(CC1101_FREND1, 0xB6);
  this->write_register(CC1101_FREND0, 0x10);

  ESP_LOGVV(TAG, "configuring frequency calibration");
  this->write_register(CC1101_FSCAL3, 0xEA);
  this->write_register(CC1101_FSCAL2, 0x2A);
  this->write_register(CC1101_FSCAL1, 0x00);
  this->write_register(CC1101_FSCAL0, 0x1F);
  this->write_register(CC1101_RCCTRL1, 0x41);
  this->write_register(CC1101_RCCTRL0, 0x00);

  ESP_LOGVV(TAG, "configuring test registers");
  this->write_register(CC1101_FSTEST, 0x59);
  this->write_register(CC1101_PTEST, 0x7F);
  this->write_register(CC1101_AGCTEST, 0x3F);
  this->write_register(CC1101_TEST2, 0x81);
  this->write_register(CC1101_TEST1, 0x35);
  this->write_register(CC1101_TEST0, 0x09);

  ESP_LOGVV(TAG, "calibrating");
  this->strobe(CC1101_SCAL);
  delay(1);

  ESP_LOGVV(TAG, "entering RX mode");
  this->strobe(CC1101_SFRX);
  this->strobe(CC1101_SRX);

  ESP_LOGV(TAG, "CC1101 setup done");
}

uint8_t CC1101::strobe(uint8_t cmd) {
  this->delegate_->begin_transaction();
  uint8_t status = this->delegate_->transfer(cmd);
  this->delegate_->end_transaction();
  return status;
}

uint8_t CC1101::read_register(uint8_t address) {
  this->delegate_->begin_transaction();
  this->delegate_->transfer(address | CC1101_READ_SINGLE);
  uint8_t value = this->delegate_->transfer(0x00);
  this->delegate_->end_transaction();
  return value;
}

uint8_t CC1101::read_status_register(uint8_t address) {
  this->delegate_->begin_transaction();
  this->delegate_->transfer(address | CC1101_READ_BURST);
  uint8_t value = this->delegate_->transfer(0x00);
  this->delegate_->end_transaction();
  return value;
}

void CC1101::write_register(uint8_t address, uint8_t value) {
  this->delegate_->begin_transaction();
  this->delegate_->transfer(address | CC1101_WRITE_SINGLE);
  this->delegate_->transfer(value);
  this->delegate_->end_transaction();
}

void CC1101::write_burst(uint8_t address, const uint8_t *data, size_t length) {
  this->delegate_->begin_transaction();
  this->delegate_->transfer(address | CC1101_WRITE_BURST);
  for (size_t i = 0; i < length; i++) {
    this->delegate_->transfer(data[i]);
  }
  this->delegate_->end_transaction();
}

void CC1101::read_burst(uint8_t address, uint8_t *data, size_t length) {
  this->delegate_->begin_transaction();
  this->delegate_->transfer(address | CC1101_READ_BURST);
  for (size_t i = 0; i < length; i++) {
    data[i] = this->delegate_->transfer(0x00);
  }
  this->delegate_->end_transaction();
}

uint8_t CC1101::get_rx_bytes() {
  return this->read_status_register(CC1101_RXBYTES) & 0x7F;
}

optional<uint8_t> CC1101::read() {
  // FIX: GDO0 is now active HIGH (0x01 = RXFIFO_THR_OR_EOP), check for true.
  if (this->irq_pin_->digital_read() == true) {
    uint8_t rx_bytes = this->get_rx_bytes();
    if (rx_bytes > 0) {
      this->last_rssi_ = (int8_t)this->read_status_register(CC1101_RSSI);
      uint8_t data;
      this->read_burst(CC1101_RXFIFO, &data, 1);
      return data;
    }
  }
  return {};
}

size_t CC1101::get_frame(uint8_t *buffer, size_t length, uint32_t offset) {
  auto byte = this->read();
  if (!byte.has_value())
    return 0;
  *buffer = *byte;
  return 1;
}

void CC1101::restart_rx() {
  ESP_LOGVV(TAG, "Restarting RX");
  this->strobe(CC1101_SIDLE);
  delay(1);
  this->strobe(CC1101_SFRX);
  this->strobe(CC1101_SRX);
  delay(1);
}

int8_t CC1101::get_rssi() {
  int8_t rssi_dec = this->last_rssi_;
  int16_t rssi_dbm;
  if (rssi_dec >= 128) {
    rssi_dbm = ((int16_t)(rssi_dec - 256) / 2) - 74;
  } else {
    rssi_dbm = (rssi_dec / 2) - 74;
  }
  return (int8_t)rssi_dbm;
}

const char *CC1101::get_name() { return TAG; }

bool CC1101::read_in_task(uint8_t *buffer, size_t length, uint32_t offset) {
  size_t total = 0;
  uint32_t last_progress = millis();

  while (total < length) {
    // Check for FIFO overflow (bit 7 of RXBYTES)
    uint8_t rxbytes_raw = this->read_status_register(CC1101_RXBYTES);
    if (rxbytes_raw & 0x80) {
      ESP_LOGW(TAG, "RX FIFO overflow");
      this->restart_rx();
      return false;
    }

    uint8_t available = rxbytes_raw & 0x7F;
    size_t remaining = length - total;

    if (available > 0) {
      // CC1101 errata: do not drain the FIFO to 0 bytes while the radio is still
      // actively shifting bits in. If the shift register is simultaneously pushing a
      // new byte, reading the last FIFO byte can corrupt it.
      //
      // FIX: Guard is applied only when available < remaining (more bytes are on the
      //      way), so we leave 1 byte as a buffer. When available >= remaining we
      //      already have all the bytes we need, drain exactly `remaining` and exit.
      //
      //      The original condition (remaining > 1 && available > 1) was wrong: it
      //      withheld the guard byte even after the frame was fully in the FIFO,
      //      causing the loop to stall forever on the last byte of long frames.
      //
      //      In infinite packet mode (PKTCTRL0 = 0x02) the radio never auto-stops,
      //      so we cannot rely on MARCSTATE going IDLE — the FIFO level relative to
      //      the expected frame length is the only reliable completion signal.
      size_t to_read;
      if (available < remaining) {
        // Frame still arriving — leave 1 guard byte.
        to_read = (available > 1) ? (size_t)(available - 1) : 0;
      } else {
        // All remaining bytes are already in the FIFO — safe to drain them all.
        to_read = remaining;
      }

      if (to_read > 0) {
        if (total == 0 && offset == 0) {
          this->last_rssi_ = (int8_t)this->read_status_register(CC1101_RSSI);
        }
        this->read_burst(CC1101_RXFIFO, buffer + total, to_read);
        total += to_read;
        last_progress = millis();
      }
    }

    // Note: MARCSTATE-based end-of-packet detection is intentionally omitted.
    // In infinite packet mode the CC1101 never transitions to IDLE automatically,
    // so MARCSTATE_IDLE never fires. The while-loop exits when total == length.
    // CC1101_MARCSTATE_RX_END was also not a valid datasheet state.

    if (millis() - last_progress > 500) {
      ESP_LOGW(TAG, "RX timeout after %zu bytes (need %zu)", total + offset, length + offset);
      return false;
    }

    if (total < length) {
      delayMicroseconds(200);
    }
  }

  return true;
}

} // namespace wmbus_radio

} // namespace esphome