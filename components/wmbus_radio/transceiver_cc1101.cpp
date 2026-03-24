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
  // GDO2: Sync word sent/received (active low, inverted) — UNCHANGED
  this->write_register(CC1101_IOCFG2, CC1101_GDO_SYNC_WORD | 0x40);
  // GDO1: High impedance (unused, default) — UNCHANGED
  this->write_register(CC1101_IOCFG1, CC1101_GDO_HI_Z);
  // GDO0: RXFIFO threshold reached (IRQ pin), active low (inverted) — UNCHANGED
  // Short frames were already working with this setting; do not change it.
  this->write_register(CC1101_IOCFG0, CC1101_GDO_RXFIFO_THR | 0x40);

  ESP_LOGVV(TAG, "configuring FIFO threshold");
  // FIFO threshold: 7 = RX FIFO >= 32 bytes — UNCHANGED
  // Short frames worked with this; do not change it.
  this->write_register(CC1101_FIFOTHR, 0x07);

  ESP_LOGVV(TAG, "configuring sync word");
  this->write_register(CC1101_SYNC1, WMBUS_SYNC_WORD_HIGH);
  this->write_register(CC1101_SYNC0, WMBUS_SYNC_WORD_LOW);

  ESP_LOGVV(TAG, "configuring packet control");
  this->write_register(CC1101_PKTLEN, 0xFF);
  // No address check, no append status, no CRC autoflush
  this->write_register(CC1101_PKTCTRL1, 0x00);
  // *** FIX 1: Infinite packet length mode (0x02) ***
  //
  // Original value 0x00 = fixed 255-byte mode. The CC1101 waited for exactly 255
  // bytes before declaring end-of-packet, so MARCSTATE never reached IDLE on real
  // wM-Bus frames and read_in_task() timed out every time on long frames.
  //
  // Variable length mode (0x01) was tried but is also wrong here: it reads the first
  // FIFO byte as the frame length. Since Manchester decoding is done in software
  // (MDMCFG2 has Manchester disabled), the FIFO contains raw Manchester chips, not
  // decoded bytes — the CC1101 would misread the length and break reception entirely.
  //
  // Infinite mode (0x02): the CC1101 never terminates reception via a hardware length
  // field. The FIFO fills continuously and software manages all frame boundaries using
  // the decoded L-field. This matches the author's own wMbus-lib reference code which
  // explicitly sets INFINITE_PACKET_LENGTH for exactly this reason.
  this->write_register(CC1101_PKTCTRL0, 0x02);
  this->write_register(CC1101_ADDR, 0x00);
  this->write_register(CC1101_CHANNR, 0x00);

  ESP_LOGVV(TAG, "configuring frequency synthesizer");
  this->write_register(CC1101_FSCTRL1, 0x08);
  this->write_register(CC1101_FSCTRL0, 0x00);

  ESP_LOGVV(TAG, "setting radio frequency");
  // Frequency: 868.95 MHz
  const uint32_t frequency = 868950000;
  uint32_t freq_reg = ((uint64_t)frequency << 16) / F_XTAL;
  this->write_register(CC1101_FREQ2, BYTE(freq_reg, 2));
  this->write_register(CC1101_FREQ1, BYTE(freq_reg, 1));
  this->write_register(CC1101_FREQ0, BYTE(freq_reg, 0));

  ESP_LOGVV(TAG, "configuring modem");
  this->write_register(CC1101_MDMCFG4, 0x5C);
  this->write_register(CC1101_MDMCFG3, 0x04);
  this->write_register(CC1101_MDMCFG2, 0x06);
  this->write_register(CC1101_MDMCFG1, 0x22);
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
  // GDO0 is active LOW (RXFIFO_THR | 0x40) — UNCHANGED from original
  if (this->irq_pin_->digital_read() == false) {
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
      size_t to_read;
      if (available < remaining) {
        // *** FIX 2: CC1101 errata guard ***
        //
        // The radio is still actively receiving (more bytes expected than are currently
        // in the FIFO). Do not drain the last byte: if the shift register is pushing a
        // new byte in simultaneously, reading the last FIFO byte can corrupt it.
        //
        // Original condition was (remaining > 1 && available > 1), which withheld the
        // guard byte even after the frame had fully arrived in the FIFO (when
        // available >= remaining), stalling the loop forever on the last byte of any
        // long frame.
        //
        // New condition: leave 1 guard byte only when available < remaining (the frame
        // is genuinely still in flight). Once available >= remaining, we have all the
        // bytes we need and can drain them all safely.
        to_read = (available > 1) ? (size_t)(available - 1) : 0;
      } else {
        // All remaining bytes are already in the FIFO — drain exactly what we need.
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

      // *** FIX 3: Removed MARCSTATE end-of-packet drain ***
      //
      // In infinite packet mode (PKTCTRL0 = 0x02) the CC1101 never transitions to
      // IDLE by itself, so the MARCSTATE_IDLE check could never fire. The loop exits
      // naturally when total == length. CC1101_MARCSTATE_RX_END was also not a valid
      // datasheet state value.
    }

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