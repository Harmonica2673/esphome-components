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
  //      active HIGH (no 0x40 inversion). The previous value (RXFIFO_THR | 0x40, active
  //      low, threshold-only) meant short frames whose payload never reached 32 bytes
  //      never triggered the IRQ at all.
  this->write_register(CC1101_IOCFG0, 0x01);

  ESP_LOGVV(TAG, "configuring FIFO threshold");
  // FIX: Lower threshold from 0x07 (32 bytes) to 0x00 (4 bytes).
  //      With the old 32-byte threshold, any frame shorter than 32 bytes of payload
  //      would never assert GDO0 via the threshold path; only the new EOP path (above)
  //      saves them. A low threshold also reduces FIFO-overflow risk on longer frames.
  this->write_register(CC1101_FIFOTHR, 0x00);

  ESP_LOGVV(TAG, "configuring sync word");
  // Sync word for wM-Bus: 0x543D (with Manchester encoding: preamble 0x54)
  this->write_register(CC1101_SYNC1, WMBUS_SYNC_WORD_HIGH);
  this->write_register(CC1101_SYNC0, WMBUS_SYNC_WORD_LOW);

  ESP_LOGVV(TAG, "configuring packet control");
  // Max packet length cap when using variable-length mode
  this->write_register(CC1101_PKTLEN, 0xFF);
  // No address check, no append status, no CRC autoflush
  this->write_register(CC1101_PKTCTRL1, 0x00);
  // FIX: Variable packet length mode (bits[1:0] = 0b01).
  //      The original value 0x00 (fixed 255-byte mode) caused the CC1101 state machine
  //      to wait for exactly 255 bytes before signalling end-of-packet, so MARCSTATE
  //      never reached IDLE on normal wM-Bus frames and every packet hit the 500ms
  //      timeout. In variable mode the first byte in the FIFO is treated as the frame
  //      length, which matches the wM-Bus L-field exactly.
  this->write_register(CC1101_PKTCTRL0, 0x01);
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
  // BW = f_xtal / (8 * (4 + CHANBW_M) * 2^CHANBW_E) = 26M / (8 * 4 * 4) = ~203 kHz
  this->write_register(CC1101_MDMCFG4, 0x5C);
  // MDMCFG3: Data rate mantissa
  // DRATE = ((256 + DRATE_M) * 2^DRATE_E) * f_xtal / 2^28 = 100 kbps
  this->write_register(CC1101_MDMCFG3, 0x04);
  // MDMCFG2: Modulation format (2-FSK), sync mode (16/16 sync word bits)
  this->write_register(CC1101_MDMCFG2, 0x06);
  // MDMCFG1: FEC disabled, preamble 4 bytes
  this->write_register(CC1101_MDMCFG1, 0x22);
  // MDMCFG0: Channel spacing exponent
  this->write_register(CC1101_MDMCFG0, 0xF8);

  ESP_LOGVV(TAG, "configuring deviation");
  // Deviation: ~50 kHz
  // DEVIATION = (f_xtal / 2^17) * (8 + DEVIATION_M) * 2^DEVIATION_E
  this->write_register(CC1101_DEVIATN, 0x44);

  ESP_LOGVV(TAG, "configuring state machine");
  // MCSM2: RX time qualifier enabled
  this->write_register(CC1101_MCSM2, 0x07);
  // MCSM1: CCA mode, RX after RX, IDLE after TX
  this->write_register(CC1101_MCSM1, 0x00);
  // MCSM0: Auto calibrate from IDLE to RX/TX, PO timeout
  this->write_register(CC1101_MCSM0, 0x18);

  ESP_LOGVV(TAG, "configuring AFC/AGC");
  // FOCCFG: Frequency offset compensation
  this->write_register(CC1101_FOCCFG, 0x2E);
  // BSCFG: Bit synchronization
  this->write_register(CC1101_BSCFG, 0xBF);
  // AGC control
  this->write_register(CC1101_AGCCTRL2, 0x43);
  this->write_register(CC1101_AGCCTRL1, 0x09);
  this->write_register(CC1101_AGCCTRL0, 0xB5);

  ESP_LOGVV(TAG, "configuring WOR");
  // Wake-on-Radio: disabled (but configured for compatibility)
  this->write_register(CC1101_WOREVT1, 0x87);
  this->write_register(CC1101_WOREVT0, 0x6B);
  this->write_register(CC1101_WORCTRL, 0xFB);

  ESP_LOGVV(TAG, "configuring front end");
  // Front end RX: LNA and mixer current
  this->write_register(CC1101_FREND1, 0xB6);
  // Front end TX: PA power index
  this->write_register(CC1101_FREND0, 0x10);

  ESP_LOGVV(TAG, "configuring frequency calibration");
  // Frequency synthesizer calibration
  this->write_register(CC1101_FSCAL3, 0xEA);
  this->write_register(CC1101_FSCAL2, 0x2A);
  this->write_register(CC1101_FSCAL1, 0x00);
  this->write_register(CC1101_FSCAL0, 0x1F);

  // RC oscillator configuration
  this->write_register(CC1101_RCCTRL1, 0x41);
  this->write_register(CC1101_RCCTRL0, 0x00);

  ESP_LOGVV(TAG, "configuring test registers");
  // Test registers for optimal performance
  this->write_register(CC1101_FSTEST, 0x59);
  this->write_register(CC1101_PTEST, 0x7F);
  this->write_register(CC1101_AGCTEST, 0x3F);
  this->write_register(CC1101_TEST2, 0x81);
  this->write_register(CC1101_TEST1, 0x35);
  this->write_register(CC1101_TEST0, 0x09);

  ESP_LOGVV(TAG, "calibrating");
  // Calibrate frequency synthesizer
  this->strobe(CC1101_SCAL);
  delay(1);

  ESP_LOGVV(TAG, "entering RX mode");
  // Flush RX FIFO and enter RX mode
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
  // FIX: GDO0 is now active HIGH (EOP or threshold), so check for true not false.
  if (this->irq_pin_->digital_read() == true) {
    uint8_t rx_bytes = this->get_rx_bytes();
    if (rx_bytes > 0) {
      // Read RSSI before reading data byte (RSSI is valid after sync word)
      this->last_rssi_ = (int8_t)this->read_status_register(CC1101_RSSI);
      // Read single byte from RX FIFO
      uint8_t data;
      this->read_burst(CC1101_RXFIFO, &data, 1);
      return data;
    }
  }
  return {};
}

size_t CC1101::get_frame(uint8_t *buffer, size_t length, uint32_t offset) {
  // CC1101 reads byte-by-byte from FIFO (offset is ignored for FIFO-based reading)
  // Returns 1 on success, 0 if FIFO is empty (waiting for more data)
  auto byte = this->read();
  if (!byte.has_value())
    return 0;
  *buffer = *byte;
  return 1;
}

void CC1101::restart_rx() {
  ESP_LOGVV(TAG, "Restarting RX");
  // Go to IDLE state
  this->strobe(CC1101_SIDLE);
  delay(1);
  // Flush RX FIFO
  this->strobe(CC1101_SFRX);
  // Enter RX mode
  this->strobe(CC1101_SRX);
  delay(1);
}

int8_t CC1101::get_rssi() {
  // Convert RSSI_dec to dBm
  // RSSI_dBm = (RSSI_dec / 2) - RSSI_offset
  // RSSI_offset is typically 74 for 868 MHz
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
    // Check for FIFO overflow
    uint8_t rxbytes_raw = this->read_status_register(CC1101_RXBYTES);
    if (rxbytes_raw & 0x80) {
      ESP_LOGW(TAG, "RX FIFO overflow");
      this->restart_rx();
      return false;
    }

    uint8_t available = rxbytes_raw & 0x7F;
    size_t remaining = length - total;

    if (available > 0) {
      // FIX: Re-read MARCSTATE here (once per loop iteration) so the errata guard and
      //      the end-of-packet drain both use the same consistent snapshot, avoiding a
      //      race where the state changes between two separate reads.
      uint8_t marcstate = this->read_status_register(CC1101_MARCSTATE) & 0x1F;
      bool still_receiving = (marcstate == CC1101_MARCSTATE_RX);  // 0x0D per datasheet

      size_t to_read;
      if (still_receiving && available == 1) {
        // FIX: Only 1 byte available and radio is still actively receiving.
        //      Leave it — reading the last byte from an active FIFO can corrupt the
        //      next byte per CC1101 errata. Wait for more data to arrive.
        to_read = 0;
      } else if (still_receiving) {
        // FIX: Radio still receiving — leave 1-byte guard per errata, read the rest.
        //      The original condition (remaining > 1 && available > 1) was wrong: it
        //      left 1 byte un-read even after the radio finished, causing the loop to
        //      stall on the last byte of any frame.
        to_read = std::min((size_t)(available - 1), remaining);
      } else {
        // Radio finished receiving (IDLE or other terminal state) — safe to drain all.
        to_read = std::min((size_t)available, remaining);
      }

      if (to_read > 0) {
        if (total == 0 && offset == 0) {
          this->last_rssi_ = (int8_t)this->read_status_register(CC1101_RSSI);
        }
        this->read_burst(CC1101_RXFIFO, buffer + total, to_read);
        total += to_read;
        last_progress = millis();
      }

      // FIX: Only check for end-of-packet via MARCSTATE_IDLE (0x01).
      //      The original code also checked CC1101_MARCSTATE_RX_END which is not a
      //      valid CC1101 state (not defined in the datasheet), so that branch never
      //      fired. With variable-length packet mode, the chip transitions to IDLE
      //      cleanly after the last byte, making this check reliable.
      if (marcstate == CC1101_MARCSTATE_IDLE) {
        // Packet is complete — drain any bytes that arrived between the RXBYTES read
        // above and now, then exit.
        uint8_t final_bytes = this->read_status_register(CC1101_RXBYTES) & 0x7F;
        if (final_bytes > 0 && total + final_bytes <= length) {
          this->read_burst(CC1101_RXFIFO, buffer + total, final_bytes);
          total += final_bytes;
        }
        break;
      }
    }

    // Timeout: 500ms without progress
    if (millis() - last_progress > 500) {
      ESP_LOGW(TAG, "RX timeout after %zu bytes (need %zu)", total + offset, length + offset);
      return false;
    }

    delayMicroseconds(200);
  }

  return (total == length);
}

} // namespace wmbus_radio

} // namespace esphome