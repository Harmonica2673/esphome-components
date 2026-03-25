#include "transceiver_cc1101.h"

#include "esphome/core/log.h"

namespace esphome {

namespace wmbus_radio {

static const char *TAG = "CC1101";

// CC1101 crystal frequency (standard 26 MHz)
#define F_XTAL 26000000

// ── setup() ─────────────────────────────────────────────────────────────────
// UNCHANGED from original. Every register value is identical.
// Previous attempts that modified PKTCTRL0, IOCFG0, or FIFOTHR all broke
// reception and have been reverted.
void CC1101::setup() {
  this->common_setup();
  ESP_LOGV(TAG, "Setup");

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

  ESP_LOGVV(TAG, "configuring GDO pins");
  // GDO2: Sync word sent/received (active low, inverted)
  this->write_register(CC1101_IOCFG2, CC1101_GDO_SYNC_WORD | 0x40);
  // GDO1: High impedance (unused)
  this->write_register(CC1101_IOCFG1, CC1101_GDO_HI_Z);
  // GDO0: RX FIFO threshold reached, active low (inverted) → falling edge = data ready
  this->write_register(CC1101_IOCFG0, CC1101_GDO_RXFIFO_THR | 0x40);

  ESP_LOGVV(TAG, "configuring FIFO threshold");
  // 0x07 = RX FIFO asserts when ≥ 32 bytes present
  this->write_register(CC1101_FIFOTHR, 0x07);

  ESP_LOGVV(TAG, "configuring sync word");
  this->write_register(CC1101_SYNC1, WMBUS_SYNC_WORD_HIGH);
  this->write_register(CC1101_SYNC0, WMBUS_SYNC_WORD_LOW);

  ESP_LOGVV(TAG, "configuring packet control");
  this->write_register(CC1101_PKTLEN, 0xFF);
  this->write_register(CC1101_PKTCTRL1, 0x00);
  // 0x00 = fixed 255-byte packet length mode (original, unchanged)
  this->write_register(CC1101_PKTCTRL0, 0x00);
  this->write_register(CC1101_ADDR, 0x00);
  this->write_register(CC1101_CHANNR, 0x00);

  ESP_LOGVV(TAG, "configuring frequency synthesizer");
  this->write_register(CC1101_FSCTRL1, 0x08);
  this->write_register(CC1101_FSCTRL0, 0x00);

  ESP_LOGVV(TAG, "setting radio frequency");
  // 868.95 MHz: FREQ = (868950000 * 2^16) / 26000000
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

// ── SPI primitives ──────────────────────────────────────────────────────────
// All unchanged from original.

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

// ── read() / get_frame() ────────────────────────────────────────────────────
// Unchanged. Used by the base-class read_in_task if our override is ever
// removed, and kept for API completeness.

optional<uint8_t> CC1101::read() {
  // GDO0 active LOW: pin LOW means FIFO ≥ threshold
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

// ── restart_rx() ────────────────────────────────────────────────────────────
// Unchanged.

void CC1101::restart_rx() {
  ESP_LOGVV(TAG, "Restarting RX");
  this->strobe(CC1101_SIDLE);
  delay(1);
  this->strobe(CC1101_SFRX);
  this->strobe(CC1101_SRX);
  delay(1);
}

// ── get_rssi() ──────────────────────────────────────────────────────────────
// Unchanged.

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

// ── read_in_task() ──────────────────────────────────────────────────────────
//
// This is the ONLY function that changed. Two targeted fixes:
//
// FIX 1 — Errata guard condition:
//   Original: (remaining > 1 && available > 1) → always withholds 1 guard byte
//   even when the frame has fully arrived in the FIFO (available >= remaining).
//   This left the final byte permanently unread when available == remaining,
//   forcing the loop to spin until timeout on every large frame.
//
//   Fixed: apply the guard ONLY when available < remaining (the radio is still
//   actively shifting bytes in, so draining the last FIFO byte risks corruption).
//   When available >= remaining, all needed bytes are already in the FIFO and
//   the shift register is idle — drain exactly what we need and exit cleanly.
//
// FIX 2 — Unconditional 200µs delay:
//   Original: delayMicroseconds(200) fires after EVERY loop iteration, including
//   ones where a large burst was just successfully read. At 100 kbps (1 byte
//   per 80µs) this delay allows ~2.5 bytes to accumulate before the next
//   RXBYTES poll. For large frames (encoded size > 64 bytes = FIFO capacity)
//   the task falls behind the incoming byte rate: the per-iteration SPI overhead
//   (RXBYTES read + burst read + MARCSTATE read) already consumes significant
//   time, and adding 200µs on top causes the FIFO to overflow.
//
//   Fixed: after a successful burst read, `continue` immediately to re-poll
//   RXBYTES without any delay. The delay is only applied when the FIFO is
//   empty (no bytes available yet), preventing a tight busy-wait spin while
//   still yielding CPU when there is genuinely nothing to read.
//
// FIX 3 — Removed unnecessary MARCSTATE read:
//   The MARCSTATE check (IDLE or RX_END detection) added one extra 2-byte SPI
//   transaction per loop iteration. In fixed-255 mode the radio never reaches
//   IDLE for frames < 255 bytes, so the check never triggered. Removing it
//   reduces per-iteration overhead and makes the drain loop faster, directly
//   reducing overflow risk on large frames.
//   (CC1101_MARCSTATE_RX_END = 0x0E is a real CC1101 state, but it only occurs
//   in fixed-length mode after the full PKTLEN bytes are received — never for
//   our sub-255-byte wM-Bus frames.)

bool CC1101::read_in_task(uint8_t *buffer, size_t length, uint32_t offset) {
  size_t total = 0;
  uint32_t last_progress = millis();

  while (total < length) {
    // Check for FIFO overflow (bit 7 set in RXBYTES)
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

      if (available >= remaining) {
        // All the bytes we still need are already in the FIFO.
        // The shift register is idle — safe to drain everything.
        to_read = remaining;
      } else {
        // Frame still arriving: the shift register may be pushing a new byte
        // into the FIFO right now. Per CC1101 errata, do not read the FIFO
        // completely to zero in this situation — leave 1 byte as a guard.
        to_read = (available > 1) ? (size_t)(available - 1) : 0;
      }

      if (to_read > 0) {
        if (total == 0 && offset == 0) {
          this->last_rssi_ = (int8_t)this->read_status_register(CC1101_RSSI);
        }
        this->read_burst(CC1101_RXFIFO, buffer + total, to_read);
        total += to_read;
        last_progress = millis();

        // Re-poll RXBYTES immediately — do NOT delay after a successful read.
        // At 100 kbps a byte arrives every 80µs; adding 200µs here causes the
        // task to fall behind on large frames and overflow the 64-byte FIFO.
        continue;
      }
    }

    // FIFO was empty (or only the guard byte remained). Check for timeout then
    // yield briefly so we don't busy-spin while waiting for the next byte.
    if (millis() - last_progress > 500) {
      ESP_LOGW(TAG, "RX timeout after %zu bytes (need %zu)", total + offset, length + offset);
      return false;
    }

    delayMicroseconds(200);
  }

  return true;
}

} // namespace wmbus_radio

} // namespace esphome