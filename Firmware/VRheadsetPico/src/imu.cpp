// IIM-42652 6-axis IMU driver for RP2040, SPI implemented with a PIO state machine.
// Single file. Based on TDK InvenSense DS-000440 rev 1.1.
//
// Build: add this file to your target and link pico_stdlib and hardware_pio, e.g.
//   add_executable(app main.cpp iim42652_pio.cpp)
//   target_link_libraries(app pico_stdlib hardware_pio)
// To build the demo at the bottom of this file, compile with -DIIM42652_EXAMPLE_MAIN.
//
// Wiring: SCK -> AP_SCLK (pin 13), MOSI -> AP_SDI (pin 14), MISO <- AP_SDO (pin 1),
//         CS -> AP_CS (pin 12), INT1 (pin 4) optional. The device runs SPI mode 0.
//         SCK, MOSI and MISO can be any GPIOs (PIO has no pin restrictions).
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

namespace iim42652 {
namespace reg {

constexpr uint8_t kWhoAmIValue    = 0x6F;
constexpr uint8_t kI2cAddrAd0Low  = 0x68;  // AP_AD0 = 0
constexpr uint8_t kI2cAddrAd0High = 0x69;  // AP_AD0 = 1

// Accessible from every bank.
constexpr uint8_t REG_BANK_SEL = 0x76;

// ------------------------------------------------------------------ Bank 0
constexpr uint8_t DEVICE_CONFIG       = 0x11;
constexpr uint8_t DRIVE_CONFIG        = 0x13;
constexpr uint8_t INT_CONFIG          = 0x14;
constexpr uint8_t FIFO_CONFIG         = 0x16;
constexpr uint8_t TEMP_DATA1          = 0x1D;  // TEMP, ACCEL XYZ, GYRO XYZ follow contiguously (14 bytes)
constexpr uint8_t ACCEL_DATA_X1       = 0x1F;
constexpr uint8_t GYRO_DATA_X1        = 0x25;
constexpr uint8_t TMST_FSYNCH         = 0x2B;
constexpr uint8_t INT_STATUS          = 0x2D;
constexpr uint8_t FIFO_COUNTH         = 0x2E;
constexpr uint8_t FIFO_COUNTL         = 0x2F;
constexpr uint8_t FIFO_DATA           = 0x30;
constexpr uint8_t INT_STATUS2         = 0x37;  // SMD / WOM X,Y,Z
constexpr uint8_t INT_STATUS3         = 0x38;  // APEX events
constexpr uint8_t SIGNAL_PATH_RESET   = 0x4B;
constexpr uint8_t INTF_CONFIG0        = 0x4C;
constexpr uint8_t INTF_CONFIG1        = 0x4D;
constexpr uint8_t PWR_MGMT0           = 0x4E;
constexpr uint8_t GYRO_CONFIG0        = 0x4F;
constexpr uint8_t ACCEL_CONFIG0       = 0x50;
constexpr uint8_t GYRO_CONFIG1        = 0x51;
constexpr uint8_t GYRO_ACCEL_CONFIG0  = 0x52;
constexpr uint8_t ACCEL_CONFIG1       = 0x53;
constexpr uint8_t TMST_CONFIG         = 0x54;
constexpr uint8_t APEX_CONFIG0        = 0x56;
constexpr uint8_t SMD_CONFIG          = 0x57;
constexpr uint8_t FIFO_CONFIG1        = 0x5F;
constexpr uint8_t FIFO_CONFIG2        = 0x60;
constexpr uint8_t FIFO_CONFIG3        = 0x61;
constexpr uint8_t FSYNC_CONFIG        = 0x62;
constexpr uint8_t INT_CONFIG0         = 0x63;
constexpr uint8_t INT_CONFIG1         = 0x64;
constexpr uint8_t INT_SOURCE0         = 0x65;
constexpr uint8_t INT_SOURCE1         = 0x66;
constexpr uint8_t INT_SOURCE3         = 0x68;
constexpr uint8_t INT_SOURCE4         = 0x69;
constexpr uint8_t FIFO_LOST_PKT0      = 0x6C;
constexpr uint8_t FIFO_LOST_PKT1      = 0x6D;
constexpr uint8_t SELF_TEST_CONFIG    = 0x70;
constexpr uint8_t WHO_AM_I            = 0x75;

// ------------------------------------------------------------------ Bank 1
constexpr uint8_t B1_SENSOR_CONFIG0       = 0x03;
constexpr uint8_t B1_GYRO_CONFIG_STATIC2  = 0x0B;  // AAF_DIS (bit1), NF_DIS (bit0)
constexpr uint8_t B1_GYRO_CONFIG_STATIC3  = 0x0C;  // AAF_DELT [5:0]
constexpr uint8_t B1_GYRO_CONFIG_STATIC4  = 0x0D;  // AAF_DELTSQR [7:0]
constexpr uint8_t B1_GYRO_CONFIG_STATIC5  = 0x0E;  // AAF_BITSHIFT [7:4], DELTSQR [11:8] [3:0]
constexpr uint8_t B1_INTF_CONFIG4         = 0x7A;
constexpr uint8_t B1_INTF_CONFIG5         = 0x7B;  // PIN9_FUNCTION [2:1]
constexpr uint8_t B1_INTF_CONFIG6         = 0x7C;  // I3C enables

// ------------------------------------------------------------------ Bank 2
constexpr uint8_t B2_ACCEL_CONFIG_STATIC2 = 0x03;  // AAF_DELT [6:1], AAF_DIS (bit0)
constexpr uint8_t B2_ACCEL_CONFIG_STATIC3 = 0x04;  // AAF_DELTSQR [7:0]
constexpr uint8_t B2_ACCEL_CONFIG_STATIC4 = 0x05;  // AAF_BITSHIFT [7:4], DELTSQR [11:8] [3:0]

// ------------------------------------------------------------------ Bank 4
constexpr uint8_t B4_ACCEL_WOM_X_THR = 0x4A;
constexpr uint8_t B4_ACCEL_WOM_Y_THR = 0x4B;
constexpr uint8_t B4_ACCEL_WOM_Z_THR = 0x4C;
constexpr uint8_t B4_OFFSET_USER0    = 0x77;  // ..0x7F, nine registers

// ------------------------------------------------------------------ Bit fields
constexpr uint8_t DEVICE_CONFIG_SOFT_RESET = 1 << 0;

constexpr uint8_t INT_STATUS_RESET_DONE = 1 << 4;
constexpr uint8_t INT_STATUS_DATA_RDY   = 1 << 3;
constexpr uint8_t INT_STATUS_FIFO_THS   = 1 << 2;
constexpr uint8_t INT_STATUS_FIFO_FULL  = 1 << 1;

constexpr uint8_t SPR_FIFO_FLUSH = 1 << 1;

constexpr uint8_t INTF1_ACCEL_LP_CLK_RC = 1 << 3;
constexpr uint8_t INTF1_RTC_MODE        = 1 << 2;

constexpr uint8_t FIFO_MODE_STREAM = 1 << 6;  // FIFO_CONFIG[7:6] = 01

constexpr uint8_t FIFO1_RESUME_PARTIAL_RD = 1 << 6;
constexpr uint8_t FIFO1_HIRES_EN          = 1 << 4;
constexpr uint8_t FIFO1_TEMP_EN           = 1 << 2;
constexpr uint8_t FIFO1_GYRO_EN           = 1 << 1;
constexpr uint8_t FIFO1_ACCEL_EN          = 1 << 0;

constexpr uint8_t PWR_GYRO_SHIFT = 2;

constexpr uint8_t GYRO_STATIC2_AAF_DIS = 1 << 1;
constexpr uint8_t GYRO_STATIC2_NF_DIS  = 1 << 0;

constexpr uint8_t INTF6_I3C_SDR_EN = 1 << 0;
constexpr uint8_t INTF6_I3C_DDR_EN = 1 << 1;

}  // namespace reg
}  // namespace iim42652

namespace iim42652 {

// ---------------------------------------------------------------------------
// PIO SPI master (mode 0, MSB first, 8-bit frames, software chip select).
//
// Hand-assembled version of the pico-examples "spi_cpha0" program, so no .pio
// file or pioasm step is needed. One side-set bit drives SCK.
//     .side_set 1
//     .wrap_target
//       out pins, 1  side 0 [1]   ; 0x6101  present MOSI while SCK low
//       in  pins, 1  side 1 [1]   ; 0x5101  SCK rises, sample MISO
//     .wrap
// 4 PIO cycles per bit => SCK = pio_clk / 4. Autopull/autopush at 8 bits.
// ---------------------------------------------------------------------------
class PioSpi {
 public:
  // The datasheet allows 24 MHz, but MISO is sampled on the rising edge and the
  // device needs up to 21.5 ns after the falling edge (tVD.SDO), so cap at
  // 16 MHz to leave margin for pad and trace delay.
  bool init(PIO pio, uint sck, uint mosi, uint miso, uint cs, uint32_t baud = 8'000'000) {
    pio_ = pio;
    cs_ = cs;
    if (baud > 16'000'000) baud = 16'000'000;

    const int sm = pio_claim_unused_sm(pio_, false);
    if (sm < 0) return false;
    sm_ = static_cast<uint>(sm);
    if (!pio_can_add_program(pio_, &program_)) {
      pio_sm_unclaim(pio_, sm_);
      return false;
    }
    const uint offset = pio_add_program(pio_, &program_);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + 1);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, sck);
    sm_config_set_out_pins(&c, mosi, 1);
    sm_config_set_in_pins(&c, miso);
    sm_config_set_out_shift(&c, /*shift_right=*/false, /*autopull=*/true, 8);
    sm_config_set_in_shift(&c, /*shift_right=*/false, /*autopush=*/true, 8);
    sm_config_set_clkdiv(&c, static_cast<float>(clock_get_hz(clk_sys)) / (4.0f * static_cast<float>(baud)));

    pio_gpio_init(pio_, sck);
    pio_gpio_init(pio_, mosi);
    pio_gpio_init(pio_, miso);
    // The 2-cycle input synchroniser would make us sample MISO before the
    // device has driven it, so bypass it.
    pio_->input_sync_bypass |= 1u << miso;

    const uint32_t outMask = (1u << sck) | (1u << mosi);
    pio_sm_set_pindirs_with_mask(pio_, sm_, outMask, outMask | (1u << miso));
    pio_sm_set_pins_with_mask(pio_, sm_, 0, outMask);  // SCK and MOSI idle low

    gpio_init(cs_);
    gpio_set_dir(cs_, GPIO_OUT);
    gpio_put(cs_, 1);

    pio_sm_init(pio_, sm_, offset, &c);
    pio_sm_set_enabled(pio_, sm_, true);
    return true;
  }

  // Burst read: address byte with bit 7 set, then `len` data bytes.
  bool read(uint8_t reg, uint8_t* dst, size_t len) {
    const uint8_t addr = reg | 0x80;
    select();
    xfer(&addr, nullptr, 1);
    xfer(nullptr, dst, len);
    deselect();
    return true;
  }

  // The device only supports single-byte SPI writes (section 9.4), so a burst
  // becomes one chip-select cycle per byte at consecutive addresses.
  bool write(uint8_t reg, const uint8_t* src, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      const uint8_t buf[2] = {static_cast<uint8_t>((reg + i) & 0x7F), src[i]};
      select();
      xfer(buf, nullptr, 2);
      deselect();
    }
    return true;
  }

  void delayUs(uint32_t us) { sleep_us(us); }
  uint64_t nowUs() { return time_us_64(); }

 private:
  // Full duplex: every byte clocked out also clocks one byte in. `tx` null
  // sends zeros, `rx` null discards. TX and RX are interleaved so neither
  // 4-deep FIFO overflows.
  void xfer(const uint8_t* tx, uint8_t* rx, size_t n) {
    size_t txLeft = n, rxLeft = n;
    while (txLeft || rxLeft) {
      if (txLeft && !pio_sm_is_tx_fifo_full(pio_, sm_) && (rxLeft - txLeft < 4)) {
        const uint8_t b = tx ? *tx++ : 0;
        pio_sm_put(pio_, sm_, static_cast<uint32_t>(b) << 24);  // MSB-first: byte in top 8 bits
        --txLeft;
      }
      if (rxLeft && !pio_sm_is_rx_fifo_empty(pio_, sm_)) {
        const uint8_t v = static_cast<uint8_t>(pio_sm_get(pio_, sm_));
        if (rx) *rx++ = v;
        --rxLeft;
      }
    }
  }

  // tSU.CS = 39 ns, tHD.CS = 18 ns. After the last byte is pushed, the state
  // machine still holds SCK high for one more cycle, so wait briefly.
  static inline void pad(int n) { for (int i = 0; i < n; ++i) asm volatile("nop"); }
  void select() { gpio_put(cs_, 0); pad(8); }
  void deselect() { pad(12); gpio_put(cs_, 1); pad(8); }

  static constexpr uint16_t kInstr[2] = {0x6101, 0x5101};
  static constexpr pio_program_t program_ = {kInstr, 2, -1};

  PIO pio_ = nullptr;
  uint sm_ = 0;
  uint cs_ = 0;
};

}  // namespace iim42652

namespace iim42652 {

enum class Status : uint8_t {
  Ok = 0,
  BusError,
  WrongDevice,    // WHO_AM_I mismatch
  ResetFailed,    // RESET_DONE_INT never set after soft reset
  InvalidConfig,  // combination not supported by the datasheet
};

// ---------------------------------------------------------------------------
// Configuration enums (values are the register codes)
// ---------------------------------------------------------------------------
enum class AccelFs : uint8_t { G16 = 0, G8 = 1, G4 = 2, G2 = 3 };

enum class GyroFs : uint8_t {
  Dps2000 = 0, Dps1000, Dps500, Dps250, Dps125, Dps62_5, Dps31_25, Dps15_625
};

enum class Odr : uint8_t {
  Hz32000 = 1, Hz16000, Hz8000, Hz4000, Hz2000, Hz1000,  // 1..6
  Hz200, Hz100, Hz50, Hz25, Hz12_5,                        // 7..11
  Hz6_25, Hz3_125, Hz1_5625,                               // 12..14, accel low-power only
  Hz500 = 15
};

enum class AccelMode : uint8_t { Off = 0, LowPower = 2, LowNoise = 3 };
enum class GyroMode : uint8_t { Off = 0, Standby = 1, LowNoise = 3 };
enum class FilterOrder : uint8_t { First = 0, Second = 1, Third = 2 };

// Which events are routed to INT1 (INT_SOURCE0 bit positions).
constexpr uint8_t kInt1DataReady     = 1 << 3;
constexpr uint8_t kInt1FifoThreshold = 1 << 2;
constexpr uint8_t kInt1FifoFull      = 1 << 1;

struct Config {
  // Sensor modes / ranges / rates
  AccelMode accelMode = AccelMode::LowNoise;
  GyroMode  gyroMode  = GyroMode::LowNoise;
  AccelFs   accelFs   = AccelFs::G16;
  GyroFs    gyroFs    = GyroFs::Dps2000;
  Odr       accelOdr  = Odr::Hz1000;
  Odr       gyroOdr   = Odr::Hz1000;

  // UI filter (section 5.4). Bandwidth codes in low-noise mode: 0..7, 14, 15
  // (see GYRO_ACCEL_CONFIG0). 1 = max(400 Hz, ODR)/4, the power-on default.
  FilterOrder accelFilterOrder = FilterOrder::Second;
  FilterOrder gyroFilterOrder  = FilterOrder::Second;
  uint8_t accelFilterBw = 1;
  uint8_t gyroFilterBw  = 1;
  bool    accelLpAvg16x = true;  // low-power mode only: 16x averaging (else 1x)

  // Anti-alias filters (low-noise mode only). Hz = 0 keeps the power-on
  // default (accel 1163 Hz, gyro 585 Hz). Otherwise the nearest table entry
  // (42..3979 Hz) is used.
  bool     accelAafEnable = true;
  uint16_t accelAafHz     = 0;
  bool     gyroAafEnable  = true;
  uint16_t gyroAafHz      = 0;
  bool     gyroNotchEnable = true;  // factory trimmed notch, power-on default on

  // Interrupt pins
  bool    intActiveHigh = true;
  bool    intPushPull   = true;
  bool    intLatched    = false;
  uint8_t int1Sources   = 0;  // OR of kInt1*

  // FIFO
  bool     fifoEnable        = false;
  bool     fifoHiRes         = false;  // 20-bit packets (forces +-2000 dps / +-16 g)
  uint16_t fifoWatermarkBytes = 0;     // 0 = no watermark (max 2048)

  // External 31..50 kHz clock on pin 9 (CLKIN). 0 = internal clock.
  // ODRs then scale by (clk_kHz / 32). Pin 9 can no longer be used as INT2.
  uint16_t externalClockKhz = 0;
  bool     accelLpUsesRcClock = false;  // ACCEL_LP_CLK_SEL
};

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------
struct RawSample {
  int16_t temp, ax, ay, az, gx, gy, gz;
};

struct Sample {
  float ax_g = 0, ay_g = 0, az_g = 0;
  float gx_dps = 0, gy_dps = 0, gz_dps = 0;
  float temp_c = 0;
  uint16_t timestamp = 0;  // FIFO only; see timestampToUs()
  bool hasAccel = false;
  bool hasGyro  = false;
};

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------
class Device {
 public:
  explicit Device(PioSpi& bus) : bus_(bus) {}

  // Verify WHO_AM_I, soft reset, set up the host interface, apply `cfg` and
  // start the sensors.
  Status begin(const Config& cfg = Config{});

  // Re-apply a configuration. Sensors are turned off, registers programmed,
  // then sensors restarted (the datasheet forbids changing most registers
  // while the sensors run, section 12.9).
  Status configure(const Config& cfg);

  Status stop();    // sensors off (respects the 45 ms minimum gyro on-time)
  Status start();   // sensors back to the configured modes

  // ---- Polled data (sensor data registers)
  Status readRaw(RawSample& out);
  Status read(Sample& out);
  Status readIntStatus(uint8_t& status);  // INT_STATUS, clears on read

  // ---- FIFO
  Status fifoCount(uint16_t& bytes);
  Status fifoFlush();
  // Reads whole packets, up to `maxSamples`. `n` receives the number parsed.
  Status fifoRead(Sample* out, size_t maxSamples, size_t& n);
  Status fifoLostPackets(uint16_t& lost);
  size_t fifoPacketSize() const { return packetSize_; }
  // Convert a FIFO timestamp to microseconds (internal clock, TMST_RES = 0).
  static float timestampToUs(uint16_t ts) { return ts * (32.0f / 30.0f); }

  // ---- User offsets (applied in hardware, survive configure())
  Status setGyroOffsetDps(float x, float y, float z);   // +-64 dps, 1/32 dps steps
  Status setAccelOffsetG(float x, float y, float z);    // +-1 g, 0.5 mg steps

  // ---- Wake on motion. Accelerometer must be running; 50 Hz low-power is
  // the datasheet's reference setup. Thresholds have ~3.9 mg resolution.
  Status enableWakeOnMotion(float thresholdMg, bool routeToInt2 = false);
  Status disableWakeOnMotion();
  Status readWomStatus(uint8_t& status);  // INT_STATUS2, clears on read

  // ---- Helpers
  const Config& config() const { return cfg_; }
  float accelLsbPerG() const;
  float gyroLsbPerDps() const;
  static uint16_t nearestAafHz(uint16_t hz);

  // ---- Low level register access (bank-aware)
  Status readRegister(uint8_t bank, uint8_t addr, uint8_t& value);
  Status writeRegister(uint8_t bank, uint8_t addr, uint8_t value);
  Status modifyRegister(uint8_t bank, uint8_t addr, uint8_t mask, uint8_t value);

 private:
  static constexpr uint8_t kBankUnknown = 0xFF;

  Status selectBank(uint8_t bank);
  Status readBurst(uint8_t bank, uint8_t addr, uint8_t* dst, size_t len);
  Status setupInterface();
  Status validate(const Config& cfg) const;
  Status writePower(AccelMode a, GyroMode g);
  Status applyGyroAaf(bool enable, uint16_t hz);
  Status applyAccelAaf(bool enable, uint16_t hz);
  Status writeOffsets();
  bool parsePacket(const uint8_t* p, Sample& s) const;

  PioSpi& bus_;
  Config cfg_{};
  uint8_t bank_ = kBankUnknown;
  bool running_ = false;
  uint64_t gyroOnAtUs_ = 0;
  size_t packetSize_ = 16;
  int16_t gyroOff_[3] = {0, 0, 0};
  int16_t accelOff_[3] = {0, 0, 0};
};

}  // namespace iim42652

namespace iim42652 {
namespace {

#define IIM_TRY(expr)                                  \
  do {                                                 \
    const Status s_ = (expr);                          \
    if (s_ != Status::Ok) return s_;                   \
  } while (0)

// Anti-alias filter table, section 5.2. Index + 1 == AAF_DELT.
struct AafEntry {
  uint16_t bwHz;
  uint16_t deltsqr;
  uint8_t bitshift;
};
constexpr AafEntry kAaf[] = {
    {42, 1, 15},    {84, 4, 13},    {126, 9, 12},   {170, 16, 11},  {213, 25, 10},
    {258, 36, 10},  {303, 49, 9},   {348, 64, 9},   {394, 81, 9},   {441, 100, 8},
    {488, 122, 8},  {536, 144, 8},  {585, 170, 8},  {634, 196, 7},  {684, 224, 7},
    {734, 256, 7},  {785, 288, 7},  {837, 324, 7},  {890, 360, 6},  {943, 400, 6},
    {997, 440, 6},  {1051, 488, 6}, {1107, 528, 6}, {1163, 576, 6}, {1220, 624, 6},
    {1277, 680, 6}, {1336, 736, 5}, {1395, 784, 5}, {1454, 848, 5}, {1515, 896, 5},
    {1577, 960, 5}, {1639, 1024, 5}, {1702, 1088, 5}, {1766, 1152, 5}, {1830, 1232, 5},
    {1896, 1296, 5}, {1962, 1376, 4}, {2029, 1440, 4}, {2097, 1536, 4}, {2166, 1600, 4},
    {2235, 1696, 4}, {2306, 1760, 4}, {2377, 1856, 4}, {2449, 1952, 4}, {2522, 2016, 4},
    {2596, 2112, 4}, {2671, 2208, 4}, {2746, 2304, 4}, {2823, 2400, 4}, {2900, 2496, 4},
    {2978, 2592, 4}, {3057, 2720, 4}, {3137, 2816, 3}, {3217, 2944, 3}, {3299, 3008, 3},
    {3381, 3136, 3}, {3464, 3264, 3}, {3548, 3392, 3}, {3633, 3456, 3}, {3718, 3584, 3},
    {3805, 3712, 3}, {3892, 3840, 3}, {3979, 3968, 3},
};
static_assert(sizeof(kAaf) / sizeof(kAaf[0]) == 63, "AAF table must have 63 entries");

// Sensitivity scale factors, Tables 1 and 2.
constexpr float kGyroLsbPerDps[8] = {16.4f, 32.8f, 65.5f, 131.0f, 262.0f, 524.3f, 1048.6f, 2097.2f};
constexpr float kAccelLsbPerG[4]  = {2048.0f, 4096.0f, 8192.0f, 16384.0f};

// FIFO 20-bit packets always use these (section 6.1).
constexpr float kHiResGyroLsbPerDps = 131.0f;
constexpr float kHiResAccelLsbPerG  = 8192.0f;

constexpr float kTempLsbPerC_Reg  = 132.48f;
constexpr float kTempLsbPerC_Fifo = 2.07f;

constexpr uint32_t kGyroMinOnUs = 45000;  // PWR_MGMT0: keep gyro on >= 45 ms

bool odrIsLowNoiseValid(Odr o) {
  const uint8_t c = static_cast<uint8_t>(o);
  return (c >= 1 && c <= 11) || c == 15;
}
bool odrIsLowPowerValid(Odr o) {
  const uint8_t c = static_cast<uint8_t>(o);
  return c >= 7 && c <= 15;  // 200 Hz down to 1.5625 Hz, plus 500 Hz
}
bool odrAtLeast4k(Odr o) { return static_cast<uint8_t>(o) >= 1 && static_cast<uint8_t>(o) <= 4; }
bool lnFilterBwValid(uint8_t bw) { return bw <= 7 || bw == 14 || bw == 15; }

int16_t be16(const uint8_t* p) { return static_cast<int16_t>((p[0] << 8) | p[1]); }

int32_t signExtend20(uint8_t hi, uint8_t mid, uint8_t nib) {
  int32_t v = (static_cast<int32_t>(hi) << 12) | (static_cast<int32_t>(mid) << 4) | (nib & 0x0F);
  if (v & 0x80000) v -= 0x100000;
  return v;
}

size_t aafIndex(uint16_t hz) {
  size_t best = 0;
  int bestDiff = 1 << 30;
  for (size_t i = 0; i < sizeof(kAaf) / sizeof(kAaf[0]); ++i) {
    const int d = std::abs(static_cast<int>(kAaf[i].bwHz) - static_cast<int>(hz));
    if (d < bestDiff) { bestDiff = d; best = i; }
  }
  return best;
}

int16_t clamp12(long v) { return static_cast<int16_t>(std::max(-2048L, std::min(2047L, v))); }

}  // namespace

// ===========================================================================
// Low-level register access
// ===========================================================================
Status Device::selectBank(uint8_t bank) {
  if (bank_ == bank) return Status::Ok;
  const uint8_t v = bank & 0x07;
  if (!bus_.write(reg::REG_BANK_SEL, &v, 1)) return Status::BusError;
  bank_ = bank;
  return Status::Ok;
}

Status Device::readBurst(uint8_t bank, uint8_t addr, uint8_t* dst, size_t len) {
  IIM_TRY(selectBank(bank));
  return bus_.read(addr, dst, len) ? Status::Ok : Status::BusError;
}

Status Device::readRegister(uint8_t bank, uint8_t addr, uint8_t& value) {
  return readBurst(bank, addr, &value, 1);
}

Status Device::writeRegister(uint8_t bank, uint8_t addr, uint8_t value) {
  IIM_TRY(selectBank(bank));
  return bus_.write(addr, &value, 1) ? Status::Ok : Status::BusError;
}

Status Device::modifyRegister(uint8_t bank, uint8_t addr, uint8_t mask, uint8_t value) {
  uint8_t cur = 0;
  IIM_TRY(readRegister(bank, addr, cur));
  return writeRegister(bank, addr, static_cast<uint8_t>((cur & ~mask) | (value & mask)));
}

// ===========================================================================
// Bring-up
// ===========================================================================
Status Device::begin(const Config& cfg) {
  bank_ = kBankUnknown;
  running_ = false;

  bus_.delayUs(1000);  // register access is valid 1 ms after power-up (Table 4)
  IIM_TRY(selectBank(0));

  uint8_t id = 0;
  IIM_TRY(readRegister(0, reg::WHO_AM_I, id));
  if (id != reg::kWhoAmIValue) return Status::WrongDevice;

  IIM_TRY(writeRegister(0, reg::DEVICE_CONFIG, reg::DEVICE_CONFIG_SOFT_RESET));
  bus_.delayUs(2000);  // datasheet: wait 1 ms before any other access
  bank_ = kBankUnknown;
  IIM_TRY(selectBank(0));

  uint8_t st = 0;
  IIM_TRY(readRegister(0, reg::INT_STATUS, st));  // also clears the flag
  if (!(st & reg::INT_STATUS_RESET_DONE)) return Status::ResetFailed;

  IIM_TRY(setupInterface());
  return configure(cfg);
}

// Host interface settings from section 12.3. The power-on values (reset values
// of DRIVE_CONFIG = 0x05 and INTF_CONFIG6 = 0x5F) already match the datasheet's
// "SPI driver setting", so nothing else needs changing for SPI.
Status Device::setupInterface() {
  // Big-endian sensor data and FIFO count in bytes (the defaults), no
  // "hold last data" mode so invalid FIFO samples are flagged by value.
  IIM_TRY(modifyRegister(0, reg::INTF_CONFIG0, 0xF0, 0x30));

  return Status::Ok;
}

// ===========================================================================
// Configuration
// ===========================================================================
Status Device::validate(const Config& c) const {
  if (c.gyroMode != GyroMode::Off && !odrIsLowNoiseValid(c.gyroOdr)) return Status::InvalidConfig;

  if (c.accelMode == AccelMode::LowNoise && !odrIsLowNoiseValid(c.accelOdr))
    return Status::InvalidConfig;  // LN mode needs >= 12.5 Hz
  if (c.accelMode == AccelMode::LowPower && !odrIsLowPowerValid(c.accelOdr))
    return Status::InvalidConfig;  // LP mode needs <= 500 Hz

  if (c.accelMode == AccelMode::LowNoise && !lnFilterBwValid(c.accelFilterBw))
    return Status::InvalidConfig;
  if (c.gyroMode != GyroMode::Off && !lnFilterBwValid(c.gyroFilterBw)) return Status::InvalidConfig;

  if (c.accelFilterOrder > FilterOrder::Third || c.gyroFilterOrder > FilterOrder::Third)
    return Status::InvalidConfig;

  if (c.externalClockKhz != 0 && (c.externalClockKhz < 31 || c.externalClockKhz > 50))
    return Status::InvalidConfig;
  if (c.fifoWatermarkBytes > 2048) return Status::InvalidConfig;
  if (c.fifoWatermarkBytes != 0 && !c.fifoEnable) return Status::InvalidConfig;
  if ((c.int1Sources & (kInt1FifoThreshold | kInt1FifoFull)) && !c.fifoEnable)
    return Status::InvalidConfig;
  if ((c.int1Sources & kInt1FifoThreshold) && c.fifoWatermarkBytes == 0)
    return Status::InvalidConfig;  // FIFO_WM must be non-zero for this source
  return Status::Ok;
}

Status Device::writePower(AccelMode a, GyroMode g) {
  const uint8_t v = static_cast<uint8_t>((static_cast<uint8_t>(g) << reg::PWR_GYRO_SHIFT) |
                                         static_cast<uint8_t>(a));
  IIM_TRY(writeRegister(0, reg::PWR_MGMT0, v));
  bus_.delayUs(250);  // no register writes for 200 us after a mode change
  return Status::Ok;
}

Status Device::stop() {
  if (!running_) return Status::Ok;
  if (cfg_.gyroMode != GyroMode::Off) {
    const uint64_t elapsed = bus_.nowUs() - gyroOnAtUs_;
    if (elapsed < kGyroMinOnUs) bus_.delayUs(static_cast<uint32_t>(kGyroMinOnUs - elapsed));
  }
  IIM_TRY(writePower(AccelMode::Off, GyroMode::Off));
  running_ = false;
  return Status::Ok;
}

Status Device::start() {
  if (running_) return Status::Ok;
  IIM_TRY(writePower(cfg_.accelMode, cfg_.gyroMode));
  if (cfg_.gyroMode != GyroMode::Off) gyroOnAtUs_ = bus_.nowUs();
  running_ = true;
  // Gyro drive ready: 30 ms, accelerometer valid data: 10 ms (Tables 1, 2).
  bus_.delayUs(cfg_.gyroMode != GyroMode::Off ? 30000 : 10000);
  return Status::Ok;
}

Status Device::configure(const Config& c) {
  IIM_TRY(validate(c));
  IIM_TRY(stop());
  cfg_ = c;

  // --- Clocking (INTF_CONFIG1 / INTF_CONFIG5)
  uint8_t intf1 = 0;
  if (c.accelLpUsesRcClock) intf1 |= reg::INTF1_ACCEL_LP_CLK_RC;
  if (c.externalClockKhz) intf1 |= reg::INTF1_RTC_MODE;
  IIM_TRY(modifyRegister(0, reg::INTF_CONFIG1, reg::INTF1_ACCEL_LP_CLK_RC | reg::INTF1_RTC_MODE, intf1));
  IIM_TRY(modifyRegister(1, reg::B1_INTF_CONFIG5, 0x06, c.externalClockKhz ? (2 << 1) : 0));

  // --- Interrupt pins. INT2 mirrors the INT1 electrical settings.
  uint8_t intCfg = 0;
  if (c.intActiveHigh) intCfg |= 1 << 0;
  if (c.intPushPull) intCfg |= 1 << 1;
  if (c.intLatched) intCfg |= 1 << 2;
  IIM_TRY(writeRegister(0, reg::INT_CONFIG, static_cast<uint8_t>(intCfg | (intCfg << 3))));

  // INT_ASYNC_RESET must be 0 for proper INT pin operation (section 12.7).
  // ODR >= 4 kHz requires 8 us pulses and no de-assert time.
  const bool fast = (c.accelMode != AccelMode::Off && odrAtLeast4k(c.accelOdr)) ||
                    (c.gyroMode != GyroMode::Off && odrAtLeast4k(c.gyroOdr));
  IIM_TRY(writeRegister(0, reg::INT_CONFIG1, fast ? ((1 << 6) | (1 << 5)) : 0));
  IIM_TRY(writeRegister(0, reg::INT_SOURCE0, c.int1Sources));

  // --- Full scale and ODR
  IIM_TRY(writeRegister(0, reg::GYRO_CONFIG0,
                        static_cast<uint8_t>((static_cast<uint8_t>(c.gyroFs) << 5) |
                                             static_cast<uint8_t>(c.gyroOdr))));
  IIM_TRY(writeRegister(0, reg::ACCEL_CONFIG0,
                        static_cast<uint8_t>((static_cast<uint8_t>(c.accelFs) << 5) |
                                             static_cast<uint8_t>(c.accelOdr))));

  // --- UI filters
  IIM_TRY(modifyRegister(0, reg::GYRO_CONFIG1, 0x0C, static_cast<uint8_t>(c.gyroFilterOrder) << 2));
  IIM_TRY(modifyRegister(0, reg::ACCEL_CONFIG1, 0x18, static_cast<uint8_t>(c.accelFilterOrder) << 3));
  const uint8_t accelBw = (c.accelMode == AccelMode::LowPower) ? (c.accelLpAvg16x ? 6 : 1) : c.accelFilterBw;
  IIM_TRY(writeRegister(0, reg::GYRO_ACCEL_CONFIG0,
                        static_cast<uint8_t>(((accelBw & 0x0F) << 4) | (c.gyroFilterBw & 0x0F))));

  // --- Anti-alias and notch filters (bank 1 / 2)
  IIM_TRY(applyGyroAaf(c.gyroAafEnable, c.gyroAafHz));
  IIM_TRY(modifyRegister(1, reg::B1_GYRO_CONFIG_STATIC2, reg::GYRO_STATIC2_NF_DIS,
                         c.gyroNotchEnable ? 0 : reg::GYRO_STATIC2_NF_DIS));
  IIM_TRY(applyAccelAaf(c.accelAafEnable, c.accelAafHz));

  // --- FIFO
  if (c.fifoEnable) {
    const bool a = c.accelMode != AccelMode::Off;
    const bool g = c.gyroMode != GyroMode::Off;
    uint8_t f1 = reg::FIFO1_RESUME_PARTIAL_RD | reg::FIFO1_TEMP_EN;  // partial reads: we read in chunks
    if (a) f1 |= reg::FIFO1_ACCEL_EN;
    if (g) f1 |= reg::FIFO1_GYRO_EN;
    if (c.fifoHiRes) f1 |= reg::FIFO1_HIRES_EN;
    IIM_TRY(writeRegister(0, reg::FIFO_CONFIG1, f1));
    packetSize_ = c.fifoHiRes ? 20 : ((a && g) ? 16 : 8);
    if (c.fifoWatermarkBytes) {
      IIM_TRY(writeRegister(0, reg::FIFO_CONFIG2, c.fifoWatermarkBytes & 0xFF));
      IIM_TRY(writeRegister(0, reg::FIFO_CONFIG3, (c.fifoWatermarkBytes >> 8) & 0x0F));
    }
    IIM_TRY(writeRegister(0, reg::FIFO_CONFIG, reg::FIFO_MODE_STREAM));
    IIM_TRY(writeRegister(0, reg::SIGNAL_PATH_RESET, reg::SPR_FIFO_FLUSH));
  } else {
    IIM_TRY(writeRegister(0, reg::FIFO_CONFIG, 0));  // bypass
    IIM_TRY(writeRegister(0, reg::FIFO_CONFIG1, 0));
  }

  IIM_TRY(writeOffsets());
  return start();
}

// ---------------------------------------------------------------------------
uint16_t Device::nearestAafHz(uint16_t hz) { return kAaf[aafIndex(hz)].bwHz; }

Status Device::applyGyroAaf(bool enable, uint16_t hz) {
  IIM_TRY(modifyRegister(1, reg::B1_GYRO_CONFIG_STATIC2, reg::GYRO_STATIC2_AAF_DIS,
                         enable ? 0 : reg::GYRO_STATIC2_AAF_DIS));
  if (!enable || hz == 0) return Status::Ok;
  const size_t i = aafIndex(hz);
  const AafEntry& e = kAaf[i];
  IIM_TRY(modifyRegister(1, reg::B1_GYRO_CONFIG_STATIC3, 0x3F, static_cast<uint8_t>(i + 1)));
  IIM_TRY(writeRegister(1, reg::B1_GYRO_CONFIG_STATIC4, e.deltsqr & 0xFF));
  return writeRegister(1, reg::B1_GYRO_CONFIG_STATIC5,
                       static_cast<uint8_t>((e.bitshift << 4) | ((e.deltsqr >> 8) & 0x0F)));
}

Status Device::applyAccelAaf(bool enable, uint16_t hz) {
  const uint8_t dis = enable ? 0 : 1;
  if (!enable || hz == 0)
    return modifyRegister(2, reg::B2_ACCEL_CONFIG_STATIC2, 0x01, dis);
  const size_t i = aafIndex(hz);
  const AafEntry& e = kAaf[i];
  IIM_TRY(modifyRegister(2, reg::B2_ACCEL_CONFIG_STATIC2, 0x7F, static_cast<uint8_t>(((i + 1) << 1) | dis)));
  IIM_TRY(writeRegister(2, reg::B2_ACCEL_CONFIG_STATIC3, e.deltsqr & 0xFF));
  return writeRegister(2, reg::B2_ACCEL_CONFIG_STATIC4,
                       static_cast<uint8_t>((e.bitshift << 4) | ((e.deltsqr >> 8) & 0x0F)));
}

// ===========================================================================
// Scaling helpers
// ===========================================================================
float Device::accelLsbPerG() const { return kAccelLsbPerG[static_cast<uint8_t>(cfg_.accelFs) & 3]; }
float Device::gyroLsbPerDps() const { return kGyroLsbPerDps[static_cast<uint8_t>(cfg_.gyroFs) & 7]; }

// ===========================================================================
// Polled readout
// ===========================================================================
Status Device::readRaw(RawSample& o) {
  uint8_t b[14];
  IIM_TRY(readBurst(0, reg::TEMP_DATA1, b, sizeof(b)));
  o.temp = be16(&b[0]);
  o.ax = be16(&b[2]);  o.ay = be16(&b[4]);  o.az = be16(&b[6]);
  o.gx = be16(&b[8]);  o.gy = be16(&b[10]); o.gz = be16(&b[12]);
  return Status::Ok;
}

Status Device::read(Sample& s) {
  RawSample r;
  IIM_TRY(readRaw(r));
  const float aS = 1.0f / accelLsbPerG();
  const float gS = 1.0f / gyroLsbPerDps();
  // -32768 is the power-on value before the first sample (section 12.8).
  s.hasAccel = r.ax != -32768 && cfg_.accelMode != AccelMode::Off;
  s.hasGyro  = r.gx != -32768 && cfg_.gyroMode != GyroMode::Off;
  s.ax_g = r.ax * aS;  s.ay_g = r.ay * aS;  s.az_g = r.az * aS;
  s.gx_dps = r.gx * gS; s.gy_dps = r.gy * gS; s.gz_dps = r.gz * gS;
  s.temp_c = r.temp / kTempLsbPerC_Reg + 25.0f;
  s.timestamp = 0;
  return Status::Ok;
}

Status Device::readIntStatus(uint8_t& status) { return readRegister(0, reg::INT_STATUS, status); }

// ===========================================================================
// FIFO
// ===========================================================================
Status Device::fifoCount(uint16_t& bytes) {
  uint8_t b[2];  // FIFO_COUNTH then FIFO_COUNTL; reading COUNTL latches both
  IIM_TRY(readBurst(0, reg::FIFO_COUNTH, b, 2));
  bytes = static_cast<uint16_t>((b[0] << 8) | b[1]);
  return Status::Ok;
}

Status Device::fifoFlush() { return writeRegister(0, reg::SIGNAL_PATH_RESET, reg::SPR_FIFO_FLUSH); }

Status Device::fifoLostPackets(uint16_t& lost) {
  uint8_t b[2];
  IIM_TRY(readBurst(0, reg::FIFO_LOST_PKT0, b, 2));
  lost = static_cast<uint16_t>((b[0] << 8) | b[1]);  // PKT0 = high byte (section 14)
  return Status::Ok;
}

bool Device::parsePacket(const uint8_t* p, Sample& s) const {
  const uint8_t h = p[0];
  if (h & 0x80) return false;  // HEADER_MSG: FIFO empty
  const bool hdrAccel = h & 0x40;
  const bool hdrGyro = h & 0x20;

  if (cfg_.fifoHiRes) {  // 20-byte packet (Packet 4)
    const int32_t ax = signExtend20(p[1], p[2], p[17] >> 4);
    const int32_t ay = signExtend20(p[3], p[4], p[18] >> 4);
    const int32_t az = signExtend20(p[5], p[6], p[19] >> 4);
    const int32_t gx = signExtend20(p[7], p[8], p[17] & 0x0F);
    const int32_t gy = signExtend20(p[9], p[10], p[18] & 0x0F);
    const int32_t gz = signExtend20(p[11], p[12], p[19] & 0x0F);
    constexpr int32_t kInvalid20 = -524288;
    s.hasAccel = hdrAccel && ax != kInvalid20;
    s.hasGyro = hdrGyro && gx != kInvalid20;
    s.ax_g = ax / kHiResAccelLsbPerG; s.ay_g = ay / kHiResAccelLsbPerG; s.az_g = az / kHiResAccelLsbPerG;
    s.gx_dps = gx / kHiResGyroLsbPerDps; s.gy_dps = gy / kHiResGyroLsbPerDps; s.gz_dps = gz / kHiResGyroLsbPerDps;
    s.temp_c = be16(&p[13]) / kTempLsbPerC_Fifo + 25.0f;
    s.timestamp = static_cast<uint16_t>((p[15] << 8) | p[16]);
    return true;
  }

  const float aS = 1.0f / accelLsbPerG();
  const float gS = 1.0f / gyroLsbPerDps();
  s.hasAccel = s.hasGyro = false;
  s.timestamp = 0;

  if (hdrAccel && hdrGyro) {  // 16-byte packet (Packet 3)
    const int16_t ax = be16(&p[1]), ay = be16(&p[3]), az = be16(&p[5]);
    const int16_t gx = be16(&p[7]), gy = be16(&p[9]), gz = be16(&p[11]);
    s.hasAccel = ax != -32768;
    s.hasGyro = gx != -32768;
    s.ax_g = ax * aS; s.ay_g = ay * aS; s.az_g = az * aS;
    s.gx_dps = gx * gS; s.gy_dps = gy * gS; s.gz_dps = gz * gS;
    s.temp_c = static_cast<int8_t>(p[13]) / kTempLsbPerC_Fifo + 25.0f;
    s.timestamp = static_cast<uint16_t>((p[14] << 8) | p[15]);
  } else if (hdrAccel) {  // Packet 1
    const int16_t ax = be16(&p[1]);
    s.hasAccel = ax != -32768;
    s.ax_g = ax * aS; s.ay_g = be16(&p[3]) * aS; s.az_g = be16(&p[5]) * aS;
    s.temp_c = static_cast<int8_t>(p[7]) / kTempLsbPerC_Fifo + 25.0f;
  } else if (hdrGyro) {  // Packet 2
    const int16_t gx = be16(&p[1]);
    s.hasGyro = gx != -32768;
    s.gx_dps = gx * gS; s.gy_dps = be16(&p[3]) * gS; s.gz_dps = be16(&p[5]) * gS;
    s.temp_c = static_cast<int8_t>(p[7]) / kTempLsbPerC_Fifo + 25.0f;
  } else {
    return false;
  }
  return true;
}

Status Device::fifoRead(Sample* out, size_t maxSamples, size_t& n) {
  n = 0;
  if (!cfg_.fifoEnable || packetSize_ == 0) return Status::InvalidConfig;

  uint16_t bytes = 0;
  IIM_TRY(fifoCount(bytes));
  size_t packets = std::min<size_t>(bytes / packetSize_, maxSamples);

  uint8_t buf[240];  // multiple of 8, 16 and 20 bytes
  const size_t perChunk = sizeof(buf) / packetSize_;
  while (packets > 0) {
    const size_t chunk = std::min(packets, perChunk);
    IIM_TRY(readBurst(0, reg::FIFO_DATA, buf, chunk * packetSize_));
    for (size_t i = 0; i < chunk; ++i) {
      if (!parsePacket(&buf[i * packetSize_], out[n])) return Status::Ok;  // empty / malformed
      ++n;
    }
    packets -= chunk;
  }
  return Status::Ok;
}

// ===========================================================================
// User offsets (bank 4, section 5.3)
// ===========================================================================
Status Device::writeOffsets() {
  const uint16_t gx = gyroOff_[0] & 0x0FFF, gy = gyroOff_[1] & 0x0FFF, gz = gyroOff_[2] & 0x0FFF;
  const uint16_t ax = accelOff_[0] & 0x0FFF, ay = accelOff_[1] & 0x0FFF, az = accelOff_[2] & 0x0FFF;
  const uint8_t v[9] = {
      static_cast<uint8_t>(gx & 0xFF),
      static_cast<uint8_t>(((gy >> 8) << 4) | (gx >> 8)),
      static_cast<uint8_t>(gy & 0xFF),
      static_cast<uint8_t>(gz & 0xFF),
      static_cast<uint8_t>(((ax >> 8) << 4) | (gz >> 8)),
      static_cast<uint8_t>(ax & 0xFF),
      static_cast<uint8_t>(ay & 0xFF),
      static_cast<uint8_t>(((az >> 8) << 4) | (ay >> 8)),
      static_cast<uint8_t>(az & 0xFF),
  };
  IIM_TRY(selectBank(4));
  return bus_.write(reg::B4_OFFSET_USER0, v, sizeof(v)) ? Status::Ok : Status::BusError;
}

Status Device::setGyroOffsetDps(float x, float y, float z) {
  const bool wasRunning = running_;
  IIM_TRY(stop());
  gyroOff_[0] = clamp12(std::lroundf(x * 32.0f));
  gyroOff_[1] = clamp12(std::lroundf(y * 32.0f));
  gyroOff_[2] = clamp12(std::lroundf(z * 32.0f));
  IIM_TRY(writeOffsets());
  return wasRunning ? start() : Status::Ok;
}

Status Device::setAccelOffsetG(float x, float y, float z) {
  const bool wasRunning = running_;
  IIM_TRY(stop());
  accelOff_[0] = clamp12(std::lroundf(x * 2000.0f));
  accelOff_[1] = clamp12(std::lroundf(y * 2000.0f));
  accelOff_[2] = clamp12(std::lroundf(z * 2000.0f));
  IIM_TRY(writeOffsets());
  return wasRunning ? start() : Status::Ok;
}

// ===========================================================================
// Wake on motion (section 8.7)
// ===========================================================================
Status Device::enableWakeOnMotion(float thresholdMg, bool routeToInt2) {
  if (cfg_.accelMode == AccelMode::Off) return Status::InvalidConfig;
  const long t = std::lroundf(thresholdMg * 256.0f / 1000.0f);  // 1 g / 256 per LSB
  const uint8_t th = static_cast<uint8_t>(std::max(0L, std::min(255L, t)));

  const bool wasRunning = running_;
  IIM_TRY(stop());
  IIM_TRY(writeRegister(4, reg::B4_ACCEL_WOM_X_THR, th));
  IIM_TRY(writeRegister(4, reg::B4_ACCEL_WOM_Y_THR, th));
  IIM_TRY(writeRegister(4, reg::B4_ACCEL_WOM_Z_THR, th));
  IIM_TRY(modifyRegister(0, routeToInt2 ? reg::INT_SOURCE4 : reg::INT_SOURCE1, 0x07, 0x07));
  if (wasRunning) IIM_TRY(start());
  bus_.delayUs(50000);  // datasheet: wait 50 ms before enabling WoM
  // WOM_INT_MODE = 0 (OR of axes), WOM_MODE = 1 (compare to previous sample), SMD_MODE = 1 (WoM)
  return writeRegister(0, reg::SMD_CONFIG, (0 << 3) | (1 << 2) | 0x01);
}

Status Device::disableWakeOnMotion() {
  IIM_TRY(writeRegister(0, reg::SMD_CONFIG, 0));
  IIM_TRY(modifyRegister(0, reg::INT_SOURCE1, 0x07, 0));
  return modifyRegister(0, reg::INT_SOURCE4, 0x07, 0);
}

Status Device::readWomStatus(uint8_t& status) { return readRegister(0, reg::INT_STATUS2, status); }

}  // namespace iim42652

#ifdef IIM42652_EXAMPLE_MAIN
// Demo: data-ready interrupt on INT1 (GP20), 200 Hz readout over PIO SPI.
using namespace iim42652;
static volatile bool g_event = false;
static void onInt(uint, uint32_t) { g_event = true; }

int main() {
  stdio_init_all();
  sleep_ms(2000);

  PioSpi bus;
  if (!bus.init(pio0, /*sck=*/18, /*mosi=*/19, /*miso=*/16, /*cs=*/17, 8'000'000)) {
    printf("PIO init failed\n");
    while (true) tight_loop_contents();
  }
  Device imu(bus);

  Config cfg;
  cfg.accelFs = AccelFs::G4;
  cfg.gyroFs = GyroFs::Dps500;
  cfg.accelOdr = Odr::Hz200;
  cfg.gyroOdr = Odr::Hz200;
  cfg.int1Sources = kInt1DataReady;

  const Status s = imu.begin(cfg);
  if (s != Status::Ok) {
    printf("IIM-42652 init failed: %d\n", static_cast<int>(s));
    while (true) tight_loop_contents();
  }
  gpio_set_irq_enabled_with_callback(20, GPIO_IRQ_EDGE_RISE, true, &onInt);

  while (true) {
    if (!g_event) { tight_loop_contents(); continue; }
    g_event = false;
    uint8_t st = 0;
    imu.readIntStatus(st);
    Sample smp;
    if (imu.read(smp) == Status::Ok)
      printf("a=%.3f %.3f %.3f g  w=%.2f %.2f %.2f dps  T=%.1f C\n", smp.ax_g, smp.ay_g, smp.az_g,
             smp.gx_dps, smp.gy_dps, smp.gz_dps, smp.temp_c);
  }
}
#endif