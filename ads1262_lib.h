/**
 * @file ads1262_lib.h
 * @brief Library for ADS1262 32-bit ADC on Linux spidev (Orange Pi, Raspberry Pi)
 * @version 1.0
 * @date 2026-10-06
 *
 * - Device handle (ads1262_t), any number of devices, no global state
 * - Optional DRDY pin on GPIO (kernel GPIO uAPI); without it the ADC1 bit of the
 *   status byte that comes with every RDATA1 read is polled
 * - Any input combination AIN0-AIN9 / AINCOM, internal temperature sensor and supply monitors
 * - Digital filter, chop mode, reference selection, IDAC excitation currents
 * - Raw 32-bit codes + ads1262_to_volts(), every read checked by the checksum byte
 * - Gain, data rate, filter and reference setters run offset self-calibration
 * - No library dependencies: Linux spidev + GPIO character device (kernel >= 5.10)
 * - ADS1263 works too, but only its ADC1
 *
 * Wiring (Orange Pi 5 / Raspberry Pi header):
 *  ADS1262     Header
 *  ---------------------
 *  CS          CE0   (24)
 *  DOUT/DRDY   MISO  (21)
 *  DIN         MOSI  (19)
 *  SCLK        SCLK  (23)
 *  DGND, AVSS  GND   (6,9,14,20,25,30,34,39)
 *  AVDD        5V    (2)
 *  DVDD        3.3V  (1)
 *  START       GND (conversions are started by commands)
 *  RESET/PWDN  DVDD
 *  DRDY        any GPIO, optional (see ads1262_config_t.drdy_chip)
 */

#ifndef ADS1262_LIB_H
#define ADS1262_LIB_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Return codes */
#define ADS1262_OK                   0  /**< Operation successful */
#define ADS1262_ERROR_PARAMETER     -1  /**< Invalid parameter */
#define ADS1262_ERROR_COMMUNICATION -2  /**< SPI or GPIO error, or no ADS1262 found */
#define ADS1262_ERROR_TIMEOUT       -3  /**< No new data in time */
#define ADS1262_ERROR_OVERRUN       -4  /**< read_stream() couldn't keep up, conversions skipped */
#define ADS1262_ERROR_CHECKSUM      -5  /**< Conversion data corrupted on the bus */

/* Registers (ADC2 registers 15h-1Ah of ADS1263 are not supported) */
#define ADS1262_REG_ID           0x00   /**< Device identification */
#define ADS1262_REG_POWER        0x01   /**< Reset flag, level shift, internal reference */
#define ADS1262_REG_INTERFACE    0x02   /**< Status and checksum bytes, SPI time-out */
#define ADS1262_REG_MODE0        0x03   /**< Reference reversal, run mode, chop, delay */
#define ADS1262_REG_MODE1        0x04   /**< Digital filter, sensor bias */
#define ADS1262_REG_MODE2        0x05   /**< PGA bypass, gain, data rate */
#define ADS1262_REG_INPMUX       0x06   /**< Input multiplexer */
#define ADS1262_REG_OFCAL0       0x07   /**< Offset calibration, 24 bits LSB first */
#define ADS1262_REG_OFCAL1       0x08
#define ADS1262_REG_OFCAL2       0x09
#define ADS1262_REG_FSCAL0       0x0A   /**< Full-scale calibration, 24 bits LSB first */
#define ADS1262_REG_FSCAL1       0x0B
#define ADS1262_REG_FSCAL2       0x0C
#define ADS1262_REG_IDACMUX      0x0D   /**< IDAC pins */
#define ADS1262_REG_IDACMAG      0x0E   /**< IDAC currents */
#define ADS1262_REG_REFMUX       0x0F   /**< Reference multiplexer */
#define ADS1262_REG_TDACP        0x10   /**< Test DAC positive */
#define ADS1262_REG_TDACN        0x11   /**< Test DAC negative */
#define ADS1262_REG_GPIOCON      0x12   /**< GPIO connection */
#define ADS1262_REG_GPIODIR      0x13   /**< GPIO direction */
#define ADS1262_REG_GPIODAT      0x14   /**< GPIO data */

/* Status byte (ads1262_t.status), datasheet table 9-18 */
#define ADS1262_STATUS_ADC1      0x40   /**< New ADC1 data since the last read */
#define ADS1262_STATUS_EXTCLK    0x20   /**< External clock */
#define ADS1262_STATUS_REF_ALM   0x10   /**< Reference below 0.4 V */
#define ADS1262_STATUS_PGAL_ALM  0x08   /**< PGA output below AVSS + 0.2 V */
#define ADS1262_STATUS_PGAH_ALM  0x04   /**< PGA output above AVDD - 0.2 V */
#define ADS1262_STATUS_PGAD_ALM  0x02   /**< PGA differential output beyond +-105 % FS */
#define ADS1262_STATUS_RESET     0x01   /**< Device was reset since ads1262_open() */

/* Commands */
#define ADS1262_CMD_NOP          0x00   /**< No operation */
#define ADS1262_CMD_RESET        0x06   /**< Reset */
#define ADS1262_CMD_START1       0x08   /**< Start or restart ADC1 conversions */
#define ADS1262_CMD_STOP1        0x0A   /**< Stop ADC1 conversions */
#define ADS1262_CMD_RDATA1       0x12   /**< Read ADC1 data */
#define ADS1262_CMD_SYOCAL1      0x16   /**< System offset calibration (inputs shorted externally) */
#define ADS1262_CMD_SYGCAL1      0x17   /**< System gain calibration (full scale applied) */
#define ADS1262_CMD_SFOCAL1      0x19   /**< Self offset calibration */
#define ADS1262_CMD_RREG         0x20   /**< Read registers */
#define ADS1262_CMD_WREG         0x40   /**< Write registers */

/* Inputs for ads1262_set_input(); internal ones are selected as pos == neg, e.g. TEMP/TEMP */
enum {
    ADS1262_AIN0 = 0, ADS1262_AIN1, ADS1262_AIN2, ADS1262_AIN3, ADS1262_AIN4,
    ADS1262_AIN5, ADS1262_AIN6, ADS1262_AIN7, ADS1262_AIN8, ADS1262_AIN9,
    ADS1262_AINCOM,           /**< Common input for single-ended measurement */
    ADS1262_TEMP,             /**< Temperature sensor, see ads1262_to_celsius() */
    ADS1262_AVDD_MON,         /**< (AVDD - AVSS) / 4 */
    ADS1262_DVDD_MON,         /**< DVDD / 4 */
    ADS1262_TDAC,             /**< Test DAC */
    ADS1262_FLOAT             /**< Open connection */
};

/* IDAC pin for ads1262_set_idac(): ADS1262_AIN0..ADS1262_AINCOM, or not connected */
#define ADS1262_IDAC_NC 11

/* Data rates (DR register value) */
typedef enum {
    ADS1262_RATE_2_5 = 0, ADS1262_RATE_5, ADS1262_RATE_10, ADS1262_RATE_16_6,
    ADS1262_RATE_20, ADS1262_RATE_50, ADS1262_RATE_60, ADS1262_RATE_100,
    ADS1262_RATE_400, ADS1262_RATE_1200, ADS1262_RATE_2400, ADS1262_RATE_4800,
    ADS1262_RATE_7200, ADS1262_RATE_14400, ADS1262_RATE_19200, ADS1262_RATE_38400
} ads1262_rate_t;

/* Digital filter; FIR only at 2.5, 5, 10 and 20 SPS. 14400 SPS and up always use sinc5. */
typedef enum {
    ADS1262_FILTER_SINC1 = 0, ADS1262_FILTER_SINC2, ADS1262_FILTER_SINC3,
    ADS1262_FILTER_SINC4, ADS1262_FILTER_FIR
} ads1262_filter_t;

/* Gain */
typedef enum {
    ADS1262_GAIN_1 = 1, ADS1262_GAIN_2 = 2, ADS1262_GAIN_4 = 4,
    ADS1262_GAIN_8 = 8, ADS1262_GAIN_16 = 16, ADS1262_GAIN_32 = 32
} ads1262_gain_t;

/* Reference (REFMUX register value); set v_ref to match */
#define ADS1262_REF_INTERNAL   0x00   /**< Internal 2.5 V */
#define ADS1262_REF_AIN0_AIN1  0x09   /**< External, REFP = AIN0, REFN = AIN1 */
#define ADS1262_REF_AIN2_AIN3  0x12   /**< External, REFP = AIN2, REFN = AIN3 */
#define ADS1262_REF_AIN4_AIN5  0x1B   /**< External, REFP = AIN4, REFN = AIN5 */
#define ADS1262_REF_AVDD       0x24   /**< Analog supply AVDD - AVSS */

/* IDAC current (IDACMAG register value) */
typedef enum {
    ADS1262_IDAC_OFF = 0, ADS1262_IDAC_50UA, ADS1262_IDAC_100UA, ADS1262_IDAC_250UA,
    ADS1262_IDAC_500UA, ADS1262_IDAC_750UA, ADS1262_IDAC_1000UA, ADS1262_IDAC_1500UA,
    ADS1262_IDAC_2000UA, ADS1262_IDAC_2500UA, ADS1262_IDAC_3000UA
} ads1262_idac_t;

/* Configuration for ads1262_open() */
typedef struct {
    const char *spi_device;    /**< SPI device, e.g. "/dev/spidev0.0" */
    uint32_t spi_speed_hz;     /**< SCLK, max 8000000 */
    const char *drdy_chip;     /**< GPIO chip with DRDY, e.g. "/dev/gpiochip1"; NULL = poll status byte */
    unsigned int drdy_line;    /**< GPIO line offset of DRDY on drdy_chip */
    double v_ref;              /**< Reference voltage [V], 0.9-5.25 (2.5 for internal) */
    uint8_t refmux;            /**< ADS1262_REF_* */
    ads1262_rate_t rate;       /**< Data rate */
    ads1262_filter_t filter;   /**< Digital filter */
    ads1262_gain_t gain;       /**< PGA gain */
    uint8_t pos, neg;          /**< Inputs: ADS1262_AIN0..ADS1262_FLOAT */
    bool chop;                 /**< Input chop: lower offset and drift, half the data rate */
    uint32_t timeout_ms;       /**< Timeout on top of the expected conversion time, 1..3600000 */
} ads1262_config_t;

/* Device handle, owned by the caller; cfg always holds the current settings */
typedef struct {
    int spi_fd;
    int drdy_fd;               /**< -1 when DRDY pin is not used */
    ads1262_config_t cfg;
    uint8_t status;            /**< Status byte of the last conversion read (ADS1262_STATUS_*) */
} ads1262_t;

/**
 * Open SPI (and DRDY GPIO), reset the ADC, check it's there, apply configuration,
 * start conversions and run offset self-calibration (not with chop).
 * Call ads1262_close() before opening the same handle again, otherwise its file
 * descriptors leak (and the GPIO line stays busy).
 * @return ADS1262_OK or negative error; on error nothing stays open
 */
int ads1262_open(ads1262_t *dev, const ads1262_config_t *cfg);

/** Close SPI and GPIO file descriptors */
void ads1262_close(ads1262_t *dev);

/** Select inputs, e.g. AIN0/AIN1 differential, AIN3/AINCOM single-ended, TEMP/TEMP internal */
int ads1262_set_input(ads1262_t *dev, uint8_t pos, uint8_t neg);

/** Set PGA gain and self-calibrate */
int ads1262_set_gain(ads1262_t *dev, ads1262_gain_t gain);

/** Set data rate and self-calibrate (FIR filter allows only 2.5, 5, 10 and 20 SPS) */
int ads1262_set_rate(ads1262_t *dev, ads1262_rate_t rate);

/** Set digital filter and self-calibrate */
int ads1262_set_filter(ads1262_t *dev, ads1262_filter_t filter);

/** Select reference (ADS1262_REF_*) with its voltage v_ref and self-calibrate */
int ads1262_set_reference(ads1262_t *dev, uint8_t refmux, double v_ref);

/** Input chop on/off; turning it off self-calibrates (offset calibration is unused with chop) */
int ads1262_set_chop(ads1262_t *dev, bool on);

/** Connect IDAC 1 or 2 to pin (ADS1262_AIN0..ADS1262_AINCOM or ADS1262_IDAC_NC) with current */
int ads1262_set_idac(ads1262_t *dev, int idac, uint8_t pin, ads1262_idac_t current);

/**
 * Run calibration command ADS1262_CMD_SFOCAL1, SYOCAL1 or SYGCAL1 and wait for it.
 * Takes up to 9.2 s at 2.5 SPS (datasheet table 9-28). SFOCAL1 opens the inputs
 * meanwhile; for SYOCAL1 short the selected inputs, for SYGCAL1 apply full scale.
 */
int ads1262_calibrate(ads1262_t *dev, uint8_t cmd);

/** One fresh conversion with current settings (restart, wait, RDATA1), raw signed 32-bit code */
int ads1262_read(ads1262_t *dev, int32_t *raw);

/**
 * n consecutive conversions of the current input.
 * With DRDY pin returns ADS1262_ERROR_OVERRUN when a conversion was skipped.
 * Without the pin each sample is polled by RDATA1; skipped conversions are not detected there.
 * count (optional) gets the number of valid samples in raw, also on error.
 */
int ads1262_read_stream(ads1262_t *dev, int32_t *raw, size_t n, size_t *count);

/**
 * One conversion of each input pair inputs[i] = { pos, neg } into raw[i].
 * The last pair stays selected as the current input.
 */
int ads1262_scan(ads1262_t *dev, uint8_t inputs[][2], size_t n, int32_t *raw);  /* Not const: C < C23 */

/** Convert raw code to volts using current v_ref and gain */
double ads1262_to_volts(const ads1262_t *dev, int32_t raw);

/** Temperature sensor reading (TEMP/TEMP, gain 1, no chop, internal reference on) to degrees C */
double ads1262_to_celsius(const ads1262_t *dev, int32_t raw);

/** Data rate in samples per second (0 for invalid rate) */
float ads1262_sps(ads1262_rate_t rate);

/** Low-level register access; bypasses dev->cfg, use the setters for settings kept there */
int ads1262_read_register(ads1262_t *dev, uint8_t reg, uint8_t *value);
int ads1262_write_register(ads1262_t *dev, uint8_t reg, uint8_t value);

/** Text description of a return code */
const char *ads1262_strerror(int error_code);

#endif /* ADS1262_LIB_H */
