/**
 * @file ads1263_lib.h
 * @brief Library for ADS1263 32-bit ADC + 24-bit auxiliary ADC2 on Linux spidev (Orange Pi, Raspberry Pi)
 * @version 2.1
 * @date 2026-10-08
 *
 * - Device handle (ads1263_t), any number of devices, no global state
 * - Optional DRDY pin on GPIO (kernel GPIO uAPI); without it the ADC1 bit of the
 *   status byte that comes with every RDATA1 read is polled
 * - Any input combination AIN0-AIN9 / AINCOM, internal temperature sensor and supply monitors
 * - Digital filter, chop mode, reference selection, IDAC excitation currents
 * - Raw 32-bit codes + ads1263_to_volts(), every read checked by the checksum byte
 * - Gain, data rate, filter and reference setters run offset self-calibration
 * - ADC2 (24 bits, 10-800 SPS, gain 1-128) with its own inputs and reference, ads1263_adc2_*()
 * - No library dependencies: Linux spidev + GPIO character device (kernel >= 5.10)
 * - ADS1263 only: the ADS1262 (no ADC2) is rejected at open
 *
 * Wiring (Orange Pi 5 / Raspberry Pi header):
 *  ADS1263     Header
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
 *  DRDY        any GPIO, optional (see ads1263_config_t.drdy_chip)
 */

#ifndef ADS1263_LIB_H
#define ADS1263_LIB_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Return codes */
#define ADS1263_OK                   0  /**< Operation successful */
#define ADS1263_ERROR_PARAMETER     -1  /**< Invalid parameter */
#define ADS1263_ERROR_COMMUNICATION -2  /**< SPI or GPIO error, or no ADS1263 found */
#define ADS1263_ERROR_TIMEOUT       -3  /**< No new data in time */
#define ADS1263_ERROR_OVERRUN       -4  /**< read_stream() couldn't keep up, conversions skipped */
#define ADS1263_ERROR_CHECKSUM      -5  /**< Conversion data corrupted on the bus */

/* Registers */
#define ADS1263_REG_ID           0x00   /**< Device identification */
#define ADS1263_REG_POWER        0x01   /**< Reset flag, level shift, internal reference */
#define ADS1263_REG_INTERFACE    0x02   /**< Status and checksum bytes, SPI time-out */
#define ADS1263_REG_MODE0        0x03   /**< Reference reversal, run mode, chop, delay */
#define ADS1263_REG_MODE1        0x04   /**< Digital filter, sensor bias */
#define ADS1263_REG_MODE2        0x05   /**< PGA bypass, gain, data rate */
#define ADS1263_REG_INPMUX       0x06   /**< Input multiplexer */
#define ADS1263_REG_OFCAL0       0x07   /**< Offset calibration, 24 bits LSB first */
#define ADS1263_REG_OFCAL1       0x08
#define ADS1263_REG_OFCAL2       0x09
#define ADS1263_REG_FSCAL0       0x0A   /**< Full-scale calibration, 24 bits LSB first */
#define ADS1263_REG_FSCAL1       0x0B
#define ADS1263_REG_FSCAL2       0x0C
#define ADS1263_REG_IDACMUX      0x0D   /**< IDAC pins */
#define ADS1263_REG_IDACMAG      0x0E   /**< IDAC currents */
#define ADS1263_REG_REFMUX       0x0F   /**< Reference multiplexer */
#define ADS1263_REG_TDACP        0x10   /**< Test DAC positive */
#define ADS1263_REG_TDACN        0x11   /**< Test DAC negative */
#define ADS1263_REG_GPIOCON      0x12   /**< GPIO connection */
#define ADS1263_REG_GPIODIR      0x13   /**< GPIO direction */
#define ADS1263_REG_GPIODAT      0x14   /**< GPIO data */
#define ADS1263_REG_ADC2CFG      0x15   /**< ADC2 data rate, reference, gain */
#define ADS1263_REG_ADC2MUX      0x16   /**< ADC2 input multiplexer */
#define ADS1263_REG_ADC2OFC0     0x17   /**< ADC2 offset calibration, 16 bits LSB first */
#define ADS1263_REG_ADC2OFC1     0x18
#define ADS1263_REG_ADC2FSC0     0x19   /**< ADC2 full-scale calibration, 16 bits LSB first */
#define ADS1263_REG_ADC2FSC1     0x1A

/* Status byte (ads1263_t.status, status2), datasheet table 9-18; alarms are valid for ADC1 data only */
#define ADS1263_STATUS_ADC2      0x80   /**< New ADC2 data since the last read */
#define ADS1263_STATUS_ADC1      0x40   /**< New ADC1 data since the last read */
#define ADS1263_STATUS_EXTCLK    0x20   /**< External clock */
#define ADS1263_STATUS_REF_ALM   0x10   /**< Reference below 0.4 V */
#define ADS1263_STATUS_PGAL_ALM  0x08   /**< PGA output below AVSS + 0.2 V */
#define ADS1263_STATUS_PGAH_ALM  0x04   /**< PGA output above AVDD - 0.2 V */
#define ADS1263_STATUS_PGAD_ALM  0x02   /**< PGA differential output beyond +-105 % FS */
#define ADS1263_STATUS_RESET     0x01   /**< Device was reset since ads1263_open() */

/* Commands */
#define ADS1263_CMD_NOP          0x00   /**< No operation */
#define ADS1263_CMD_RESET        0x06   /**< Reset */
#define ADS1263_CMD_START1       0x08   /**< Start or restart ADC1 conversions */
#define ADS1263_CMD_STOP1        0x0A   /**< Stop ADC1 conversions */
#define ADS1263_CMD_RDATA1       0x12   /**< Read ADC1 data */
#define ADS1263_CMD_SYOCAL1      0x16   /**< System offset calibration (inputs shorted externally) */
#define ADS1263_CMD_SYGCAL1      0x17   /**< System gain calibration (full scale applied) */
#define ADS1263_CMD_SFOCAL1      0x19   /**< Self offset calibration */
#define ADS1263_CMD_START2       0x0C   /**< Start or restart ADC2 conversions */
#define ADS1263_CMD_STOP2        0x0E   /**< Stop ADC2 conversions */
#define ADS1263_CMD_RDATA2       0x14   /**< Read ADC2 data */
#define ADS1263_CMD_SYOCAL2      0x1B   /**< ADC2 system offset calibration */
#define ADS1263_CMD_SYGCAL2      0x1C   /**< ADC2 system gain calibration */
#define ADS1263_CMD_SFOCAL2      0x1E   /**< ADC2 self offset calibration */
#define ADS1263_CMD_RREG         0x20   /**< Read registers */
#define ADS1263_CMD_WREG         0x40   /**< Write registers */

/* Inputs for ads1263_set_input() and ads1263_adc2_set_input(); internal ones are selected as pos == neg, e.g. TEMP/TEMP */
enum {
    ADS1263_AIN0 = 0, ADS1263_AIN1, ADS1263_AIN2, ADS1263_AIN3, ADS1263_AIN4,
    ADS1263_AIN5, ADS1263_AIN6, ADS1263_AIN7, ADS1263_AIN8, ADS1263_AIN9,
    ADS1263_AINCOM,           /**< Common input for single-ended measurement */
    ADS1263_TEMP,             /**< Temperature sensor, see ads1263_to_celsius() */
    ADS1263_AVDD_MON,         /**< (AVDD - AVSS) / 4 */
    ADS1263_DVDD_MON,         /**< DVDD / 4 */
    ADS1263_TDAC,             /**< Test DAC */
    ADS1263_FLOAT             /**< Open connection */
};

/* IDAC pin for ads1263_set_idac(): ADS1263_AIN0..ADS1263_AINCOM, or not connected */
#define ADS1263_IDAC_NC 11

/* Data rates (DR register value) */
typedef enum {
    ADS1263_DRATE_2_5 = 0, ADS1263_DRATE_5, ADS1263_DRATE_10, ADS1263_DRATE_16_6,
    ADS1263_DRATE_20, ADS1263_DRATE_50, ADS1263_DRATE_60, ADS1263_DRATE_100,
    ADS1263_DRATE_400, ADS1263_DRATE_1200, ADS1263_DRATE_2400, ADS1263_DRATE_4800,
    ADS1263_DRATE_7200, ADS1263_DRATE_14400, ADS1263_DRATE_19200, ADS1263_DRATE_38400
} ads1263_drate_t;

/* Digital filter; FIR only at 2.5, 5, 10 and 20 SPS. 14400 SPS and up always use sinc5. */
typedef enum {
    ADS1263_FILTER_SINC1 = 0, ADS1263_FILTER_SINC2, ADS1263_FILTER_SINC3,
    ADS1263_FILTER_SINC4, ADS1263_FILTER_FIR
} ads1263_filter_t;

/* ADC2 data rates (DR2 bits); ADC2 always uses a sinc3 filter */
typedef enum {
    ADS1263_ADC2_DRATE_10 = 0, ADS1263_ADC2_DRATE_100, ADS1263_ADC2_DRATE_400, ADS1263_ADC2_DRATE_800
} ads1263_adc2_drate_t;

/* Gain; 64 and 128 for ADC2 only */
typedef enum {
    ADS1263_GAIN_1 = 1, ADS1263_GAIN_2 = 2, ADS1263_GAIN_4 = 4,
    ADS1263_GAIN_8 = 8, ADS1263_GAIN_16 = 16, ADS1263_GAIN_32 = 32,
    ADS1263_GAIN_64 = 64, ADS1263_GAIN_128 = 128
} ads1263_gain_t;

/* Reference (REFMUX register value; ADC2 takes the same constants); set v_ref to match */
#define ADS1263_REF_INTERNAL   0x00   /**< Internal 2.5 V */
#define ADS1263_REF_AIN0_AIN1  0x09   /**< External, REFP = AIN0, REFN = AIN1 */
#define ADS1263_REF_AIN2_AIN3  0x12   /**< External, REFP = AIN2, REFN = AIN3 */
#define ADS1263_REF_AIN4_AIN5  0x1B   /**< External, REFP = AIN4, REFN = AIN5 */
#define ADS1263_REF_AVDD       0x24   /**< Analog supply AVDD - AVSS */

/* IDAC current (IDACMAG register value) */
typedef enum {
    ADS1263_IDAC_OFF = 0, ADS1263_IDAC_50UA, ADS1263_IDAC_100UA, ADS1263_IDAC_250UA,
    ADS1263_IDAC_500UA, ADS1263_IDAC_750UA, ADS1263_IDAC_1000UA, ADS1263_IDAC_1500UA,
    ADS1263_IDAC_2000UA, ADS1263_IDAC_2500UA, ADS1263_IDAC_3000UA
} ads1263_idac_t;

/* Configuration for ads1263_open() */
typedef struct {
    const char *spi_device;    /**< SPI device, e.g. "/dev/spidev0.0" */
    uint32_t spi_speed_hz;     /**< SCLK, max 8000000 */
    const char *drdy_chip;     /**< GPIO chip with DRDY, e.g. "/dev/gpiochip1"; NULL = poll status byte */
    unsigned int drdy_line;    /**< GPIO line offset of DRDY on drdy_chip */
    double v_ref;              /**< Reference voltage [V], 0.9-5.25 (2.5 for internal) */
    uint8_t refmux;            /**< ADS1263_REF_* */
    ads1263_drate_t drate;       /**< Data rate */
    ads1263_filter_t filter;   /**< Digital filter */
    ads1263_gain_t gain;       /**< PGA gain */
    uint8_t pos, neg;          /**< Inputs: ADS1263_AIN0..ADS1263_FLOAT */
    bool chop;                 /**< Input chop: lower offset and drift, half the data rate */
    bool bypass;               /**< PGA bypassed: inputs AVSS - 0.1 .. AVDD + 0.1 V (e.g. single-ended
                                    against a grounded AINCOM), gain 1 only, 40 MOhm; PGA alarms unused */
    uint32_t timeout_ms;       /**< Timeout on top of the expected conversion time, 1..3600000 (ADC2 too) */
} ads1263_config_t;

/* ADC2 configuration for ads1263_adc2_start() */
typedef struct {
    double v_ref;              /**< Reference voltage [V], 0.9-5.25 (2.5 for internal) */
    uint8_t refmux;            /**< ADS1263_REF_* */
    ads1263_adc2_drate_t drate;  /**< Data rate */
    ads1263_gain_t gain;       /**< Gain 1-128 */
    uint8_t pos, neg;          /**< Inputs: ADS1263_AIN0..ADS1263_FLOAT */
} ads1263_adc2_config_t;

/* Device handle, owned by the caller; cfg always holds the current settings */
typedef struct {
    int spi_fd;
    int drdy_fd;               /**< -1 when DRDY pin is not used */
    ads1263_config_t cfg;
    uint8_t status;            /**< Status byte of the last ADC1 conversion read (ADS1263_STATUS_*) */
    ads1263_adc2_config_t cfg2;  /**< Current ADC2 settings; register defaults until ads1263_adc2_start() */
    uint8_t status2;           /**< Status byte of the last ADC2 conversion read (alarm bits not valid) */
} ads1263_t;

/**
 * Open SPI (and DRDY GPIO), reset the ADC, check it's there, apply configuration,
 * start conversions and run offset self-calibration (not with chop).
 * Call ads1263_close() before opening the same handle again, otherwise its file
 * descriptors leak (and the GPIO line stays busy).
 * @return ADS1263_OK or negative error; on error nothing stays open
 */
int ads1263_open(ads1263_t *dev, const ads1263_config_t *cfg);

/** Close SPI and GPIO file descriptors */
void ads1263_close(ads1263_t *dev);

/** Select inputs, e.g. AIN0/AIN1 differential, AIN3/AINCOM single-ended, TEMP/TEMP internal */
int ads1263_set_input(ads1263_t *dev, uint8_t pos, uint8_t neg);

/** Set PGA gain and self-calibrate; with the PGA bypassed only gain 1 */
int ads1263_set_gain(ads1263_t *dev, ads1263_gain_t gain);

/** Set data rate and self-calibrate (FIR filter allows only 2.5, 5, 10 and 20 SPS) */
int ads1263_set_drate(ads1263_t *dev, ads1263_drate_t drate);

/** Set digital filter and self-calibrate */
int ads1263_set_filter(ads1263_t *dev, ads1263_filter_t filter);

/** Select reference (ADS1263_REF_*) with its voltage v_ref and self-calibrate */
int ads1263_set_reference(ads1263_t *dev, uint8_t refmux, double v_ref);

/** Input chop on/off; turning it off self-calibrates (offset calibration is unused with chop) */
int ads1263_set_chop(ads1263_t *dev, bool on);

/**
 * PGA bypass on/off and self-calibrate. With the PGA on, both inputs must stay
 * AVSS + 0.3 V .. AVDD - 0.3 V (datasheet equation 12), so a grounded input reads wrong;
 * bypassed the range is AVSS - 0.1 .. AVDD + 0.1 V, gain 1 only (set it first).
 */
int ads1263_set_bypass(ads1263_t *dev, bool on);

/** Connect IDAC 1 or 2 to pin (ADS1263_AIN0..ADS1263_AINCOM or ADS1263_IDAC_NC) with current */
int ads1263_set_idac(ads1263_t *dev, int idac, uint8_t pin, ads1263_idac_t current);

/**
 * Run calibration command ADS1263_CMD_SFOCAL1, SYOCAL1 or SYGCAL1 and wait for it.
 * Takes up to 9.2 s at 2.5 SPS (datasheet table 9-28). SFOCAL1 opens the inputs
 * meanwhile; for SYOCAL1 short the selected inputs, for SYGCAL1 apply full scale.
 */
int ads1263_calibrate(ads1263_t *dev, uint8_t cmd);

/** One fresh conversion with current settings (restart, wait, RDATA1), raw signed 32-bit code */
int ads1263_read(ads1263_t *dev, int32_t *raw);

/**
 * n consecutive conversions of the current input.
 * With DRDY pin returns ADS1263_ERROR_OVERRUN when a conversion was skipped.
 * Without the pin each sample is polled by RDATA1; skipped conversions are not detected there.
 * count (optional) gets the number of valid samples in raw, also on error.
 */
int ads1263_read_stream(ads1263_t *dev, int32_t *raw, size_t n, size_t *count);

/**
 * One conversion of each input pair inputs[i] = { pos, neg } into raw[i].
 * The last pair stays selected as the current input.
 */
int ads1263_scan(ads1263_t *dev, uint8_t inputs[][2], size_t n, int32_t *raw);  /* Not const: C < C23 */

/** Convert raw code to volts using current v_ref and gain */
double ads1263_to_volts(const ads1263_t *dev, int32_t raw);

/** Temperature sensor reading (TEMP/TEMP, gain 1, no chop, internal reference on) to degrees C */
double ads1263_to_celsius(const ads1263_t *dev, int32_t raw);

/** Data rate in samples per second (0 for invalid rate) */
float ads1263_sps(ads1263_drate_t drate);

/**
 * Measured value of the internal reference (REFOUT pin) for v_ref, from a file with one number
 * (decimal point whatever the locale; # comment lines allowed), 2.49-2.51 V.
 * path NULL: $XDG_CONFIG_HOME/ads1263/vref, or ~/.config/ads1263/vref.
 * Call before ads1263_open() / ads1263_adc2_start() and put it into their v_ref.
 * @return ADS1263_OK, or ADS1263_ERROR_PARAMETER with errno ENOENT (no file, or no HOME),
 *         EINVAL (damaged file or out of range) or another from fopen()
 */
int ads1263_load_vref(const char *path, double *v_ref);

/** Low-level register access; bypasses dev->cfg, use the setters for settings kept there */
int ads1263_read_register(ads1263_t *dev, uint8_t reg, uint8_t *value);
int ads1263_write_register(ads1263_t *dev, uint8_t reg, uint8_t value);

/* ===== ADC2 =====
 * Runs independently of ADC1. It has no DRDY pin: new data are polled by RDATA2 and the
 * end of calibration is waited by the datasheet time (table 9-29, 28 ms - 1.7 s).
 * Codes are signed 24-bit (-2^23 .. 2^23 - 1).
 */

/**
 * Configure ADC2, start its conversions and run offset self-calibration.
 * Until called, ADC2 keeps its reset settings (10 SPS, gain 1, internal reference, AIN0/AIN1).
 */
int ads1263_adc2_start(ads1263_t *dev, const ads1263_adc2_config_t *cfg);

/** Select ADC2 inputs, same choices as ads1263_set_input() */
int ads1263_adc2_set_input(ads1263_t *dev, uint8_t pos, uint8_t neg);

/** Set ADC2 gain 1-128 and self-calibrate */
int ads1263_adc2_set_gain(ads1263_t *dev, ads1263_gain_t gain);

/** Set ADC2 data rate and self-calibrate */
int ads1263_adc2_set_drate(ads1263_t *dev, ads1263_adc2_drate_t drate);

/** Select ADC2 reference (ADS1263_REF_*) with its voltage v_ref and self-calibrate */
int ads1263_adc2_set_reference(ads1263_t *dev, uint8_t refmux, double v_ref);

/**
 * Run ADS1263_CMD_SFOCAL2, SYOCAL2 or SYGCAL2 and wait for it (up to 1.9 s at 10 SPS).
 * SFOCAL2 opens the ADC2 inputs meanwhile; for SYOCAL2 short the inputs, for SYGCAL2 apply full scale.
 */
int ads1263_adc2_calibrate(ads1263_t *dev, uint8_t cmd);

/** One fresh ADC2 conversion (restart, wait, RDATA2), raw signed 24-bit code */
int ads1263_adc2_read(ads1263_t *dev, int32_t *raw);

/** n consecutive ADC2 conversions, polled; skipped conversions are not detected */
int ads1263_adc2_read_stream(ads1263_t *dev, int32_t *raw, size_t n, size_t *count);

/** One ADC2 conversion of each input pair, the last pair stays selected */
int ads1263_adc2_scan(ads1263_t *dev, uint8_t inputs[][2], size_t n, int32_t *raw);

/** Convert ADC2 raw code to volts using its v_ref and gain */
double ads1263_adc2_to_volts(const ads1263_t *dev, int32_t raw);

/** ADC2 temperature sensor reading (TEMP/TEMP, gain 1, internal reference on) to degrees C */
double ads1263_adc2_to_celsius(const ads1263_t *dev, int32_t raw);

/** ADC2 data rate in samples per second (0 for invalid rate) */
float ads1263_adc2_sps(ads1263_adc2_drate_t drate);

/** Text description of a return code */
const char *ads1263_strerror(int error_code);

#endif /* ADS1263_LIB_H */
