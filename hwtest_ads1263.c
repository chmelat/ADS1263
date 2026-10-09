/*
 *  Hardware self-check: needs a connected ADS1263 with DRDY wired. Inputs may float,
 *  except with -v. Covers what the emulator in test_ads1263.c can't:
 *   1. A restart clears the holding register and its checksum reads 00h too (datasheet
 *      9.4.7.1 says only "cleared"); reads without DRDY rely on it
 *   2. Reads with and without the DRDY pin, ADC1 and ADC2
 *   3. Supply monitors and temperature (PGA on) are plausible and don't depend on the channel
 *      read before, ADC1 and ADC2 agree on temperature (with the PGA bypassed they don't)
 *   4. PGA gains 1-32 (ADC1) and 1-128 (ADC2) on the internal test DAC (2.5 V common
 *      mode, nothing driven to the pins): all gains read the same voltage, no PGA alarm
 *   5. Overrange saturates at the full-scale code
 *   6. With -v: a known voltage between AIN0 (+) and AIN1 (-, grounded) in both polarities,
 *      all filters, high data rates, stream, chop and ADC2, PGA bypassed
 *
 *  Usage: ./hwtest_ads1263 [-v volts] [gpiochip drdy_line]   (same wiring as ads1263_example)
 *  Build & run: make hwtest (includes ads1263_lib.c itself, don't link it again)
 */

#define _DEFAULT_SOURCE

#include "ads1263_lib.c"                               /* Reuses transfer() and send_command() */

#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#define READS 8
#define TDAC_MID     0x00                              /* 2.5 V */
#define TDAC_P31M    0x03                              /* 2.53125 V */
#define TDAC_N31M    0x13                              /* 2.46875 V */
#define TDAC_P7M8    0x01                              /* 2.5078125 V */
#define TDAC_N7M8    0x11                              /* 2.4921875 V */
#define TDAC_3V      0x07
#define TDAC_2V      0x17

static ads1263_config_t base = {
    .spi_device = "/dev/spidev4.1",
    .spi_speed_hz = 4000000,
    .drdy_chip = "/dev/gpiochip1",
    .drdy_line = 3,                                    /* GPIO1_A3 */
    .v_ref = 2.5,
    .refmux = ADS1263_REF_INTERNAL,
    .drate = ADS1263_DRATE_100,
    .filter = ADS1263_FILTER_SINC3,
    .gain = ADS1263_GAIN_1,
    .pos = ADS1263_AIN0, .neg = ADS1263_AIN1,
    .chop = false,
    .bypass = true,
    .timeout_ms = 1000,
};
static ads1263_t adc;
static int failures;

static void check(bool ok, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    printf("%s  ", ok ? "PASS" : "FAIL");
    vprintf(fmt, ap);
    putchar('\n');
    va_end(ap);
    failures += !ok;
}

static bool open_adc(bool pin)
{
    ads1263_config_t cfg = base;
    if (!pin) {
        cfg.drdy_chip = NULL;
    }
    int result = ads1263_open(&adc, &cfg);
    check(result == ADS1263_OK, "open (%s): %s", pin ? "DRDY pin" : "polled", ads1263_strerror(result));
    return result == ADS1263_OK;
}

static int adc2_start(ads1263_gain_t gain, uint8_t pos, uint8_t neg)
{
    const ads1263_adc2_config_t cfg2 = { .v_ref = base.v_ref, .refmux = ADS1263_REF_INTERNAL,
                                         .drate = ADS1263_ADC2_DRATE_100, .gain = gain, .pos = pos, .neg = neg };
    return ads1263_adc2_start(&adc, &cfg2);
}

/** Mean of READS fresh conversions of ADC 1 or 2 from pos/neg in volts, NAN on error */
static double mean(int adc_n, uint8_t pos, uint8_t neg)
{
    double sum = 0;
    int32_t raw;
    int result = adc_n == 1 ? ads1263_set_input(&adc, pos, neg) : ads1263_adc2_set_input(&adc, pos, neg);
    for (int i = 0; i < READS && result == ADS1263_OK; i++) {
        result = adc_n == 1 ? ads1263_read(&adc, &raw) : ads1263_adc2_read(&adc, &raw);
        sum += adc_n == 1 ? ads1263_to_volts(&adc, raw) : ads1263_adc2_to_volts(&adc, raw);
    }
    if (result != ADS1263_OK) {
        printf("      read: %s\n", ads1263_strerror(result));
        return NAN;
    }
    return sum / READS;
}

static void set_tdac(uint8_t p, uint8_t n)
{
    /* OUTP/OUTN (bit 7) stay 0: the test DAC is not driven to AIN6/AIN7 */
    ads1263_write_register(&adc, ADS1263_REG_TDACP, p);
    ads1263_write_register(&adc, ADS1263_REG_TDACN, n);
}

/* 1. Raw RDATA right after a restart, before the first conversion is done */
static void test_cleared(void)
{
    for (int n = 1; n <= 2; n++) {
        const uint8_t tx[7] = { n == 1 ? ADS1263_CMD_RDATA1 : ADS1263_CMD_RDATA2 };
        uint8_t d[7];
        send_command(&adc, n == 1 ? ADS1263_CMD_STOP1 : ADS1263_CMD_STOP2);
        send_command(&adc, n == 1 ? ADS1263_CMD_START1 : ADS1263_CMD_START2);
        transfer(&adc, tx, d, sizeof(d));
        uint8_t flag = d[1] & (n == 1 ? ADS1263_STATUS_ADC1 : ADS1263_STATUS_ADC2);
        check(!flag && !(d[2] | d[3] | d[4] | d[5] | d[6]),
              "ADC%d after restart: no new-data flag, data and checksum 00h (status %02X, %02X %02X %02X %02X, sum %02X)",
              n, d[1], d[2], d[3], d[4], d[5], d[6]);
    }
}

/* 2. Reads with and without the pin */
static void test_reads(bool pin)
{
    int32_t raw, stream[20];
    size_t count = 0;
    int result = ads1263_read(&adc, &raw);
    check(result == ADS1263_OK, "ADC1 read (%s): %s", pin ? "DRDY pin" : "polled", ads1263_strerror(result));
    result = ads1263_read_stream(&adc, stream, 20, &count);
    check(result == ADS1263_OK || (result == ADS1263_ERROR_OVERRUN && count > 0),
          "ADC1 read_stream (%s), 20 samples at 100 SPS: %s, %zu samples", pin ? "DRDY pin" : "polled",
          ads1263_strerror(result), count);
    result = adc2_start(ADS1263_GAIN_1, ADS1263_AIN0, ADS1263_AIN1);
    if (result == ADS1263_OK) {
        result = ads1263_adc2_read(&adc, &raw);
    }
    check(result == ADS1263_OK, "ADC2 start and read: %s", ads1263_strerror(result));
    count = 0;
    result = ads1263_adc2_read_stream(&adc, stream, 10, &count);
    check(result == ADS1263_OK, "ADC2 read_stream, 10 samples: %s, %zu samples", ads1263_strerror(result), count);
}

/* 3. Internal sensors (PGA on, gain 1, chop off), at 1200 SPS where a channel read before shows most */
static void test_monitors(void)
{
    double t1, avdd, dvdd, t2;
    int32_t raw[4];
    uint8_t inputs[4][2] = {
        { ADS1263_TEMP, ADS1263_TEMP }, { ADS1263_AVDD_MON, ADS1263_AVDD_MON }, { ADS1263_DVDD_MON, ADS1263_DVDD_MON },
        { ADS1263_TEMP, ADS1263_TEMP },
    };
    ads1263_set_bypass(&adc, false);
    ads1263_set_drate(&adc, ADS1263_DRATE_1200);
    int result = ads1263_scan(&adc, inputs, 4, raw);
    ads1263_set_drate(&adc, ADS1263_DRATE_100);
    if (result != ADS1263_OK) {
        check(false, "scan of internal sensors: %s", ads1263_strerror(result));
        ads1263_set_bypass(&adc, true);
        return;
    }
    t1 = ads1263_to_celsius(&adc, raw[0]);
    avdd = 4 * ads1263_to_volts(&adc, raw[1]);
    dvdd = 4 * ads1263_to_volts(&adc, raw[2]);
    check(avdd > 4.75 && avdd < 5.25, "AVDD %.4f V (4.75-5.25 V)", avdd);
    check(dvdd > 3.0 && dvdd < 3.6, "DVDD %.4f V (3.0-3.6 V)", dvdd);
    check(t1 > 0 && t1 < 70, "chip temperature %.2f C (0-70 C)", t1);
    check(fabs(ads1263_to_celsius(&adc, raw[3]) - t1) < 0.2, "temperature right after DVDD %.2f C (0.2 C)",
          ads1263_to_celsius(&adc, raw[3]));

    if ((result = adc2_start(ADS1263_GAIN_1, ADS1263_TEMP, ADS1263_TEMP)) == ADS1263_OK) {
        result = ads1263_adc2_read(&adc, &raw[0]);
    }
    t2 = ads1263_adc2_to_celsius(&adc, raw[0]);
    check(result == ADS1263_OK && fabs(t2 - t1) < 1, "ADC2 temperature %.2f C, ADC1 %.2f C (1 C)", t2, t1);
    ads1263_set_bypass(&adc, true);                    /* ADC1 still on TEMP: from now on it loads the sensor */
}

/* 4. PGA gains on the test DAC; 5. overrange */
static void test_gains(void)
{
    ads1263_set_bypass(&adc, false);
    set_tdac(TDAC_P31M, TDAC_N31M);
    double ref = NAN;
    for (int gain = 1; gain <= 32; gain *= 2) {
        ads1263_set_gain(&adc, (ads1263_gain_t)gain);
        double v = mean(1, ADS1263_TDAC, ADS1263_TDAC);
        if (gain == 1) {
            ref = v;
            check(fabs(v - 0.0625) < 0.0625 * 0.05, "ADC1 test DAC 62.5 mV at gain 1: %.6f V (5 %%), status %02X", v,
                  adc.status);
        } else {
            check(fabs(v / ref - 1) < 0.002 && !(adc.status & 0x0E),
                  "ADC1 gain %2d: %.6f V, %+.3f %% from gain 1 (0.2 %%), no PGA alarm (status %02X)", gain, v,
                  100 * (v / ref - 1), adc.status);
        }
    }

    set_tdac(TDAC_3V, TDAC_2V);                        /* 1 V at gain 4: full scale 0.625 V */
    ads1263_set_gain(&adc, ADS1263_GAIN_4);
    int32_t raw = 0, neg = 0;
    ads1263_set_input(&adc, ADS1263_TDAC, ADS1263_TDAC);
    ads1263_read(&adc, &raw);
    set_tdac(TDAC_2V, TDAC_3V);
    ads1263_read(&adc, &neg);
    check(raw == INT32_MAX && neg == INT32_MIN, "ADC1 +-1 V at gain 4 saturates: %08X / %08X (status %02X)",
          (unsigned)raw, (unsigned)neg, adc.status);

    set_tdac(TDAC_P7M8, TDAC_N7M8);                    /* 15.6 mV fits gain 128 (19.5 mV) */
    for (int gain = 1; gain <= 128; gain *= 2) {
        adc2_start((ads1263_gain_t)gain, ADS1263_TDAC, ADS1263_TDAC);
        double v = mean(2, ADS1263_TDAC, ADS1263_TDAC);
        if (gain == 1) {
            ref = v;
            check(fabs(v - 0.015625) < 0.015625 * 0.05, "ADC2 test DAC 15.6 mV at gain 1: %.6f V (5 %%)", v);
        } else {
            check(fabs(v / ref - 1) < 0.005, "ADC2 gain %3d: %.6f V, %+.3f %% from gain 1 (0.5 %%)", gain, v,
                  100 * (v / ref - 1));
        }
    }
    set_tdac(TDAC_3V, TDAC_2V);
    adc2_start(ADS1263_GAIN_4, ADS1263_TDAC, ADS1263_TDAC);
    ads1263_adc2_read(&adc, &raw);
    check(raw == 0x7FFFFF, "ADC2 1 V at gain 4 saturates: %06X", (unsigned)raw & 0xFFFFFF);

    set_tdac(TDAC_MID, TDAC_MID);
    ads1263_set_gain(&adc, ADS1263_GAIN_1);
    ads1263_set_bypass(&adc, true);
}

/* 6. Known voltage between AIN0 and grounded AIN1, PGA bypassed */
static void test_known(double known)
{
    /* Uncalibrated error at most about 0.25 % (README, Accuracy) plus the meter */
    const double tol = 0.003 * known;
    double v = mean(1, ADS1263_AIN0, ADS1263_AIN1);
    double r = mean(1, ADS1263_AIN1, ADS1263_AIN0);
    check(fabs(v - known) < tol, "ADC1 AIN0-AIN1 %.6f V, %+.2f mV from %.4f V (0.3 %%)", v, (v - known) * 1e3, known);
    /* Bypassed inputs draw about 150 nA: through the source resistance it changes sign with the polarity */
    check(fabs(v + r) < 200e-6, "ADC1 AIN1-AIN0 %.6f V: polarity difference %+.1f uV (200 uV, input current x source R)",
          r, (v + r) * 1e6);

    /* The source may drift: settings are compared with a fresh sinc3 reading each time */
    static const char *const names[] = { "sinc1", "sinc2", "sinc4", "FIR 20 SPS", "1200 SPS", "7200 SPS", "chop" };
    for (int i = 0; i < 7; i++) {
        double before = mean(1, ADS1263_AIN0, ADS1263_AIN1);
        if (i < 3) {
            ads1263_set_filter(&adc, (ads1263_filter_t)(i < 2 ? i : ADS1263_FILTER_SINC4));
        } else if (i == 3) {
            ads1263_set_drate(&adc, ADS1263_DRATE_20);
            ads1263_set_filter(&adc, ADS1263_FILTER_FIR);
        } else if (i < 6) {
            ads1263_set_drate(&adc, i == 4 ? ADS1263_DRATE_1200 : ADS1263_DRATE_7200);
        } else {
            ads1263_set_chop(&adc, true);
        }
        double s = mean(1, ADS1263_AIN0, ADS1263_AIN1);
        check(fabs(s - before) < 100e-6, "ADC1 %-10s %.6f V, %+.1f uV from sinc3 100 SPS (100 uV)", names[i], s,
              (s - before) * 1e6);
        ads1263_set_chop(&adc, false);
        ads1263_set_drate(&adc, ADS1263_DRATE_100);
        ads1263_set_filter(&adc, ADS1263_FILTER_SINC3);
    }

    int32_t stream[100];
    size_t count = 0;
    double sum = 0;
    ads1263_set_input(&adc, ADS1263_AIN0, ADS1263_AIN1);
    int result = ads1263_read_stream(&adc, stream, 100, &count);
    for (size_t i = 0; i < count; i++) {
        sum += ads1263_to_volts(&adc, stream[i]);
    }
    v = mean(1, ADS1263_AIN0, ADS1263_AIN1);
    check(count > 0 && fabs(sum / count - v) < 100e-6, "ADC1 stream %zu samples (%s): %.6f V (100 uV)", count,
          ads1263_strerror(result), count ? sum / count : 0);

    adc2_start(ADS1263_GAIN_1, ADS1263_AIN0, ADS1263_AIN1);
    double v2 = mean(2, ADS1263_AIN0, ADS1263_AIN1), r2 = mean(2, ADS1263_AIN1, ADS1263_AIN0);
    check(fabs(v2 - known) < tol && fabs(v2 / v - 1) < 0.001,
          "ADC2 AIN0-AIN1 %.6f V, %+.2f mV from %.4f V (0.3 %%), %+.3f %% from ADC1 (0.1 %%)", v2,
          (v2 - known) * 1e3, known, 100 * (v2 / v - 1));
    check(fabs(v2 + r2) < 200e-6, "ADC2 AIN1-AIN0 %.6f V: polarity difference %+.1f uV (200 uV)", r2, (v2 + r2) * 1e6);
}

int main(int argc, char *argv[])
{
    double known = 0;
    int arg = 1;
    if (argc > 2 && strcmp(argv[1], "-v") == 0) {
        char *end;
        known = strtod(argv[2], &end);
        if (*end || known <= 0 || known > 2.4) {
            known = -1;
        }
        arg = 3;
    }
    char *end = NULL;
    unsigned long line = argc - arg == 2 ? strtoul(argv[arg + 1], &end, 10) : 0;
    if (known < 0 || (argc - arg != 0 && argc - arg != 2) ||
        (argc - arg == 2 && (!isdigit((unsigned char)argv[arg + 1][0]) || *end || line > UINT_MAX))) {
        fprintf(stderr, "Usage: %s [-v volts] [gpiochip drdy_line]\n"
                        "  -v  known voltage 0-2.4 V between AIN0 (+) and AIN1 (-), AIN1 grounded\n", argv[0]);
        return EXIT_FAILURE;
    }
    if (argc - arg == 2) {
        base.drdy_chip = argv[arg];
        base.drdy_line = (unsigned int)line;
    }
    double v_ref;
    if (ads1263_load_vref(NULL, &v_ref) == ADS1263_OK) {
        base.v_ref = v_ref;
    }

    if (open_adc(false)) {
        test_cleared();
        test_reads(false);
        ads1263_close(&adc);
    }
    if (!open_adc(true)) {
        return EXIT_FAILURE;
    }
    test_reads(true);
    test_monitors();
    test_gains();
    if (known > 0) {
        test_known(known);
    }
    ads1263_close(&adc);

    printf("\n%s: %d failure%s\n", failures ? "FAILED" : "All checks passed", failures, failures == 1 ? "" : "s");
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
