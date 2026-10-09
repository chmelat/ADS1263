/*
 *  ADS1263 Example - 32-bit, low-noise ADC with 10 inputs and 24-bit auxiliary ADC2
 *
 *  Usage: ./ads1263                     DRDY on GPIO1_A3 (/dev/gpiochip1, line 3)
 *         ./ads1263 /dev/gpiochip1 22   DRDY on GPIO chip 1, line 22 (find yours: sudo gpioinfo)
 *
 *  Wiring: see ads1263_lib.h
 */

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ads1263_lib.h"

#define STREAM_SAMPLES 50

int main(int argc, char *argv[])
{
    char *end = NULL;
    unsigned long line = argc == 3 ? strtoul(argv[2], &end, 10) : 3;  /* GPIO1_A3 */
    if ((argc != 1 && argc != 3) ||
        (argc == 3 && (!isdigit((unsigned char)argv[2][0]) || *end || line > UINT_MAX))) {
        fprintf(stderr, "Usage: %s [gpiochip drdy_line]\n", argv[0]);
        return EXIT_FAILURE;
    }

    ads1263_config_t cfg = {
        .spi_device = "/dev/spidev4.1",
        .spi_speed_hz = 4000000,           /* 4 MHz, max is 8 MHz */
        .drdy_chip = argc == 3 ? argv[1] : "/dev/gpiochip1",
        .drdy_line = (unsigned int)line,
        .v_ref = 2.5,
        .refmux = ADS1263_REF_INTERNAL,
        .drate = ADS1263_DRATE_400,
        .filter = ADS1263_FILTER_SINC3,
        .gain = ADS1263_GAIN_1,
        .pos = ADS1263_AIN0, .neg = ADS1263_AIN1,
        .chop = false,
        .bypass = true,                    /* Single-ended scan against a grounded AINCOM needs it */
        .timeout_ms = 1000,
    };
    ads1263_t adc;
    int32_t raw;

    /* Measured internal reference (README, Accuracy); without the file the nominal 2.5 V stays */
    double v_ref;
    if (ads1263_load_vref(NULL, &v_ref) == ADS1263_OK) {
        cfg.v_ref = v_ref;
    } else if (errno != ENOENT) {
        fprintf(stderr, "Reference value not loaded: %s\n", strerror(errno));
    }

    int result = ads1263_open(&adc, &cfg);
    if (result != ADS1263_OK) {
        fprintf(stderr, "Cannot open ADS1263: %s\n", ads1263_strerror(result));
        return EXIT_FAILURE;
    }
    printf("ADS1263 at %.0f SPS, DRDY %s, v_ref %.4f V\n", ads1263_sps(cfg.drate),
           cfg.drdy_chip ? "on GPIO" : "polled", cfg.v_ref);

    /* Register dump */
    printf("\nReg  Binary    Hex\n");
    for (uint8_t reg = ADS1263_REG_ID; reg <= ADS1263_REG_ADC2FSC1; reg++) {
        uint8_t value;
        if (ads1263_read_register(&adc, reg, &value) == ADS1263_OK) {
            printf("0x%02X ", reg);
            for (int bit = 7; bit >= 0; bit--) {
                putchar(value >> bit & 1 ? '1' : '0');
            }
            printf("  0x%02X\n", value);
        }
    }

    /* Single differential reading AIN0 - AIN1 */
    if ((result = ads1263_read(&adc, &raw)) != ADS1263_OK) {
        goto error;
    }
    printf("\nAIN0-AIN1: %.9f V\n", ads1263_to_volts(&adc, raw));

    /* All 10 inputs single-ended against AINCOM */
    uint8_t inputs[10][2];
    int32_t values[10];
    for (int i = 0; i < 10; i++) {
        inputs[i][0] = (uint8_t)(ADS1263_AIN0 + i);
        inputs[i][1] = ADS1263_AINCOM;
    }
    if ((result = ads1263_scan(&adc, inputs, 10, values)) != ADS1263_OK) {
        goto error;
    }
    printf("\n");
    for (int i = 0; i < 10; i++) {
        printf("AIN%d-COM: %.9f V\n", i, ads1263_to_volts(&adc, values[i]));
    }

    /* Internal temperature sensor and supply monitors: PGA on, gain 1, no chop (datasheet 9.3.4) */
    uint8_t internal[3][2] = {
        { ADS1263_TEMP, ADS1263_TEMP },
        { ADS1263_AVDD_MON, ADS1263_AVDD_MON },
        { ADS1263_DVDD_MON, ADS1263_DVDD_MON },
    };
    if ((result = ads1263_set_bypass(&adc, false)) != ADS1263_OK ||
        (result = ads1263_scan(&adc, internal, 3, values)) != ADS1263_OK ||
        (result = ads1263_set_bypass(&adc, true)) != ADS1263_OK) {
        goto error;
    }
    printf("\nChip temperature: %.2f C\n", ads1263_to_celsius(&adc, values[0]));
    printf("AVDD: %.4f V\n", 4 * ads1263_to_volts(&adc, values[1]));
    printf("DVDD: %.4f V\n", 4 * ads1263_to_volts(&adc, values[2]));

    /* Consecutive conversions of AIN0 - AIN1 */
    int32_t samples[STREAM_SAMPLES];
    size_t count = 0;
    if ((result = ads1263_set_input(&adc, ADS1263_AIN0, ADS1263_AIN1)) != ADS1263_OK) {
        goto error;
    }
    result = ads1263_read_stream(&adc, samples, STREAM_SAMPLES, &count);
    if (result == ADS1263_ERROR_OVERRUN && count > 0) {
        /* Linux isn't real-time: at high data rates a conversion can be missed, keep the valid ones */
        printf("\nStream overran after %zu samples (see README, streaming limits)\n", count);
    } else if (result != ADS1263_OK) {
        fprintf(stderr, "Stream stopped after %zu samples\n", count);
        goto error;
    }
    double sum = 0.0;
    for (size_t i = 0; i < count; i++) {
        sum += ads1263_to_volts(&adc, samples[i]);
    }
    printf("\nAIN0-AIN1 average of %zu samples: %.9f V\n", count, sum / (double)count);

    /* ADC2 cross-checks AIN0 - AIN1 and reads the chip temperature on its own */
    const ads1263_adc2_config_t cfg2 = {
        .v_ref = cfg.v_ref,                /* Same internal reference */
        .refmux = ADS1263_REF_INTERNAL,
        .drate = ADS1263_ADC2_DRATE_100,
        .gain = ADS1263_GAIN_1,
        .pos = ADS1263_AIN0, .neg = ADS1263_AIN1,
    };
    if ((result = ads1263_adc2_start(&adc, &cfg2)) != ADS1263_OK ||
        (result = ads1263_adc2_read(&adc, &raw)) != ADS1263_OK) {
        goto error;
    }
    printf("\nADC2 AIN0-AIN1: %.7f V\n", ads1263_adc2_to_volts(&adc, raw));
    if ((result = ads1263_adc2_set_input(&adc, ADS1263_TEMP, ADS1263_TEMP)) != ADS1263_OK ||
        (result = ads1263_adc2_read(&adc, &raw)) != ADS1263_OK) {
        goto error;
    }
    printf("ADC2 chip temperature: %.2f C\n", ads1263_adc2_to_celsius(&adc, raw));

    ads1263_close(&adc);
    return EXIT_SUCCESS;

error:
    fprintf(stderr, "ADS1263 error: %s\n", ads1263_strerror(result));
    ads1263_close(&adc);
    return EXIT_FAILURE;
}
