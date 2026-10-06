/*
 *  ADS1262 Example - 32-bit, low-noise ADC with 10 inputs
 *
 *  Usage: ./ads1262                     DRDY on GPIO1_A3 (/dev/gpiochip1, line 3)
 *         ./ads1262 /dev/gpiochip1 22   DRDY on GPIO chip 1, line 22 (find yours: sudo gpioinfo)
 *
 *  Wiring: see ads1262_lib.h
 */

#include <ctype.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include "ads1262_lib.h"

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

    ads1262_config_t cfg = {
        .spi_device = "/dev/spidev4.1",
        .spi_speed_hz = 4000000,           /* 4 MHz, max is 8 MHz */
        .drdy_chip = argc == 3 ? argv[1] : "/dev/gpiochip1",
        .drdy_line = (unsigned int)line,
        .v_ref = 2.5,
        .refmux = ADS1262_REF_INTERNAL,
        .rate = ADS1262_RATE_400,
        .filter = ADS1262_FILTER_SINC3,
        .gain = ADS1262_GAIN_1,
        .pos = ADS1262_AIN0, .neg = ADS1262_AIN1,
        .chop = false,
        .timeout_ms = 1000,
    };
    ads1262_t adc;
    int32_t raw;

    int result = ads1262_open(&adc, &cfg);
    if (result != ADS1262_OK) {
        fprintf(stderr, "Cannot open ADS1262: %s\n", ads1262_strerror(result));
        return EXIT_FAILURE;
    }
    printf("ADS1262 at %.0f SPS, DRDY %s\n", ads1262_sps(cfg.rate),
           cfg.drdy_chip ? "on GPIO" : "polled");

    /* Register dump */
    printf("\nReg  Binary    Hex\n");
    for (uint8_t reg = ADS1262_REG_ID; reg <= ADS1262_REG_GPIODAT; reg++) {
        uint8_t value;
        if (ads1262_read_register(&adc, reg, &value) == ADS1262_OK) {
            printf("0x%02X ", reg);
            for (int bit = 7; bit >= 0; bit--) {
                putchar(value >> bit & 1 ? '1' : '0');
            }
            printf("  0x%02X\n", value);
        }
    }

    /* Single differential reading AIN0 - AIN1 */
    if ((result = ads1262_read(&adc, &raw)) != ADS1262_OK) {
        goto error;
    }
    printf("\nAIN0-AIN1: %.9f V\n", ads1262_to_volts(&adc, raw));

    /* All 10 inputs single-ended against AINCOM */
    uint8_t inputs[10][2];
    int32_t values[10];
    for (int i = 0; i < 10; i++) {
        inputs[i][0] = (uint8_t)(ADS1262_AIN0 + i);
        inputs[i][1] = ADS1262_AINCOM;
    }
    if ((result = ads1262_scan(&adc, inputs, 10, values)) != ADS1262_OK) {
        goto error;
    }
    printf("\n");
    for (int i = 0; i < 10; i++) {
        printf("AIN%d-COM: %.9f V\n", i, ads1262_to_volts(&adc, values[i]));
    }

    /* Internal temperature sensor and supply monitors (gain 1, no chop) */
    uint8_t internal[3][2] = {
        { ADS1262_TEMP, ADS1262_TEMP },
        { ADS1262_AVDD_MON, ADS1262_AVDD_MON },
        { ADS1262_DVDD_MON, ADS1262_DVDD_MON },
    };
    if ((result = ads1262_scan(&adc, internal, 3, values)) != ADS1262_OK) {
        goto error;
    }
    printf("\nChip temperature: %.2f C\n", ads1262_to_celsius(&adc, values[0]));
    printf("AVDD: %.4f V\n", 4 * ads1262_to_volts(&adc, values[1]));
    printf("DVDD: %.4f V\n", 4 * ads1262_to_volts(&adc, values[2]));

    /* Consecutive conversions of AIN0 - AIN1 */
    int32_t samples[STREAM_SAMPLES];
    size_t count = 0;
    if ((result = ads1262_set_input(&adc, ADS1262_AIN0, ADS1262_AIN1)) != ADS1262_OK ||
        (result = ads1262_read_stream(&adc, samples, STREAM_SAMPLES, &count)) != ADS1262_OK) {
        fprintf(stderr, "Stream stopped after %zu samples\n", count);
        goto error;
    }
    double sum = 0.0;
    for (size_t i = 0; i < count; i++) {
        sum += ads1262_to_volts(&adc, samples[i]);
    }
    printf("\nAIN0-AIN1 average of %zu samples: %.9f V\n", count, sum / (double)count);

    ads1262_close(&adc);
    return EXIT_SUCCESS;

error:
    fprintf(stderr, "ADS1262 error: %s\n", ads1262_strerror(result));
    ads1262_close(&adc);
    return EXIT_FAILURE;
}
