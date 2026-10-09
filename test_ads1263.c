/*
 *  Hardware-free self-check: emulates ADS1263 behind fake ioctl (spidev) and poll.
 *  DRDY is tested both polled through the status byte and on an emulated GPIO line,
 *  whose edge events go through a real pipe (so read/O_NONBLOCK are the kernel's).
 *  Build & run: make test
 */

#define _GNU_SOURCE                                    /* ppoll */
#undef NDEBUG                                          /* The test is made of asserts */
#include <assert.h>
#include <errno.h>
#include <locale.h>
#include <math.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include "ads1263_lib.h"

/* --- Emulated ADS1263 --- */
static const uint8_t RESET_REGS[27] = {
    0x22, 0x11, 0x05, 0x00, 0x80, 0x04, 0x01, 0, 0, 0, 0, 0, 0x40, 0xBB, 0, 0, 0, 0, 0, 0, 0,
    0x00, 0x01, 0, 0, 0, 0x40                         /* ADC2 */
};
static uint8_t chip_id = 0x22;                         /* DEV_ID 001 = ADS1263 */
static int spi_fd = -1;
static uint32_t spi_speed = 8000000;                   /* Expected SCLK */
static uint8_t regs[27];
static uint8_t last_cmds[8];                           /* Last single-byte commands */
static int running;                                    /* Conversions started (START1) */
static uint32_t pending;                               /* Conversion in progress */
static int pending_valid;
static uint32_t data_reg;                              /* Holding register */
static int cleared, cleared2;                          /* Restart zeroed the holding register, checksum too */
static int new_flag;                                   /* ADC1 bit of the status byte */
static int looks;                                      /* Polled reads since conversion start */
static int calibrations;
static uint8_t cal_mux;                                /* INPMUX when calibration was sent */

/* Emulated DRDY GPIO line: conversion finishes when the library looks at the pin */
static int gpio_rd = -1, gpio_wr = -1;                 /* Pipe carrying edge events */
static int drdy_level = 1;
static int edges_per_conversion = 1;                   /* 2 = a conversion was skipped */
static int commits_left = -1;                          /* >= 0: DRDY stalls after that many */
static int stale_edge;                                 /* Add a late-delivered old event */
static int noise;                                      /* Edges keep coming, DRDY stays high */
static int poll_eintr;                                 /* Next poll() is interrupted */
static int no_chip;                                    /* MISO floats high: reads are 0xFF */
static int miso_low;                                   /* MISO stuck low: reads are 0x00 */
static int bad_checksum;                               /* Corrupt the checksum byte */
static int stale_flag;                                 /* Restart leaves an old unread result */
static int late_read;                                  /* Data read already once (read too late) */

/* Emulated ADC2: always polled, a conversion is done at the second look after its start */
static int running2, pending2_valid, new_flag2, looks2, calibrations2;
static uint32_t pending2, data2_reg;
static uint8_t cal_mux2;

/* Each input pair converts to a distinct code: INPMUX in every byte (AINCOM as pos gives negative) */
static int32_t code_for(uint8_t mux)
{
    return (int32_t)(mux * 0x01010101u);
}

static void start_next_conversion(void)
{
    pending = (uint32_t)code_for(regs[ADS1263_REG_INPMUX]);
    pending_valid = 1;
    looks = 0;
    drdy_level = 1;
}

static void restart(void)
{
    if (!running) {
        return;
    }
    start_next_conversion();
    if (stale_flag) {                                  /* Unread result of the old input */
        data_reg = 0xDEADBEEF;
        new_flag = 1;
    } else {                                           /* Seen on hardware */
        data_reg = 0;
        cleared = 1;
    }
}

/* ADC2 code of an input pair: INPMUX in each of the 3 bytes, signed 24-bit */
static int32_t code2_for(uint8_t mux)
{
    int32_t code = mux * 0x010101;
    return code >= 0x800000 ? code - 0x1000000 : code;
}

static void restart2(void)
{
    if (running2) {
        pending2 = (uint32_t)code2_for(regs[ADS1263_REG_ADC2MUX]) & 0xFFFFFF;
        pending2_valid = 1;
        looks2 = 0;
        data2_reg = 0;
        cleared2 = 1;
    }
}

static void read_data2(uint8_t *out)
{
    if (pending2_valid && looks2++ > 0 && commits_left != 0) {
        pending2_valid = 0;
        data2_reg = pending2;
        new_flag2 = 1;
        cleared2 = 0;
    }
    out[1] = new_flag2 ? ADS1263_STATUS_ADC2 : 0;
    for (int i = 0; i < 3; i++) {
        out[2 + i] = (uint8_t)(data2_reg >> (16 - 8 * i));
    }
    out[5] = 0;                                        /* Pad byte */
    out[6] = cleared2 ? 0 : (uint8_t)(out[2] + out[3] + out[4] + 0x9B + bad_checksum);
    new_flag2 = 0;
    if (!pending2_valid) {                             /* Continuous conversions */
        restart2();
    }
}

static void finish_conversion(void)
{
    if (!pending_valid || commits_left == 0) {
        return;
    }
    pending_valid = 0;
    commits_left -= commits_left > 0;
    data_reg = pending;
    new_flag = 1;
    cleared = 0;
    drdy_level = 0;
    if (gpio_wr < 0) {
        return;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    struct gpio_v2_line_event old = { .timestamp_ns = 1 }, event = {
        .timestamp_ns = (uint64_t)now.tv_sec * 1000000000u + (uint64_t)now.tv_nsec
    };
    if (stale_edge) {
        ssize_t written = write(gpio_wr, &old, sizeof(old));
        assert(written == (ssize_t)sizeof(old));
    }
    for (int i = 0; i < edges_per_conversion; i++) {
        ssize_t written = write(gpio_wr, &event, sizeof(event));
        assert(written == (ssize_t)sizeof(event));
    }
}

static void read_data(uint8_t *out)
{
    if (gpio_rd < 0 && looks++ > 0) {                  /* Polled: done at the second look */
        finish_conversion();
    }
    out[1] = new_flag && !late_read ? ADS1263_STATUS_ADC1 : 0;
    for (int i = 0; i < 4; i++) {
        out[2 + i] = (uint8_t)(data_reg >> (24 - 8 * i));
    }
    out[6] = cleared ? 0 : (uint8_t)(out[2] + out[3] + out[4] + out[5] + 0x9B + bad_checksum);
    new_flag = 0;
    drdy_level = 1;                                    /* DRDY goes high on the read */
    if (running && !pending_valid) {                   /* Continuous conversion mode */
        start_next_conversion();
    }
}

static void spi_message(const struct spi_ioc_transfer *t)
{
    const uint8_t *tx = (const uint8_t *)(unsigned long)t->tx_buf;
    uint8_t *rx = (uint8_t *)(unsigned long)t->rx_buf;
    uint8_t cmd = tx[0];

    if ((cmd & 0xE0) == ADS1263_CMD_RREG || (cmd & 0xE0) == ADS1263_CMD_WREG) {
        uint8_t start = cmd & 0x1F, n = (uint8_t)(tx[1] + 1);
        assert(start + n <= (int)sizeof(regs) && t->len == 2u + n);
        if ((cmd & 0xE0) == ADS1263_CMD_RREG) {
            assert(rx);
            memcpy(rx + 2, regs + start, n);
        } else {
            for (uint8_t i = 0; i < n; i++) {
                if (start + i != ADS1263_REG_ID) {
                    regs[start + i] = tx[2 + i];
                }
            }
            if ((start <= ADS1263_REG_INPMUX && start + n > ADS1263_REG_MODE0) ||
                (start <= ADS1263_REG_REFMUX && start + n > ADS1263_REG_IDACMUX)) {
                restart();                             /* Datasheet table 9-34, ADC restart */
            }
            if (start <= ADS1263_REG_ADC2MUX && start + n > ADS1263_REG_ADC2CFG) {
                restart2();
            }
        }
    } else if (cmd == ADS1263_CMD_RDATA1) {
        assert(rx && t->len == 7);
        read_data(rx);
    } else if (cmd == ADS1263_CMD_RDATA2) {
        assert(rx && t->len == 7);
        read_data2(rx);
    } else {
        assert(t->len == 1);
        memmove(last_cmds + 1, last_cmds, sizeof(last_cmds) - 1);
        last_cmds[0] = cmd;
        if (cmd == ADS1263_CMD_RESET) {
            memcpy(regs, RESET_REGS, sizeof(regs));
            regs[ADS1263_REG_ID] = chip_id;
            running = pending_valid = new_flag = 0;
            running2 = pending2_valid = new_flag2 = 0;
        } else if (cmd == ADS1263_CMD_START1) {
            running = 1;
            restart();
        } else if (cmd == ADS1263_CMD_STOP1) {
            running = 0;
        } else if (cmd == ADS1263_CMD_START2) {
            running2 = 1;
            restart2();
        } else if (cmd == ADS1263_CMD_STOP2) {
            running2 = pending2_valid = 0;
        } else if (cmd == ADS1263_CMD_SFOCAL2 || cmd == ADS1263_CMD_SYOCAL2 || cmd == ADS1263_CMD_SYGCAL2) {
            assert(running2);
            calibrations2++;
            cal_mux2 = regs[ADS1263_REG_ADC2MUX];
            restart2();
        } else {
            assert(cmd == ADS1263_CMD_SFOCAL1 || cmd == ADS1263_CMD_SYOCAL1 || cmd == ADS1263_CMD_SYGCAL1);
            assert(running);                           /* Calibration needs running conversions */
            calibrations++;
            cal_mux = regs[ADS1263_REG_INPMUX];
            start_next_conversion();
        }
    }

    if (rx && (no_chip || miso_low)) {
        memset(rx, no_chip ? 0xFF : 0x00, t->len);
    }
}

int ioctl(int fd, unsigned long request, ...)
{
    va_list ap;
    va_start(ap, request);
    void *arg = va_arg(ap, void *);
    va_end(ap);

    if (request == SPI_IOC_WR_MODE) {
        assert(*(uint8_t *)arg == SPI_MODE_1);
        spi_fd = fd;
    } else if (request == SPI_IOC_WR_MAX_SPEED_HZ) {
        assert(fd == spi_fd && *(uint32_t *)arg == spi_speed);
    } else if (request == SPI_IOC_MESSAGE(1)) {
        assert(fd == spi_fd);
        spi_message(arg);
    } else if (request == GPIO_V2_GET_LINE_IOCTL) {     /* Request DRDY line */
        struct gpio_v2_line_request *req = arg;
        int p[2];
        assert(req->num_lines == 1 && req->offsets[0] == 22);
        assert(req->config.flags == (GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_FALLING));
        int piped = pipe(p);
        assert(piped == 0);
        gpio_rd = req->fd = p[0];
        gpio_wr = p[1];
    } else {
        struct gpio_v2_line_values *values = arg;     /* Read DRDY level */
        assert(request == GPIO_V2_LINE_GET_VALUES_IOCTL && fd == gpio_rd && values->mask == 1);
        finish_conversion();
        values->bits = (uint64_t)drdy_level;
    }
    return 0;
}

int poll(struct pollfd *fds, nfds_t nfds, int timeout)  /* Wait for DRDY edge */
{
    (void)timeout;
    assert(nfds == 1);                                 /* glibc marks fds write-only: no reads */
    if (poll_eintr) {
        poll_eintr = 0;
        errno = EINTR;
        return -1;
    }
    if (noise) {
        struct gpio_v2_line_event event = { .timestamp_ns = 1 };
        ssize_t written = write(gpio_wr, &event, sizeof(event));
        assert(written == (ssize_t)sizeof(event));
        fds[0].revents = POLLIN;
        return 1;
    }
    finish_conversion();
    struct timespec now = { 0 };
    return ppoll(fds, nfds, &now, NULL);               /* Real pipe state, 0 = timeout */
}

/* Read, scan and stream must return each input's own conversion */
static void check_acquisition(ads1263_t *adc)
{
    int32_t raw;
    assert(ads1263_set_input(adc, ADS1263_AIN0, ADS1263_AIN1) == ADS1263_OK);
    assert(ads1263_read(adc, &raw) == ADS1263_OK);
    assert(raw == code_for(0x01));
    assert(ads1263_set_input(adc, ADS1263_AINCOM, ADS1263_AIN0) == ADS1263_OK);
    assert(ads1263_read(adc, &raw) == ADS1263_OK);
    assert(raw == code_for(0xA0) && raw < 0);          /* Sign */

    /* An unread result from before the restart is never returned */
    stale_flag = 1;
    assert(ads1263_read(adc, &raw) == ADS1263_OK && raw == code_for(0xA0));
    stale_flag = 0;

    uint8_t inputs[3][2] = {
        { ADS1263_AIN0, ADS1263_AINCOM }, { ADS1263_AIN8, ADS1263_AIN9 }, { ADS1263_TEMP, ADS1263_TEMP }
    };
    int32_t values[3];
    assert(ads1263_scan(adc, inputs, 3, values) == ADS1263_OK);
    assert(values[0] == code_for(0x0A) && values[1] == code_for(0x89) && values[2] == code_for(0xBB));
    assert(adc->cfg.pos == ADS1263_TEMP && adc->cfg.neg == ADS1263_TEMP);  /* Last pair stays */

    int32_t stream[5];
    size_t count;
    assert(ads1263_set_input(adc, ADS1263_AIN4, ADS1263_AIN5) == ADS1263_OK);
    assert(ads1263_read_stream(adc, stream, 5, &count) == ADS1263_OK && count == 5);
    for (int i = 0; i < 5; i++) {
        assert(stream[i] == code_for(0x45));
    }
    assert(adc->status & ADS1263_STATUS_ADC1);
}

/* ADC2: configuration, calibration and acquisition, independent of ADC1 */
static void check_adc2(ads1263_t *adc)
{
    int32_t raw;
    ads1263_adc2_config_t cfg2 = {
        .v_ref = 2.5, .refmux = ADS1263_REF_INTERNAL, .drate = ADS1263_ADC2_DRATE_800,
        .gain = ADS1263_GAIN_128, .pos = ADS1263_AIN6, .neg = ADS1263_AIN7
    };

    /* Invalid configuration is rejected */
    const ads1263_adc2_config_t bads[] = {
        { .v_ref = 2.5, .drate = 0, .gain = 256, .neg = 1 },
        { .v_ref = 2.5, .drate = 0, .gain = 3, .neg = 1 },
        { .v_ref = 2.5, .drate = 4, .gain = 1, .neg = 1 },
        { .v_ref = 2.5, .refmux = 0x0A, .gain = 1, .neg = 1 },      /* Mixed pair: ADC1 only */
        { .v_ref = 0.5, .gain = 1, .neg = 1 },
        { .v_ref = 2.5, .gain = 1, .pos = 3, .neg = 3 },
    };
    for (size_t i = 0; i < sizeof(bads) / sizeof(bads[0]); i++) {
        assert(ads1263_adc2_start(adc, &bads[i]) == ADS1263_ERROR_PARAMETER);
    }
    assert(ads1263_adc2_start(adc, NULL) == ADS1263_ERROR_PARAMETER);

    /* Start writes both registers, starts conversions and self-calibrates with open inputs */
    int cals = calibrations, cals2 = calibrations2;
    uint8_t inpmux = regs[ADS1263_REG_INPMUX];
    assert(ads1263_adc2_start(adc, &cfg2) == ADS1263_OK);
    assert(regs[ADS1263_REG_ADC2CFG] == 0xC7 && regs[ADS1263_REG_ADC2MUX] == 0x67);
    assert(cal_mux2 == 0xFF && calibrations2 == cals2 + 1 && running2);
    assert(last_cmds[0] == ADS1263_CMD_SFOCAL2 && last_cmds[1] == ADS1263_CMD_START2);
    assert(calibrations == cals && regs[ADS1263_REG_INPMUX] == inpmux);  /* ADC1 untouched */

    /* Setters self-calibrate */
    assert(ads1263_adc2_set_gain(adc, ADS1263_GAIN_64) == ADS1263_OK);
    assert(regs[ADS1263_REG_ADC2CFG] == 0xC6 && calibrations2 == cals2 + 2);
    assert(ads1263_adc2_set_gain(adc, 3) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_adc2_set_drate(adc, ADS1263_ADC2_DRATE_100) == ADS1263_OK);
    assert(regs[ADS1263_REG_ADC2CFG] == 0x46 && calibrations2 == cals2 + 3);
    assert(ads1263_adc2_set_drate(adc, 4) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_adc2_set_reference(adc, ADS1263_REF_AIN4_AIN5, 5.0) == ADS1263_OK);
    assert(regs[ADS1263_REG_ADC2CFG] == 0x5E && adc->cfg2.v_ref == 5.0);
    assert(ads1263_adc2_set_reference(adc, 0x0A, 2.5) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_adc2_set_reference(adc, ADS1263_REF_INTERNAL, 2.5) == ADS1263_OK);
    assert(ads1263_adc2_set_gain(adc, ADS1263_GAIN_1) == ADS1263_OK);
    assert(ads1263_adc2_set_drate(adc, ADS1263_ADC2_DRATE_800) == ADS1263_OK);
    assert(regs[ADS1263_REG_ADC2CFG] == 0xC0 && regs[ADS1263_REG_ADC2MUX] == 0x67);
    assert(ads1263_adc2_calibrate(adc, ADS1263_CMD_SYGCAL2) == ADS1263_OK && cal_mux2 == 0x67);
    assert(ads1263_adc2_calibrate(adc, ADS1263_CMD_SFOCAL1) == ADS1263_ERROR_PARAMETER);

    /* Read restarts ADC2, signed 24-bit codes */
    assert(ads1263_adc2_read(adc, &raw) == ADS1263_OK && raw == code2_for(0x67));
    assert(last_cmds[0] == ADS1263_CMD_START2 && last_cmds[1] == ADS1263_CMD_STOP2);
    assert(adc->status2 & ADS1263_STATUS_ADC2);
    assert(ads1263_adc2_set_input(adc, ADS1263_AINCOM, ADS1263_AIN0) == ADS1263_OK);
    assert(regs[ADS1263_REG_ADC2MUX] == 0xA0 && regs[ADS1263_REG_INPMUX] == inpmux);
    assert(ads1263_adc2_read(adc, &raw) == ADS1263_OK && raw == code2_for(0xA0) && raw < 0);
    assert(ads1263_adc2_set_input(adc, ADS1263_AIN2, ADS1263_AIN2) == ADS1263_ERROR_PARAMETER);

    uint8_t inputs[2][2] = { { ADS1263_AIN2, ADS1263_AINCOM }, { ADS1263_TEMP, ADS1263_TEMP } };
    int32_t values[2];
    assert(ads1263_adc2_scan(adc, inputs, 2, values) == ADS1263_OK);
    assert(values[0] == code2_for(0x2A) && values[1] == code2_for(0xBB));
    assert(adc->cfg2.pos == ADS1263_TEMP && adc->cfg2.neg == ADS1263_TEMP);

    int32_t stream[3];
    size_t count;
    assert(ads1263_adc2_read_stream(adc, stream, 3, &count) == ADS1263_OK && count == 3);
    assert(stream[0] == code2_for(0xBB) && stream[2] == code2_for(0xBB));

    /* ADC1 keeps its own input and data */
    assert(ads1263_read(adc, &raw) == ADS1263_OK && raw == code_for(inpmux));

    bad_checksum = 1;
    assert(ads1263_adc2_read(adc, &raw) == ADS1263_ERROR_CHECKSUM);
    bad_checksum = 0;
    commits_left = 0;
    assert(ads1263_adc2_read(adc, &raw) == ADS1263_ERROR_TIMEOUT);
    commits_left = -1;

    /* Volts: full scale is +-Vref / gain at +-2^23 */
    assert(fabs(ads1263_adc2_to_volts(adc, 0x400000) - 1.25) < 1e-9);
    assert(fabs(ads1263_adc2_to_volts(adc, -0x800000) + 2.5) < 1e-9);
    assert(fabs(ads1263_adc2_to_celsius(adc, (int32_t)(0.1224 / 2.5 * 8388608.0)) - 25.0) < 1e-3);
    assert(ads1263_adc2_sps(ADS1263_ADC2_DRATE_400) == 400.0f && ads1263_adc2_sps(4) == 0.0f);
}

static void write_text(const char *path, const char *text)
{
    FILE *f = fopen(path, "w");
    assert(f && fputs(text, f) >= 0 && fclose(f) == 0);
}

/* Measured reference from a file: exactly one number in range, decimal point in any locale */
static void check_load_vref(void)
{
    char dir[] = "/tmp/ads1263-test-XXXXXX", sub[64], path[96];
    double v = 0;
    assert(mkdtemp(dir));
    snprintf(path, sizeof(path), "%s/vref", dir);

    errno = 0;
    assert(ads1263_load_vref(path, &v) == ADS1263_ERROR_PARAMETER && errno == ENOENT);
    write_text(path, "# REFOUT measured 2026-10-08\n\n  2.4987 \n");
    assert(ads1263_load_vref(path, &v) == ADS1263_OK && v == 2.4987);
    write_text(path, "2.5");                           /* No newline at the end */
    assert(ads1263_load_vref(path, &v) == ADS1263_OK && v == 2.5);
    if (setlocale(LC_NUMERIC, "cs_CZ.UTF-8")) {        /* Decimal comma locale, if installed */
        assert(ads1263_load_vref(path, &v) == ADS1263_OK && v == 2.5);
        setlocale(LC_NUMERIC, "C");
    }
    static const char *bad[] = {
        "", "# only a comment\n", "2,4987\n", "2.4987 V\n", "2.4987\n2.5\n", "24987\n", "2.52\n", "nan\n"
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        write_text(path, bad[i]);
        v = 0;
        errno = 0;
        assert(ads1263_load_vref(path, &v) == ADS1263_ERROR_PARAMETER && errno == EINVAL && v == 0);
    }
    assert(ads1263_load_vref(path, NULL) == ADS1263_ERROR_PARAMETER);

    /* Default file: $XDG_CONFIG_HOME/ads1263/vref (absolute only), else ~/.config/ads1263/vref */
    snprintf(sub, sizeof(sub), "%s/ads1263", dir);
    assert(mkdir(sub, 0700) == 0);
    snprintf(path, sizeof(path), "%s/ads1263/vref", dir);
    write_text(path, "2.501\n");
    setenv("XDG_CONFIG_HOME", dir, 1);
    assert(ads1263_load_vref(NULL, &v) == ADS1263_OK && v == 2.501);
    setenv("XDG_CONFIG_HOME", "relative", 1);
    unsetenv("HOME");
    errno = 0;
    assert(ads1263_load_vref(NULL, &v) == ADS1263_ERROR_PARAMETER && errno == ENOENT);

    snprintf(sub, sizeof(sub), "%s/vref", dir);
    assert(remove(path) == 0 && remove(sub) == 0);
    snprintf(sub, sizeof(sub), "%s/ads1263", dir);
    assert(rmdir(sub) == 0 && rmdir(dir) == 0);
}

int main(void)
{
    ads1263_config_t cfg = {
        .spi_device = "/dev/null", .spi_speed_hz = 8000000,  /* SPI ioctls go to the fake */
        .v_ref = 2.5, .refmux = ADS1263_REF_INTERNAL,
        .drate = ADS1263_DRATE_38400, .filter = ADS1263_FILTER_SINC1, .gain = ADS1263_GAIN_8,
        .pos = ADS1263_AIN3, .neg = ADS1263_AINCOM, .timeout_ms = 100
    };
    ads1263_t adc;
    int32_t raw;

    /* Invalid configuration is rejected before touching the device, fds safe to close */
    ads1263_config_t bad = cfg;
    bad.v_ref = 0.5;
    assert(ads1263_open(&adc, &bad) == ADS1263_ERROR_PARAMETER);
    bad.v_ref = NAN;
    assert(ads1263_open(&adc, &bad) == ADS1263_ERROR_PARAMETER);
    assert(adc.spi_fd == -1 && adc.drdy_fd == -1);
    const ads1263_config_t bads[] = {
        { .spi_device = "/dev/null", .spi_speed_hz = 8000001, .v_ref = 2.5, .gain = 1, .neg = 1, .timeout_ms = 1 },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 1, .neg = 1, .timeout_ms = 0 },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 64, .neg = 1, .timeout_ms = 1 },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 1, .pos = 2, .neg = 2, .timeout_ms = 1 },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 1, .pos = 15, .neg = 15, .timeout_ms = 1 },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 1, .neg = 1, .timeout_ms = 1,
          .drate = ADS1263_DRATE_400, .filter = ADS1263_FILTER_FIR },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 1, .neg = 1, .timeout_ms = 1,
          .drate = 16 },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 1, .neg = 1, .timeout_ms = 1,
          .refmux = 0x2D },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 2, .neg = 1, .timeout_ms = 1,
          .bypass = true },                            /* Bypass is gain 1 only */
    };
    for (size_t i = 0; i < sizeof(bads) / sizeof(bads[0]); i++) {
        assert(ads1263_open(&adc, &bads[i]) == ADS1263_ERROR_PARAMETER);
    }
    assert(ads1263_open(&adc, NULL) == ADS1263_ERROR_PARAMETER);

    /* No chip on the bus: open fails at once instead of a timeout on the first read */
    no_chip = 1;
    assert(ads1263_open(&adc, &cfg) == ADS1263_ERROR_COMMUNICATION);
    assert(adc.spi_fd == -1 && adc.drdy_fd == -1);
    no_chip = 0;
    miso_low = 1;
    assert(ads1263_open(&adc, &cfg) == ADS1263_ERROR_COMMUNICATION);
    miso_low = 0;

    /* ADS1262 (DEV_ID 000) has no ADC2: rejected */
    chip_id = 0x02;
    assert(ads1263_open(&adc, &cfg) == ADS1263_ERROR_COMMUNICATION);
    chip_id = 0x22;

    /* ===== DRDY polled through the status byte ===== */

    /* Open writes all settings, starts conversions and self-calibrates with open inputs */
    assert(ads1263_open(&adc, &cfg) == ADS1263_OK);
    assert(adc.spi_fd == spi_fd && adc.drdy_fd == -1);
    assert(regs[ADS1263_REG_POWER] == 0x01 && regs[ADS1263_REG_INTERFACE] == 0x05);
    assert(regs[ADS1263_REG_MODE0] == 0x00 && regs[ADS1263_REG_MODE1] == 0x00);
    assert(regs[ADS1263_REG_MODE2] == 0x3F);           /* Gain 8, 38400 SPS */
    assert(regs[ADS1263_REG_INPMUX] == 0x3A);          /* AIN3 - AINCOM, restored after calibration */
    assert(cal_mux == 0xFF && calibrations == 1);
    assert(last_cmds[0] == ADS1263_CMD_SFOCAL1 && last_cmds[1] == ADS1263_CMD_START1);
    assert(running);
    assert(!running2 && adc.cfg2.drate == ADS1263_ADC2_DRATE_10 && adc.cfg2.gain == ADS1263_GAIN_1 &&
           adc.cfg2.pos == ADS1263_AIN0 && adc.cfg2.neg == ADS1263_AIN1);  /* ADC2 at reset values */

    /* Inputs: any pair, internal ones on both sides, never other pos == neg or out of range */
    assert(ads1263_set_input(&adc, ADS1263_AIN0, ADS1263_AIN1) == ADS1263_OK);
    assert(regs[ADS1263_REG_INPMUX] == 0x01);
    assert(ads1263_set_input(&adc, ADS1263_AVDD_MON, ADS1263_AVDD_MON) == ADS1263_OK);
    assert(regs[ADS1263_REG_INPMUX] == 0xCC);
    assert(ads1263_set_input(&adc, ADS1263_AIN2, ADS1263_AIN2) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_set_input(&adc, ADS1263_FLOAT, ADS1263_FLOAT) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_set_input(&adc, 16, ADS1263_AIN0) == ADS1263_ERROR_PARAMETER);
    assert(regs[ADS1263_REG_INPMUX] == 0xCC);

    /* Setters self-calibrate, unless chop is on */
    assert(ads1263_set_gain(&adc, ADS1263_GAIN_32) == ADS1263_OK);
    assert(regs[ADS1263_REG_MODE2] == 0x5F && calibrations == 2);
    assert(ads1263_set_gain(&adc, 64) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_set_gain(&adc, ADS1263_GAIN_1) == ADS1263_OK);
    assert(ads1263_set_drate(&adc, ADS1263_DRATE_19200) == ADS1263_OK);
    assert(regs[ADS1263_REG_MODE2] == 0x0E && calibrations == 4);
    assert(ads1263_set_filter(&adc, ADS1263_FILTER_FIR) == ADS1263_ERROR_PARAMETER);  /* Not at 19200 SPS */
    assert(ads1263_set_filter(&adc, ADS1263_FILTER_SINC4) == ADS1263_OK);
    assert(regs[ADS1263_REG_MODE1] == 0x60 && calibrations == 5);
    assert(ads1263_set_reference(&adc, ADS1263_REF_AIN0_AIN1, 2.048) == ADS1263_OK);
    assert(regs[ADS1263_REG_REFMUX] == 0x09 && adc.cfg.v_ref == 2.048 && calibrations == 6);
    assert(ads1263_set_reference(&adc, 0x2D, 2.5) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_set_reference(&adc, ADS1263_REF_INTERNAL, 6.0) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_set_reference(&adc, ADS1263_REF_INTERNAL, 2.5) == ADS1263_OK);
    assert(ads1263_set_chop(&adc, true) == ADS1263_OK);
    assert(regs[ADS1263_REG_MODE0] == 0x10 && calibrations == 7);
    assert(ads1263_set_gain(&adc, ADS1263_GAIN_1) == ADS1263_OK && calibrations == 7);
    assert(ads1263_set_chop(&adc, false) == ADS1263_OK);
    assert(regs[ADS1263_REG_MODE0] == 0x00 && calibrations == 8);

    /* PGA bypass: gain 1 only, gain and rate setters keep the bypass bit */
    assert(ads1263_set_bypass(&adc, true) == ADS1263_OK);
    assert(regs[ADS1263_REG_MODE2] == 0x8E && adc.cfg.bypass && calibrations == 9);
    assert(ads1263_set_gain(&adc, ADS1263_GAIN_2) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_set_gain(&adc, ADS1263_GAIN_1) == ADS1263_OK && regs[ADS1263_REG_MODE2] == 0x8E);
    assert(ads1263_set_drate(&adc, ADS1263_DRATE_38400) == ADS1263_OK && regs[ADS1263_REG_MODE2] == 0x8F);
    assert(ads1263_set_drate(&adc, ADS1263_DRATE_19200) == ADS1263_OK);
    assert(ads1263_set_bypass(&adc, false) == ADS1263_OK);
    assert(regs[ADS1263_REG_MODE2] == 0x0E && !adc.cfg.bypass);
    assert(ads1263_set_gain(&adc, ADS1263_GAIN_2) == ADS1263_OK);
    assert(ads1263_set_bypass(&adc, true) == ADS1263_ERROR_PARAMETER && regs[ADS1263_REG_MODE2] == 0x1E);
    assert(ads1263_set_gain(&adc, ADS1263_GAIN_1) == ADS1263_OK);
    assert(regs[ADS1263_REG_INPMUX] == 0xCC);          /* Restored after each calibration */

    /* System calibration keeps the selected input */
    assert(ads1263_calibrate(&adc, ADS1263_CMD_SYOCAL1) == ADS1263_OK && cal_mux == 0xCC);
    assert(ads1263_calibrate(&adc, ADS1263_CMD_STOP1) == ADS1263_ERROR_PARAMETER);

    /* IDACs: one nibble each */
    assert(ads1263_set_idac(&adc, 1, ADS1263_AIN5, ADS1263_IDAC_500UA) == ADS1263_OK);
    assert(regs[ADS1263_REG_IDACMUX] == 0xB5 && regs[ADS1263_REG_IDACMAG] == 0x04);
    assert(ads1263_set_idac(&adc, 2, ADS1263_AINCOM, ADS1263_IDAC_3000UA) == ADS1263_OK);
    assert(regs[ADS1263_REG_IDACMUX] == 0xA5 && regs[ADS1263_REG_IDACMAG] == 0xA4);
    assert(ads1263_set_idac(&adc, 3, ADS1263_AIN0, ADS1263_IDAC_50UA) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_set_idac(&adc, 1, 12, ADS1263_IDAC_50UA) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_set_idac(&adc, 1, ADS1263_AIN0, 11) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_set_idac(&adc, 1, ADS1263_IDAC_NC, ADS1263_IDAC_OFF) == ADS1263_OK);
    assert(regs[ADS1263_REG_IDACMUX] == 0xAB && regs[ADS1263_REG_IDACMAG] == 0xA0);

    /* Read restarts conversion, so the value always belongs to the current input */
    assert(ads1263_read(&adc, &raw) == ADS1263_OK && raw == code_for(0xCC));
    assert(last_cmds[0] == ADS1263_CMD_START1 && last_cmds[1] == ADS1263_CMD_STOP1);
    check_acquisition(&adc);
    uint8_t bad_inputs[2][2] = { { ADS1263_AIN0, ADS1263_AIN1 }, { ADS1263_AIN1, ADS1263_AIN1 } };
    int32_t values[2];
    assert(ads1263_scan(&adc, bad_inputs, 2, values) == ADS1263_ERROR_PARAMETER);
    check_adc2(&adc);

    /* Register access covers the ADC2 registers */
    uint8_t reg;
    assert(ads1263_read_register(&adc, ADS1263_REG_ADC2FSC1, &reg) == ADS1263_OK && reg == 0x40);
    assert(ads1263_read_register(&adc, ADS1263_REG_ADC2FSC1 + 1, &reg) == ADS1263_ERROR_PARAMETER);
    assert(ads1263_write_register(&adc, ADS1263_REG_ADC2FSC1 + 1, 0) == ADS1263_ERROR_PARAMETER);

    /* Corrupted data are reported */
    bad_checksum = 1;
    assert(ads1263_read(&adc, &raw) == ADS1263_ERROR_CHECKSUM);
    bad_checksum = 0;

    /* No new data: timeout */
    commits_left = 0;
    assert(ads1263_read(&adc, &raw) == ADS1263_ERROR_TIMEOUT);
    commits_left = -1;

    /* Volts: full scale is +-Vref / gain at +-2^31; temperature 122.4 mV = 25 C */
    assert(fabs(ads1263_to_volts(&adc, 0x40000000) - 1.25) < 1e-9);
    assert(fabs(ads1263_to_volts(&adc, INT32_MIN) + 2.5) < 1e-9);
    assert(fabs(ads1263_to_celsius(&adc, (int32_t)(0.1224 / 2.5 * 2147483648.0)) - 25.0) < 1e-4);
    assert(fabs(ads1263_to_celsius(&adc, (int32_t)(0.1266 / 2.5 * 2147483648.0)) - 35.0) < 1e-4);
    assert(ads1263_sps(ADS1263_DRATE_2_5) == 2.5f && ads1263_sps(16) == 0.0f);
    assert(strcmp(ads1263_strerror(ADS1263_ERROR_CHECKSUM), "Checksum error in conversion data") == 0);
    assert(strcmp(ads1263_strerror(-9), "Unknown error") == 0);

    ads1263_close(&adc);
    assert(adc.spi_fd == -1 && adc.drdy_fd == -1);

    /* ===== DRDY on GPIO ===== */

    cfg.drdy_chip = "/dev/null";                       /* Line request goes to fake ioctl */
    cfg.drdy_line = 22;
    cfg.drate = ADS1263_DRATE_100;                       /* 10 ms period */
    cfg.timeout_ms = 1;                                /* Calibration must not use it alone */
    cfg.spi_speed_hz = spi_speed = 4000000;
    cfg.gain = ADS1263_GAIN_1;
    cfg.bypass = true;                                 /* Open writes the bypass bit */
    assert(ads1263_open(&adc, &cfg) == ADS1263_OK);
    assert(adc.drdy_fd == gpio_rd);
    assert(regs[ADS1263_REG_MODE2] == 0x87);           /* Bypass, gain 1, 100 SPS */
    check_acquisition(&adc);
    check_adc2(&adc);                                  /* ADC2 never waits on the ADC1 pin */

    /* Signal while waiting for DRDY is not an error */
    int32_t stream[5];
    size_t count;
    poll_eintr = 1;
    assert(ads1263_read_stream(&adc, stream, 5, &count) == ADS1263_OK && count == 5);
    assert(!poll_eintr);

    /* Late-delivered events of old conversions are ignored */
    stale_edge = 1;
    assert(ads1263_read_stream(&adc, stream, 5, &count) == ADS1263_OK && count == 5);
    stale_edge = 0;

    /* Skipped conversion is reported */
    edges_per_conversion = 2;
    assert(ads1263_read_stream(&adc, stream, 5, &count) == ADS1263_ERROR_OVERRUN && count == 0);
    edges_per_conversion = 1;

    /* Read after the next conversion (data not new) is reported */
    late_read = 1;
    assert(ads1263_read_stream(&adc, stream, 5, &count) == ADS1263_ERROR_OVERRUN && count == 0);
    late_read = 0;

    /* DRDY stalls: timeout */
    commits_left = 0;
    assert(ads1263_read(&adc, &raw) == ADS1263_ERROR_TIMEOUT);
    commits_left = 2;
    assert(ads1263_read_stream(&adc, stream, 5, &count) == ADS1263_ERROR_TIMEOUT && count == 2);

    /* Noisy DRDY line: edges keep coming but DRDY stays high, the deadline still holds */
    noise = 1;
    commits_left = 0;
    assert(ads1263_read(&adc, &raw) == ADS1263_ERROR_TIMEOUT);
    assert(ads1263_read_stream(&adc, stream, 5, &count) == ADS1263_ERROR_TIMEOUT && count == 0);
    noise = 0;

    /* GPIO line gone (POLLHUP): error instead of spinning */
    close(gpio_wr);
    assert(ads1263_read_stream(&adc, stream, 5, &count) == ADS1263_ERROR_COMMUNICATION && count == 0);
    commits_left = -1;

    ads1263_close(&adc);
    assert(adc.spi_fd == -1 && adc.drdy_fd == -1);
    ads1263_close(NULL);

    check_load_vref();

    puts("All tests passed");
    return 0;
}
