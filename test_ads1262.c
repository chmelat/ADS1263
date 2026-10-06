/*
 *  Hardware-free self-check: emulates ADS1262 behind fake ioctl (spidev) and poll.
 *  DRDY is tested both polled through the status byte and on an emulated GPIO line,
 *  whose edge events go through a real pipe (so read/O_NONBLOCK are the kernel's).
 *  Build & run: make test
 */

#define _GNU_SOURCE                                    /* ppoll */
#undef NDEBUG                                          /* The test is made of asserts */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include "ads1262_lib.h"

/* --- Emulated ADS1262 --- */
static const uint8_t RESET_REGS[21] = {
    0x02, 0x11, 0x05, 0x00, 0x80, 0x04, 0x01, 0, 0, 0, 0, 0, 0x40, 0xBB, 0, 0, 0, 0, 0, 0, 0
};
static int spi_fd = -1;
static uint32_t spi_speed = 8000000;                   /* Expected SCLK */
static uint8_t regs[21];
static uint8_t last_cmds[8];                           /* Last single-byte commands */
static int running;                                    /* Conversions started (START1) */
static uint32_t pending;                               /* Conversion in progress */
static int pending_valid;
static uint32_t data_reg;                              /* Holding register */
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

/* Each input pair converts to a distinct code: INPMUX in every byte (AINCOM as pos gives negative) */
static int32_t code_for(uint8_t mux)
{
    return (int32_t)(mux * 0x01010101u);
}

static void start_next_conversion(void)
{
    pending = (uint32_t)code_for(regs[ADS1262_REG_INPMUX]);
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
    out[1] = new_flag && !late_read ? ADS1262_STATUS_ADC1 : 0;
    for (int i = 0; i < 4; i++) {
        out[2 + i] = (uint8_t)(data_reg >> (24 - 8 * i));
    }
    out[6] = (uint8_t)(out[2] + out[3] + out[4] + out[5] + 0x9B + bad_checksum);
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

    if ((cmd & 0xE0) == ADS1262_CMD_RREG || (cmd & 0xE0) == ADS1262_CMD_WREG) {
        uint8_t start = cmd & 0x1F, n = (uint8_t)(tx[1] + 1);
        assert(start + n <= (int)sizeof(regs) && t->len == 2u + n);
        if ((cmd & 0xE0) == ADS1262_CMD_RREG) {
            assert(rx);
            memcpy(rx + 2, regs + start, n);
        } else {
            for (uint8_t i = 0; i < n; i++) {
                if (start + i != ADS1262_REG_ID) {
                    regs[start + i] = tx[2 + i];
                }
            }
            if ((start <= ADS1262_REG_INPMUX && start + n > ADS1262_REG_MODE0) ||
                (start <= ADS1262_REG_REFMUX && start + n > ADS1262_REG_IDACMUX)) {
                restart();                             /* Datasheet table 9-34, ADC restart */
            }
        }
    } else if (cmd == ADS1262_CMD_RDATA1) {
        assert(rx && t->len == 7);
        read_data(rx);
    } else {
        assert(t->len == 1);
        memmove(last_cmds + 1, last_cmds, sizeof(last_cmds) - 1);
        last_cmds[0] = cmd;
        if (cmd == ADS1262_CMD_RESET) {
            memcpy(regs, RESET_REGS, sizeof(regs));
            running = pending_valid = new_flag = 0;
        } else if (cmd == ADS1262_CMD_START1) {
            running = 1;
            restart();
        } else if (cmd == ADS1262_CMD_STOP1) {
            running = 0;
        } else {
            assert(cmd == ADS1262_CMD_SFOCAL1 || cmd == ADS1262_CMD_SYOCAL1 || cmd == ADS1262_CMD_SYGCAL1);
            assert(running);                           /* Calibration needs running conversions */
            calibrations++;
            cal_mux = regs[ADS1262_REG_INPMUX];
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
static void check_acquisition(ads1262_t *adc)
{
    int32_t raw;
    assert(ads1262_set_input(adc, ADS1262_AIN0, ADS1262_AIN1) == ADS1262_OK);
    assert(ads1262_read(adc, &raw) == ADS1262_OK);
    assert(raw == code_for(0x01));
    assert(ads1262_set_input(adc, ADS1262_AINCOM, ADS1262_AIN0) == ADS1262_OK);
    assert(ads1262_read(adc, &raw) == ADS1262_OK);
    assert(raw == code_for(0xA0) && raw < 0);          /* Sign */

    /* An unread result from before the restart is never returned */
    stale_flag = 1;
    assert(ads1262_read(adc, &raw) == ADS1262_OK && raw == code_for(0xA0));
    stale_flag = 0;

    uint8_t inputs[3][2] = {
        { ADS1262_AIN0, ADS1262_AINCOM }, { ADS1262_AIN8, ADS1262_AIN9 }, { ADS1262_TEMP, ADS1262_TEMP }
    };
    int32_t values[3];
    assert(ads1262_scan(adc, inputs, 3, values) == ADS1262_OK);
    assert(values[0] == code_for(0x0A) && values[1] == code_for(0x89) && values[2] == code_for(0xBB));
    assert(adc->cfg.pos == ADS1262_TEMP && adc->cfg.neg == ADS1262_TEMP);  /* Last pair stays */

    int32_t stream[5];
    size_t count;
    assert(ads1262_set_input(adc, ADS1262_AIN4, ADS1262_AIN5) == ADS1262_OK);
    assert(ads1262_read_stream(adc, stream, 5, &count) == ADS1262_OK && count == 5);
    for (int i = 0; i < 5; i++) {
        assert(stream[i] == code_for(0x45));
    }
    assert(adc->status & ADS1262_STATUS_ADC1);
}

int main(void)
{
    ads1262_config_t cfg = {
        .spi_device = "/dev/null", .spi_speed_hz = 8000000,  /* SPI ioctls go to the fake */
        .v_ref = 2.5, .refmux = ADS1262_REF_INTERNAL,
        .rate = ADS1262_RATE_38400, .filter = ADS1262_FILTER_SINC1, .gain = ADS1262_GAIN_8,
        .pos = ADS1262_AIN3, .neg = ADS1262_AINCOM, .timeout_ms = 100
    };
    ads1262_t adc;
    int32_t raw;

    /* Invalid configuration is rejected before touching the device, fds safe to close */
    ads1262_config_t bad = cfg;
    bad.v_ref = 0.5;
    assert(ads1262_open(&adc, &bad) == ADS1262_ERROR_PARAMETER);
    bad.v_ref = NAN;
    assert(ads1262_open(&adc, &bad) == ADS1262_ERROR_PARAMETER);
    assert(adc.spi_fd == -1 && adc.drdy_fd == -1);
    const ads1262_config_t bads[] = {
        { .spi_device = "/dev/null", .spi_speed_hz = 8000001, .v_ref = 2.5, .gain = 1, .neg = 1, .timeout_ms = 1 },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 1, .neg = 1, .timeout_ms = 0 },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 64, .neg = 1, .timeout_ms = 1 },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 1, .pos = 2, .neg = 2, .timeout_ms = 1 },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 1, .pos = 15, .neg = 15, .timeout_ms = 1 },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 1, .neg = 1, .timeout_ms = 1,
          .rate = ADS1262_RATE_400, .filter = ADS1262_FILTER_FIR },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 1, .neg = 1, .timeout_ms = 1,
          .rate = 16 },
        { .spi_device = "/dev/null", .spi_speed_hz = 1, .v_ref = 2.5, .gain = 1, .neg = 1, .timeout_ms = 1,
          .refmux = 0x2D },
    };
    for (size_t i = 0; i < sizeof(bads) / sizeof(bads[0]); i++) {
        assert(ads1262_open(&adc, &bads[i]) == ADS1262_ERROR_PARAMETER);
    }
    assert(ads1262_open(&adc, NULL) == ADS1262_ERROR_PARAMETER);

    /* No chip on the bus: open fails at once instead of a timeout on the first read */
    no_chip = 1;
    assert(ads1262_open(&adc, &cfg) == ADS1262_ERROR_COMMUNICATION);
    assert(adc.spi_fd == -1 && adc.drdy_fd == -1);
    no_chip = 0;
    miso_low = 1;
    assert(ads1262_open(&adc, &cfg) == ADS1262_ERROR_COMMUNICATION);
    miso_low = 0;

    /* ===== DRDY polled through the status byte ===== */

    /* Open writes all settings, starts conversions and self-calibrates with open inputs */
    assert(ads1262_open(&adc, &cfg) == ADS1262_OK);
    assert(adc.spi_fd == spi_fd && adc.drdy_fd == -1);
    assert(regs[ADS1262_REG_POWER] == 0x01 && regs[ADS1262_REG_INTERFACE] == 0x05);
    assert(regs[ADS1262_REG_MODE0] == 0x00 && regs[ADS1262_REG_MODE1] == 0x00);
    assert(regs[ADS1262_REG_MODE2] == 0x3F);           /* Gain 8, 38400 SPS */
    assert(regs[ADS1262_REG_INPMUX] == 0x3A);          /* AIN3 - AINCOM, restored after calibration */
    assert(cal_mux == 0xFF && calibrations == 1);
    assert(last_cmds[0] == ADS1262_CMD_SFOCAL1 && last_cmds[1] == ADS1262_CMD_START1);
    assert(running);

    /* Inputs: any pair, internal ones on both sides, never other pos == neg or out of range */
    assert(ads1262_set_input(&adc, ADS1262_AIN0, ADS1262_AIN1) == ADS1262_OK);
    assert(regs[ADS1262_REG_INPMUX] == 0x01);
    assert(ads1262_set_input(&adc, ADS1262_AVDD_MON, ADS1262_AVDD_MON) == ADS1262_OK);
    assert(regs[ADS1262_REG_INPMUX] == 0xCC);
    assert(ads1262_set_input(&adc, ADS1262_AIN2, ADS1262_AIN2) == ADS1262_ERROR_PARAMETER);
    assert(ads1262_set_input(&adc, ADS1262_FLOAT, ADS1262_FLOAT) == ADS1262_ERROR_PARAMETER);
    assert(ads1262_set_input(&adc, 16, ADS1262_AIN0) == ADS1262_ERROR_PARAMETER);
    assert(regs[ADS1262_REG_INPMUX] == 0xCC);

    /* Setters self-calibrate, unless chop is on */
    assert(ads1262_set_gain(&adc, ADS1262_GAIN_32) == ADS1262_OK);
    assert(regs[ADS1262_REG_MODE2] == 0x5F && calibrations == 2);
    assert(ads1262_set_gain(&adc, 64) == ADS1262_ERROR_PARAMETER);
    assert(ads1262_set_gain(&adc, ADS1262_GAIN_1) == ADS1262_OK);
    assert(ads1262_set_rate(&adc, ADS1262_RATE_19200) == ADS1262_OK);
    assert(regs[ADS1262_REG_MODE2] == 0x0E && calibrations == 4);
    assert(ads1262_set_filter(&adc, ADS1262_FILTER_FIR) == ADS1262_ERROR_PARAMETER);  /* Not at 19200 SPS */
    assert(ads1262_set_filter(&adc, ADS1262_FILTER_SINC4) == ADS1262_OK);
    assert(regs[ADS1262_REG_MODE1] == 0x60 && calibrations == 5);
    assert(ads1262_set_reference(&adc, ADS1262_REF_AIN0_AIN1, 2.048) == ADS1262_OK);
    assert(regs[ADS1262_REG_REFMUX] == 0x09 && adc.cfg.v_ref == 2.048 && calibrations == 6);
    assert(ads1262_set_reference(&adc, 0x2D, 2.5) == ADS1262_ERROR_PARAMETER);
    assert(ads1262_set_reference(&adc, ADS1262_REF_INTERNAL, 6.0) == ADS1262_ERROR_PARAMETER);
    assert(ads1262_set_reference(&adc, ADS1262_REF_INTERNAL, 2.5) == ADS1262_OK);
    assert(ads1262_set_chop(&adc, true) == ADS1262_OK);
    assert(regs[ADS1262_REG_MODE0] == 0x10 && calibrations == 7);
    assert(ads1262_set_gain(&adc, ADS1262_GAIN_1) == ADS1262_OK && calibrations == 7);
    assert(ads1262_set_chop(&adc, false) == ADS1262_OK);
    assert(regs[ADS1262_REG_MODE0] == 0x00 && calibrations == 8);
    assert(regs[ADS1262_REG_INPMUX] == 0xCC);          /* Restored after each calibration */

    /* System calibration keeps the selected input */
    assert(ads1262_calibrate(&adc, ADS1262_CMD_SYOCAL1) == ADS1262_OK && cal_mux == 0xCC);
    assert(ads1262_calibrate(&adc, ADS1262_CMD_STOP1) == ADS1262_ERROR_PARAMETER);

    /* IDACs: one nibble each */
    assert(ads1262_set_idac(&adc, 1, ADS1262_AIN5, ADS1262_IDAC_500UA) == ADS1262_OK);
    assert(regs[ADS1262_REG_IDACMUX] == 0xB5 && regs[ADS1262_REG_IDACMAG] == 0x04);
    assert(ads1262_set_idac(&adc, 2, ADS1262_AINCOM, ADS1262_IDAC_3000UA) == ADS1262_OK);
    assert(regs[ADS1262_REG_IDACMUX] == 0xA5 && regs[ADS1262_REG_IDACMAG] == 0xA4);
    assert(ads1262_set_idac(&adc, 3, ADS1262_AIN0, ADS1262_IDAC_50UA) == ADS1262_ERROR_PARAMETER);
    assert(ads1262_set_idac(&adc, 1, 12, ADS1262_IDAC_50UA) == ADS1262_ERROR_PARAMETER);
    assert(ads1262_set_idac(&adc, 1, ADS1262_AIN0, 11) == ADS1262_ERROR_PARAMETER);
    assert(ads1262_set_idac(&adc, 1, ADS1262_IDAC_NC, ADS1262_IDAC_OFF) == ADS1262_OK);
    assert(regs[ADS1262_REG_IDACMUX] == 0xAB && regs[ADS1262_REG_IDACMAG] == 0xA0);

    /* Read restarts conversion, so the value always belongs to the current input */
    assert(ads1262_read(&adc, &raw) == ADS1262_OK && raw == code_for(0xCC));
    assert(last_cmds[0] == ADS1262_CMD_START1 && last_cmds[1] == ADS1262_CMD_STOP1);
    check_acquisition(&adc);
    uint8_t bad_inputs[2][2] = { { ADS1262_AIN0, ADS1262_AIN1 }, { ADS1262_AIN1, ADS1262_AIN1 } };
    int32_t values[2];
    assert(ads1262_scan(&adc, bad_inputs, 2, values) == ADS1262_ERROR_PARAMETER);

    /* Corrupted data are reported */
    bad_checksum = 1;
    assert(ads1262_read(&adc, &raw) == ADS1262_ERROR_CHECKSUM);
    bad_checksum = 0;

    /* No new data: timeout */
    commits_left = 0;
    assert(ads1262_read(&adc, &raw) == ADS1262_ERROR_TIMEOUT);
    commits_left = -1;

    /* Volts: full scale is +-Vref / gain at +-2^31; temperature 122.4 mV = 25 C */
    assert(fabs(ads1262_to_volts(&adc, 0x40000000) - 1.25) < 1e-9);
    assert(fabs(ads1262_to_volts(&adc, INT32_MIN) + 2.5) < 1e-9);
    assert(fabs(ads1262_to_celsius(&adc, (int32_t)(0.1224 / 2.5 * 2147483648.0)) - 25.0) < 1e-4);
    assert(fabs(ads1262_to_celsius(&adc, (int32_t)(0.1266 / 2.5 * 2147483648.0)) - 35.0) < 1e-4);
    assert(ads1262_sps(ADS1262_RATE_2_5) == 2.5f && ads1262_sps(16) == 0.0f);
    assert(strcmp(ads1262_strerror(ADS1262_ERROR_CHECKSUM), "Checksum error in conversion data") == 0);
    assert(strcmp(ads1262_strerror(-9), "Unknown error") == 0);

    ads1262_close(&adc);
    assert(adc.spi_fd == -1 && adc.drdy_fd == -1);

    /* ===== DRDY on GPIO ===== */

    cfg.drdy_chip = "/dev/null";                       /* Line request goes to fake ioctl */
    cfg.drdy_line = 22;
    cfg.rate = ADS1262_RATE_100;                       /* 10 ms period */
    cfg.timeout_ms = 1;                                /* Calibration must not use it alone */
    cfg.spi_speed_hz = spi_speed = 4000000;
    assert(ads1262_open(&adc, &cfg) == ADS1262_OK);
    assert(adc.drdy_fd == gpio_rd);
    check_acquisition(&adc);

    /* Signal while waiting for DRDY is not an error */
    int32_t stream[5];
    size_t count;
    poll_eintr = 1;
    assert(ads1262_read_stream(&adc, stream, 5, &count) == ADS1262_OK && count == 5);
    assert(!poll_eintr);

    /* Late-delivered events of old conversions are ignored */
    stale_edge = 1;
    assert(ads1262_read_stream(&adc, stream, 5, &count) == ADS1262_OK && count == 5);
    stale_edge = 0;

    /* Skipped conversion is reported */
    edges_per_conversion = 2;
    assert(ads1262_read_stream(&adc, stream, 5, &count) == ADS1262_ERROR_OVERRUN && count == 0);
    edges_per_conversion = 1;

    /* Read after the next conversion (data not new) is reported */
    late_read = 1;
    assert(ads1262_read_stream(&adc, stream, 5, &count) == ADS1262_ERROR_OVERRUN && count == 0);
    late_read = 0;

    /* DRDY stalls: timeout */
    commits_left = 0;
    assert(ads1262_read(&adc, &raw) == ADS1262_ERROR_TIMEOUT);
    commits_left = 2;
    assert(ads1262_read_stream(&adc, stream, 5, &count) == ADS1262_ERROR_TIMEOUT && count == 2);

    /* Noisy DRDY line: edges keep coming but DRDY stays high, the deadline still holds */
    noise = 1;
    commits_left = 0;
    assert(ads1262_read(&adc, &raw) == ADS1262_ERROR_TIMEOUT);
    assert(ads1262_read_stream(&adc, stream, 5, &count) == ADS1262_ERROR_TIMEOUT && count == 0);
    noise = 0;

    /* GPIO line gone (POLLHUP): error instead of spinning */
    close(gpio_wr);
    assert(ads1262_read_stream(&adc, stream, 5, &count) == ADS1262_ERROR_COMMUNICATION && count == 0);
    commits_left = -1;

    ads1262_close(&adc);
    assert(adc.spi_fd == -1 && adc.drdy_fd == -1);
    ads1262_close(NULL);

    puts("All tests passed");
    return 0;
}
