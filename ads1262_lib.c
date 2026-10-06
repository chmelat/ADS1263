/**
 * @file ads1262_lib.c
 * @brief Library for ADS1262 32-bit ADC on Linux spidev
 * @version 1.0
 * @date 2026-10-06
 *
 * Datasheet: TI SBAS661C. Timing tables assume fCLK = 7.3728 MHz (internal oscillator).
 * Uses only Linux spidev and GPIO character device (uAPI v2, kernel >= 5.10).
 */

#define _POSIX_C_SOURCE 200809L   /* clock_nanosleep, O_CLOEXEC also under strict -std=c11 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>

#include "ads1262_lib.h"

#define RESET_DELAY_US   10       /* RESET -> next command: 8 / fCLK = 1.1 us */
#define MAX_SPI_SPEED_HZ 8000000  /* SCLK period >= 125 ns */
#define CHECKSUM_OFFSET  0x9B     /* Checksum = sum of 4 data bytes + 9Bh */
#define POWER_VALUE      0x01     /* RESET flag cleared, internal reference on */
#define INTERFACE_VALUE  0x05     /* Status byte and checksum byte on (default) */
#define MUX_OPEN         0xFF     /* Both inputs open, for self offset calibration */

/* Data rates [SPS] */
static const float SPS[16] = {
    2.5f, 5, 10, 16.6f, 20, 50, 60, 100, 400, 1200, 2400, 4800, 7200, 14400, 19200, 38400
};

/* First conversion latency [us] per [rate][filter] (sinc1-4, FIR), datasheet table 9-13.
 * 14400 SPS and up use sinc5 whatever the filter setting; 0 = FIR not available */
static const uint32_t LATENCY_US[16][5] = {
    { 400400, 800400, 1200000, 1600000, 402200 },
    { 200400, 400400,  600400,  800400, 202200 },
    { 100400, 200400,  300400,  400400, 102200 },
    {  60350, 120400,  180400,  240400,      0 },
    {  50350, 100400,  150400,  200400,  52220 },
    {  20350,  40420,   60420,   80420,      0 },
    {  17020,  33760,   50420,   67090,      0 },
    {  10350,  20420,   30420,   40420,      0 },
    {   2855,   5424,    7924,   10420,      0 },
    {   1188,   2091,    2924,    3758,      0 },
    {    771,   1258,    1674,    2091,      0 },
    {    563,    841,    1049,    1258,      0 },
    {    494,    702,     841,     980,      0 },
    {    424,    424,     424,     424,      0 },
    {    337,    337,     337,     337,      0 },
    {    207,    207,     207,     207,      0 },
};

/* Calibration time [us] per [rate][filter], datasheet table 9-28 */
static const uint32_t CALIBRATION_US[16][5] = {
    { 6801000, 7601000, 8401000, 9201000, 6805000 },
    { 3401000, 3801000, 4201000, 4601000, 3405000 },
    { 1701000, 1901000, 2101000, 2300000, 1705000 },
    { 1021000, 1141000, 1261000, 1381000,       0 },
    {  850700,  951000, 1051000, 1151000,  854500 },
    {  340900,  380900,  421000,  460900,       0 },
    {  284100,  317700,  350900,  384400,       0 },
    {  170800,  190900,  210900,  230800,       0 },
    {   43270,   48430,   53420,   58410,       0 },
    {   14930,   16720,   18400,   20070,       0 },
    {    7845,    8816,    9643,   10480,       0 },
    {    4302,    4858,    5276,    5692,       0 },
    {    3123,    3534,    3815,    4095,       0 },
    {    1941,    1941,    1941,    1941,       0 },
    {    1490,    1490,    1490,    1490,       0 },
    {     812,     812,     812,     812,       0 },
};


/* ===== Time ===== */

static uint64_t now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000u + (uint64_t)t.tv_nsec;
}

/** Sleep the whole time even when signals arrive (usleep may return early or reject >= 1 s) */
static void sleep_us(uint32_t us)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t ns = (uint64_t)t.tv_nsec + (uint64_t)us * 1000u;
    t.tv_sec += (time_t)(ns / 1000000000u);
    t.tv_nsec = (long)(ns % 1000000000u);
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL) == EINTR) {
    }
}

/** Time from (re)start to the first settled data; chop doubles it (equations 19, 20) */
static uint32_t first_us(const ads1262_t *dev)
{
    return LATENCY_US[dev->cfg.rate][dev->cfg.filter] * (dev->cfg.chop ? 2 : 1);
}

/** Time between conversions; with chop it is the latency (equation 21) */
static uint32_t period_us(const ads1262_t *dev)
{
    return dev->cfg.chop ? LATENCY_US[dev->cfg.rate][dev->cfg.filter]
                         : (uint32_t)(1e6f / SPS[dev->cfg.rate]);
}

/** Timeout for one conversion: timeout_ms on top of the first-conversion latency */
static uint32_t data_timeout_ms(const ads1262_t *dev)
{
    return dev->cfg.timeout_ms + first_us(dev) / 1000u + 1;
}


/* ===== SPI helpers ===== */

/** Open spidev in SPI mode 1 with given clock, returns fd or -1 */
static int open_spi(const char *device, uint32_t speed_hz)
{
    uint8_t mode = SPI_MODE_1;
    int fd = open(device, O_RDWR | O_CLOEXEC);

    if (fd >= 0 && (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
                    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0)) {
        close(fd);
        return -1;
    }
    return fd;
}

/** One full-duplex SPI message with CS low throughout; rx may be NULL */
static int transfer(ads1262_t *dev, const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    struct spi_ioc_transfer xfer = { .tx_buf = (unsigned long)tx, .rx_buf = (unsigned long)rx, .len = len };
    return ioctl(dev->spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0 ? ADS1262_ERROR_COMMUNICATION : ADS1262_OK;
}

static int send_command(ads1262_t *dev, uint8_t cmd)
{
    return transfer(dev, &cmd, NULL, 1);
}

/** Read n consecutive registers from start */
static int read_registers(ads1262_t *dev, uint8_t start, uint8_t *values, uint8_t n)
{
    uint8_t tx[2 + ADS1262_REG_GPIODAT + 1] = { ADS1262_CMD_RREG | start, (uint8_t)(n - 1) };
    uint8_t rx[sizeof(tx)];
    int result = transfer(dev, tx, rx, 2u + n);
    if (result == ADS1262_OK) {
        memcpy(values, rx + 2, n);
    }
    return result;
}

/** Write n consecutive registers from start; MODE0-INPMUX in one block take effect together */
static int write_registers(ads1262_t *dev, uint8_t start, const uint8_t *values, uint8_t n)
{
    uint8_t tx[2 + ADS1262_REG_GPIODAT + 1] = { ADS1262_CMD_WREG | start, (uint8_t)(n - 1) };
    memcpy(tx + 2, values, n);
    return transfer(dev, tx, NULL, 2u + n);
}

static int update_register_bits(ads1262_t *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t old;
    int result = ads1262_read_register(dev, reg, &old);
    if (result != ADS1262_OK) {
        return result;
    }
    return ads1262_write_register(dev, reg, (uint8_t)((old & ~mask) | (value & mask)));
}

/**
 * RDATA1: status byte, 32-bit code MSB first, checksum. Data come from the holding
 * register, so the read needs no synchronization to DRDY. fresh (optional) tells
 * whether the data are new since the last read.
 */
static int read_data(ads1262_t *dev, int32_t *raw, bool *fresh)
{
    const uint8_t tx[7] = { ADS1262_CMD_RDATA1 };
    uint8_t d[7];
    int result = transfer(dev, tx, d, sizeof(d));
    if (result != ADS1262_OK) {
        return result;
    }
    if ((uint8_t)(d[2] + d[3] + d[4] + d[5] + CHECKSUM_OFFSET) != d[6]) {
        return ADS1262_ERROR_CHECKSUM;  /* Also MISO stuck at 0 or 1 */
    }
    dev->status = d[1];
    *raw = (int32_t)((uint32_t)d[2] << 24 | (uint32_t)d[3] << 16 | (uint32_t)d[4] << 8 | d[5]);
    if (fresh) {
        *fresh = d[1] & ADS1262_STATUS_ADC1;
    }
    return ADS1262_OK;
}

/**
 * Restart conversion, the next data ready then delivers settled data with current settings.
 * ponytail: without DRDY pin the new-data flag must not come from before the restart, so
 * the data are read once right after it (no conversion can finish within the latency).
 * If the program is preempted for longer than that, a fresh result is dropped and the
 * read waits one period more.
 */
static int start_conversion(ads1262_t *dev)
{
    int32_t raw;
    int result;
    if ((result = send_command(dev, ADS1262_CMD_STOP1)) != ADS1262_OK ||
        (result = send_command(dev, ADS1262_CMD_START1)) != ADS1262_OK ||
        dev->drdy_fd >= 0) {
        return result;
    }
    return read_data(dev, &raw, NULL);
}


/* ===== Waiting for DRDY ===== */

/** Open DRDY GPIO line as input with falling edge events, returns line fd or -1 */
static int open_drdy(const char *chip, unsigned int line)
{
    int chip_fd = open(chip, O_RDONLY | O_CLOEXEC);
    if (chip_fd < 0) {
        return -1;
    }

    struct gpio_v2_line_request req = {
        .offsets = { line },
        .consumer = "ads1262-drdy",
        .config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_FALLING,
        .num_lines = 1,
    };
    int result = ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req);
    close(chip_fd);  /* Line fd stays valid */
    if (result < 0) {
        return -1;
    }

    if (fcntl(req.fd, F_SETFL, O_NONBLOCK) < 0) {  /* For draining queued events */
        close(req.fd);
        return -1;
    }
    return req.fd;
}

/**
 * Read all queued edge events without blocking. Counts only events not older than
 * `after` (older ones were delivered late for conversions already handled; equal
 * timestamps can occur within the clock resolution) and
 * stores the newest kernel timestamp (CLOCK_MONOTONIC) in *newest.
 * Returns the count, or -1 on read error.
 */
static int drain_edges(int fd, uint64_t after, uint64_t *newest)
{
    struct gpio_v2_line_event events[16];
    int count = 0;
    ssize_t len;

    while ((len = read(fd, events, sizeof(events))) > 0) {
        for (size_t i = 0; i < (size_t)len / sizeof(events[0]); i++) {
            if (events[i].timestamp_ns >= after) {
                count++;
                if (newest) {
                    *newest = events[i].timestamp_ns;
                }
            }
        }
    }
    return len < 0 && errno != EAGAIN ? -1 : count;
}

static int drdy_level(ads1262_t *dev, int *level)
{
    struct gpio_v2_line_values values = { .mask = 1 };
    if (ioctl(dev->drdy_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &values) < 0) {
        return ADS1262_ERROR_COMMUNICATION;
    }
    *level = (int)(values.bits & 1);
    return ADS1262_OK;
}

/** Wait until an edge event is queued or the deadline (now_ns time) passes, retried after signals */
static int poll_edge(ads1262_t *dev, uint64_t deadline)
{
    struct pollfd pfd = { .fd = dev->drdy_fd, .events = POLLIN };
    int result;

    do {
        uint64_t now = now_ns();
        int ms = now >= deadline ? 0 : (int)((deadline - now + 999999) / 1000000);
        result = poll(&pfd, 1, ms);
    } while (result < 0 && errno == EINTR);
    if (result == 0) {
        return ADS1262_ERROR_TIMEOUT;
    }
    if (result < 0 || !(pfd.revents & POLLIN)) {
        return ADS1262_ERROR_COMMUNICATION;  /* POLLERR/POLLHUP: line is gone */
    }
    return ADS1262_OK;
}

/**
 * Wait for DRDY low on the GPIO pin; the level is checked, so stale edges don't matter
 * (a restart or calibration command drives DRDY high first).
 * The deadline is fixed, so a noisy line (edges, but no low level) still times out.
 */
static int wait_drdy_pin(ads1262_t *dev, uint32_t timeout_ms)
{
    uint64_t deadline = now_ns() + (uint64_t)timeout_ms * 1000000u;

    for (;;) {
        int level, result;
        if (drain_edges(dev->drdy_fd, 0, NULL) < 0) {
            return ADS1262_ERROR_COMMUNICATION;
        }
        if ((result = drdy_level(dev, &level)) != ADS1262_OK || !level) {
            return result;  /* DRDY is active low */
        }
        if (now_ns() >= deadline) {
            return ADS1262_ERROR_TIMEOUT;  /* Edges keep coming, DRDY doesn't stay low */
        }
        if ((result = poll_edge(dev, deadline)) != ADS1262_OK) {
            return result;
        }
    }
}

/** Wait for falling edges at or after `after` (streaming); edges gets their count, *newest the last time */
static int wait_edges(ads1262_t *dev, uint64_t after, int *edges, uint64_t *newest)
{
    uint64_t deadline = now_ns() + (uint64_t)data_timeout_ms(dev) * 1000000u;

    do {
        int result = poll_edge(dev, deadline);
        if (result != ADS1262_OK) {
            return result;
        }
        if ((*edges = drain_edges(dev->drdy_fd, after, newest)) < 0) {
            return ADS1262_ERROR_COMMUNICATION;
        }
        if (*edges == 0 && now_ns() >= deadline) {
            return ADS1262_ERROR_TIMEOUT;  /* Only old or noise edges came */
        }
    } while (*edges == 0);
    return ADS1262_OK;
}

/**
 * Wait for new data and read them. With the pin waits for DRDY low. Without it sleeps
 * 70 % of `due_us` (expected time to the data), then polls RDATA1 until the status
 * byte says the data are new.
 */
static int wait_and_read(ads1262_t *dev, uint32_t due_us, int32_t *raw)
{
    int result;
    if (dev->drdy_fd >= 0) {
        result = wait_drdy_pin(dev, data_timeout_ms(dev));
        return result != ADS1262_OK ? result : read_data(dev, raw, NULL);
    }

    uint64_t deadline = now_ns() + (uint64_t)data_timeout_ms(dev) * 1000000u;
    if (due_us * 7 / 10 > 100) {
        sleep_us(due_us * 7 / 10);
    }
    uint32_t poll_us = due_us < 1000 ? 20 : due_us < 10000 ? 100 : 500;

    for (;;) {
        bool fresh;
        if ((result = read_data(dev, raw, &fresh)) != ADS1262_OK || fresh) {
            return result;
        }
        if (now_ns() > deadline) {
            return ADS1262_ERROR_TIMEOUT;
        }
        sleep_us(poll_us);
    }
}

/**
 * Wait for the end of calibration taking `us` per datasheet.
 * ponytail: during calibration CS and SCLK must stay low, so without DRDY pin sleep the
 * datasheet time + 10 % margin (the table doesn't say typical or maximum; with chop the
 * time is not given, assumed double). Upgrade: wire DRDY.
 */
static int wait_ready(ads1262_t *dev, uint32_t us)
{
    us = (us + us / 10) * (dev->cfg.chop ? 2 : 1);
    if (dev->drdy_fd >= 0) {
        return wait_drdy_pin(dev, us / 1000 + data_timeout_ms(dev));  /* Up to 20 s at 2.5 SPS */
    }
    sleep_us(us);
    return ADS1262_OK;
}


/* ===== Configuration ===== */

/** Internal inputs (temperature, supply monitors, test DAC) are selected on both sides */
static bool valid_input(uint8_t pos, uint8_t neg)
{
    return pos <= ADS1262_FLOAT && neg <= ADS1262_FLOAT &&
           (pos != neg || (pos >= ADS1262_TEMP && pos <= ADS1262_TDAC));
}

/** FIR exists only at 2.5, 5, 10 and 20 SPS */
static bool valid_filter(ads1262_rate_t rate, ads1262_filter_t filter)
{
    return (unsigned)rate <= ADS1262_RATE_38400 && (unsigned)filter <= ADS1262_FILTER_FIR &&
           LATENCY_US[rate][filter] != 0;
}

static bool valid_reference(uint8_t refmux, double v_ref)
{
    return (refmux >> 3) <= 4 && (refmux & 7) <= 4 &&
           v_ref >= 0.9 && v_ref <= 5.25;  /* Also rejects NaN */
}

/** PGA register bits = log2(gain), -1 for invalid gain */
static int pga_bits(int gain)
{
    for (int bits = 0; bits <= 5; bits++) {
        if (gain == 1 << bits) {
            return bits;
        }
    }
    return -1;
}

/** Offset self-calibration after a setting changed; the offset register is unused with chop */
static int self_calibrate(ads1262_t *dev)
{
    return dev->cfg.chop ? ADS1262_OK : ads1262_calibrate(dev, ADS1262_CMD_SFOCAL1);
}

int ads1262_open(ads1262_t *dev, const ads1262_config_t *cfg)
{
    if (!dev) {
        return ADS1262_ERROR_PARAMETER;
    }
    dev->spi_fd = dev->drdy_fd = -1;  /* ads1262_close() is safe after any failure */

    if (!cfg || !cfg->spi_device ||
        cfg->spi_speed_hz == 0 || cfg->spi_speed_hz > MAX_SPI_SPEED_HZ ||
        !valid_reference(cfg->refmux, cfg->v_ref) ||
        !valid_filter(cfg->rate, cfg->filter) || pga_bits(cfg->gain) < 0 ||
        !valid_input(cfg->pos, cfg->neg) ||
        cfg->timeout_ms == 0 || cfg->timeout_ms > 3600000) {
        return ADS1262_ERROR_PARAMETER;
    }

    dev->cfg = *cfg;
    dev->status = 0;
    dev->spi_fd = open_spi(cfg->spi_device, cfg->spi_speed_hz);
    if (dev->spi_fd < 0) {
        return ADS1262_ERROR_COMMUNICATION;
    }
    if (cfg->drdy_chip && (dev->drdy_fd = open_drdy(cfg->drdy_chip, cfg->drdy_line)) < 0) {
        ads1262_close(dev);
        return ADS1262_ERROR_COMMUNICATION;
    }

    int result = send_command(dev, ADS1262_CMD_RESET);
    sleep_us(RESET_DELAY_US);

    /* POWER .. INPMUX in one block (MODE0-INPMUX update as a group), then read back:
     * INPMUX is never 00h or FFh here, so a missing chip (MISO high or low) can't match */
    const uint8_t block[6] = {
        POWER_VALUE, INTERFACE_VALUE,
        cfg->chop ? 0x10 : 0x00,                                 /* MODE0: CHOP[1:0] = 01 */
        (uint8_t)(cfg->filter << 5),                             /* MODE1 */
        (uint8_t)(pga_bits(cfg->gain) << 4 | cfg->rate),         /* MODE2 */
        (uint8_t)(cfg->pos << 4 | cfg->neg),                     /* INPMUX */
    };
    uint8_t id, readback[sizeof(block)];
    if (result != ADS1262_OK ||
        (result = send_command(dev, ADS1262_CMD_STOP1)) != ADS1262_OK ||  /* In case START pin is high */
        (result = ads1262_read_register(dev, ADS1262_REG_ID, &id)) != ADS1262_OK ||
        (result = id >> 5 <= 1 ? ADS1262_OK : ADS1262_ERROR_COMMUNICATION) != ADS1262_OK ||  /* ADS1262/3 */
        (result = write_registers(dev, ADS1262_REG_POWER, block, sizeof(block))) != ADS1262_OK ||
        (result = read_registers(dev, ADS1262_REG_POWER, readback, sizeof(readback))) != ADS1262_OK ||
        (result = memcmp(block, readback, sizeof(block)) ? ADS1262_ERROR_COMMUNICATION : ADS1262_OK) != ADS1262_OK ||
        (result = ads1262_write_register(dev, ADS1262_REG_REFMUX, cfg->refmux)) != ADS1262_OK ||
        (result = send_command(dev, ADS1262_CMD_START1)) != ADS1262_OK ||
        (result = self_calibrate(dev)) != ADS1262_OK) {
        ads1262_close(dev);
        return result;
    }

    return ADS1262_OK;
}

void ads1262_close(ads1262_t *dev)
{
    if (!dev) {
        return;
    }
    if (dev->drdy_fd >= 0) {
        close(dev->drdy_fd);
    }
    if (dev->spi_fd >= 0) {
        close(dev->spi_fd);
    }
    dev->spi_fd = dev->drdy_fd = -1;
}

int ads1262_set_input(ads1262_t *dev, uint8_t pos, uint8_t neg)
{
    if (!valid_input(pos, neg)) {
        return ADS1262_ERROR_PARAMETER;
    }

    /* INPMUX = MUXP (bits 7-4) | MUXN (bits 3-0) */
    int result = ads1262_write_register(dev, ADS1262_REG_INPMUX, (uint8_t)(pos << 4 | neg));
    if (result == ADS1262_OK) {
        dev->cfg.pos = pos;
        dev->cfg.neg = neg;
    }
    return result;
}

int ads1262_set_gain(ads1262_t *dev, ads1262_gain_t gain)
{
    int pga = pga_bits(gain);
    if (pga < 0) {
        return ADS1262_ERROR_PARAMETER;
    }

    int result = update_register_bits(dev, ADS1262_REG_MODE2, 0x70, (uint8_t)(pga << 4));
    if (result != ADS1262_OK) {
        return result;
    }
    dev->cfg.gain = gain;
    return self_calibrate(dev);
}

int ads1262_set_rate(ads1262_t *dev, ads1262_rate_t rate)
{
    if (!valid_filter(rate, dev->cfg.filter)) {
        return ADS1262_ERROR_PARAMETER;
    }

    int result = update_register_bits(dev, ADS1262_REG_MODE2, 0x0F, (uint8_t)rate);
    if (result != ADS1262_OK) {
        return result;
    }
    dev->cfg.rate = rate;  /* Calibration time depends on the new rate */
    return self_calibrate(dev);
}

int ads1262_set_filter(ads1262_t *dev, ads1262_filter_t filter)
{
    if (!valid_filter(dev->cfg.rate, filter)) {
        return ADS1262_ERROR_PARAMETER;
    }

    int result = update_register_bits(dev, ADS1262_REG_MODE1, 0xE0, (uint8_t)(filter << 5));
    if (result != ADS1262_OK) {
        return result;
    }
    dev->cfg.filter = filter;
    return self_calibrate(dev);
}

int ads1262_set_reference(ads1262_t *dev, uint8_t refmux, double v_ref)
{
    if (!valid_reference(refmux, v_ref)) {
        return ADS1262_ERROR_PARAMETER;
    }

    int result = ads1262_write_register(dev, ADS1262_REG_REFMUX, refmux);
    if (result != ADS1262_OK) {
        return result;
    }
    dev->cfg.refmux = refmux;
    dev->cfg.v_ref = v_ref;
    return self_calibrate(dev);
}

int ads1262_set_chop(ads1262_t *dev, bool on)
{
    int result = update_register_bits(dev, ADS1262_REG_MODE0, 0x10, on ? 0x10 : 0x00);
    if (result != ADS1262_OK) {
        return result;
    }
    dev->cfg.chop = on;
    return self_calibrate(dev);
}

int ads1262_set_idac(ads1262_t *dev, int idac, uint8_t pin, ads1262_idac_t current)
{
    if ((idac != 1 && idac != 2) || pin > ADS1262_IDAC_NC || (unsigned)current > ADS1262_IDAC_3000UA) {
        return ADS1262_ERROR_PARAMETER;
    }

    /* IDAC1 in bits 3-0, IDAC2 in bits 7-4 of both registers */
    int shift = idac == 1 ? 0 : 4;
    int result = update_register_bits(dev, ADS1262_REG_IDACMUX, (uint8_t)(0x0F << shift), (uint8_t)(pin << shift));
    if (result != ADS1262_OK) {
        return result;
    }
    return update_register_bits(dev, ADS1262_REG_IDACMAG, (uint8_t)(0x0F << shift), (uint8_t)(current << shift));
}

int ads1262_calibrate(ads1262_t *dev, uint8_t cmd)
{
    if (cmd != ADS1262_CMD_SFOCAL1 && cmd != ADS1262_CMD_SYOCAL1 && cmd != ADS1262_CMD_SYGCAL1) {
        return ADS1262_ERROR_PARAMETER;
    }

    /* Datasheet 9.4.9.8: conversions running, inputs open for self calibration */
    int result;
    if ((result = send_command(dev, ADS1262_CMD_START1)) != ADS1262_OK ||
        (cmd == ADS1262_CMD_SFOCAL1 &&
         (result = ads1262_write_register(dev, ADS1262_REG_INPMUX, MUX_OPEN)) != ADS1262_OK)) {
        return result;
    }

    if ((result = send_command(dev, cmd)) == ADS1262_OK) {
        result = wait_ready(dev, CALIBRATION_US[dev->cfg.rate][dev->cfg.filter]);
    }

    if (cmd == ADS1262_CMD_SFOCAL1) {
        int restore = ads1262_write_register(dev, ADS1262_REG_INPMUX,
                                             (uint8_t)(dev->cfg.pos << 4 | dev->cfg.neg));
        if (result == ADS1262_OK) {
            result = restore;
        }
    }
    return result;
}


/* ===== Data acquisition ===== */

int ads1262_read(ads1262_t *dev, int32_t *raw)
{
    if (!raw) {
        return ADS1262_ERROR_PARAMETER;
    }

    int result = start_conversion(dev);
    return result != ADS1262_OK ? result : wait_and_read(dev, first_us(dev), raw);
}

/**
 * Stream with DRDY on GPIO, started at time `edge` (after the restart).
 * Each sample needs exactly one new falling edge, judged by kernel timestamps, so
 * late-delivered events of earlier conversions don't count. More edges mean a
 * conversion finished unread. Data not new mean the read came after the next
 * conversion (its edge is then counted for the following sample): one was lost.
 */
static int read_stream_pin(ads1262_t *dev, int32_t *raw, size_t n, size_t *done, uint64_t edge)
{
    int edges, result;
    bool fresh = false;

    do {
        if ((result = wait_edges(dev, edge, &edges, &edge)) != ADS1262_OK ||
            (result = edges > 1 ? ADS1262_ERROR_OVERRUN : read_data(dev, &raw[*done], &fresh)) != ADS1262_OK) {
            break;
        }
        if (!fresh) {
            result = ADS1262_ERROR_OVERRUN;
            break;
        }
        (*done)++;
    } while (*done < n);
    return result;
}

int ads1262_read_stream(ads1262_t *dev, int32_t *raw, size_t n, size_t *count)
{
    size_t done = 0;

    if (count) {
        *count = 0;
    }
    if (!raw) {
        return ADS1262_ERROR_PARAMETER;
    }
    if (n == 0) {
        return ADS1262_OK;
    }

    int result = start_conversion(dev);
    uint64_t started = now_ns();  /* Edges before this belong to old conversions */

    if (result == ADS1262_OK && dev->drdy_fd >= 0) {
        result = read_stream_pin(dev, raw, n, &done, started);
    } else {
        /* ponytail: without the pin each sample is polled by RDATA1 and a skipped
         * conversion looks like a slower rate. Upgrade: wire DRDY. */
        uint32_t due = first_us(dev);
        while (result == ADS1262_OK && done < n &&
               (result = wait_and_read(dev, due, &raw[done])) == ADS1262_OK) {
            done++;
            due = period_us(dev);
        }
    }

    if (count) {
        *count = done;
    }
    return result;
}

/* Writing INPMUX clears the conversion holding register, so the next input can't be
 * selected while the previous result waits (unlike ADS1256 input cycling) */
int ads1262_scan(ads1262_t *dev, uint8_t inputs[][2], size_t n, int32_t *raw)
{
    if (!inputs || !raw) {
        return ADS1262_ERROR_PARAMETER;
    }
    for (size_t i = 0; i < n; i++) {
        if (!valid_input(inputs[i][0], inputs[i][1])) {
            return ADS1262_ERROR_PARAMETER;
        }
    }

    for (size_t i = 0; i < n; i++) {
        int result;
        if ((result = ads1262_set_input(dev, inputs[i][0], inputs[i][1])) != ADS1262_OK ||
            (result = ads1262_read(dev, &raw[i])) != ADS1262_OK) {
            return result;
        }
    }
    return ADS1262_OK;
}

double ads1262_to_volts(const ads1262_t *dev, int32_t raw)
{
    /* Full scale +-Vref / gain at code +-2^31 */
    return dev->cfg.v_ref / dev->cfg.gain * raw / 2147483648.0;
}

double ads1262_to_celsius(const ads1262_t *dev, int32_t raw)
{
    /* Datasheet equation 9: 122.4 mV at 25 C, 420 uV/C */
    return (ads1262_to_volts(dev, raw) * 1e6 - 122400.0) / 420.0 + 25.0;
}

float ads1262_sps(ads1262_rate_t rate)
{
    return (unsigned)rate <= ADS1262_RATE_38400 ? SPS[rate] : 0.0f;
}


/* ===== Registers ===== */

int ads1262_read_register(ads1262_t *dev, uint8_t reg, uint8_t *value)
{
    if (reg > ADS1262_REG_GPIODAT || !value) {
        return ADS1262_ERROR_PARAMETER;
    }
    return read_registers(dev, reg, value, 1);
}

int ads1262_write_register(ads1262_t *dev, uint8_t reg, uint8_t value)
{
    if (reg > ADS1262_REG_GPIODAT) {
        return ADS1262_ERROR_PARAMETER;
    }
    return write_registers(dev, reg, &value, 1);
}

const char *ads1262_strerror(int error_code)
{
    static const char *messages[] = {
        "Success", "Invalid parameter", "Communication error", "Timeout expired",
        "Conversions skipped (reading too slow)", "Checksum error in conversion data"
    };

    if (error_code > 0 || error_code < ADS1262_ERROR_CHECKSUM) {
        return "Unknown error";
    }
    return messages[-error_code];
}
