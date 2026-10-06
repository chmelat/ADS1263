# ADS1262 Library for Linux SBCs

C library for the TI ADS1262 32-bit ADC over Linux `spidev` (Orange Pi, Raspberry Pi and other single-board computers). Built the same way as the [ADS1256 library](../ADS1256): same handle, error codes and style.

## Features

- Any input combination: AIN0-AIN9 differential, single-ended against AINCOM, or any other pair
- Internal temperature sensor and analog/digital supply monitors
- Data rates 2.5 SPS to 38.4 kSPS, sinc1-sinc4 or FIR filter, PGA gain 1 to 32, input chop
- Internal 2.5 V reference or external reference on AIN0/1, AIN2/3, AIN4/5 or the analog supply
- Two IDAC excitation current sources (50 uA to 3 mA) for RTDs
- Optional DRDY pin on GPIO; without it the library polls the new-data bit of the status byte
- Every conversion read is checked by the checksum byte
- Raw signed 32-bit codes, conversion to volts and degrees Celsius on request
- Device handle owned by the caller: no global state, any number of devices
- Gain, data rate, filter and reference changes run offset self-calibration automatically
- Timing per datasheet (TI SBAS661C, fCLK = 7.3728 MHz internal oscillator)

The ADS1263 works too, but only its main ADC1; the auxiliary ADC2 is not supported.

## Hardware Connection

```
ADS1262      Orange Pi 5 / Raspberry Pi header
----------------------------------------------
CS           CE0   (pin 24)
DOUT/DRDY    MISO  (pin 21)
DIN          MOSI  (pin 19)
SCLK         SCLK  (pin 23)
DGND, AVSS   GND   (any GND pin)
AVDD         5V    (pin 2)
DVDD         3.3V  (pin 1)   digital levels follow DVDD
START        GND             conversions are started by commands
RESET/PWDN   DVDD            low = power-down
DRDY         any GPIO, optional
```

If START is high, conversions run freely and fight with the START1/STOP1 commands; the library sends STOP1 at open, but keep START low.

### DRDY pin (optional)

Without DRDY, the library reads data with RDATA1 and checks the ADC1 bit of the status byte that comes with it, which tells whether the data are new. After a calibration it waits the datasheet time (+10 %), because no command may be sent before it finishes. Streaming works this way too, but skipped conversions are not detected.

With DRDY on a GPIO, the library waits for its falling edge through the kernel GPIO character device (no extra library needed), knows exactly when calibration ends and reports skipped conversions while streaming.

To use it, set `drdy_chip` and `drdy_line` in the configuration. Find the chip and line of your header pin with `sudo gpioinfo`. GPIO chips are root-only by default. To allow your user, add a udev rule, e.g. `/etc/udev/rules.d/99-gpio.rules`:

```
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
```

Then run `sudo groupadd -f gpio && sudo usermod -a -G gpio $USER`, reload udev rules (or reboot) and log in again.

## Platform Notes

The library uses only kernel services, so timing works the same on all boards: `CLOCK_MONOTONIC` and `clock_nanosleep` for delays and timeouts, kernel timestamps of DRDY edges from the same clock. Unlike the ADS1256, the ADS1262 needs no delays inside SPI messages. The ADC clock comes from its internal oscillator (or a crystal on the module), not from the board.

### Raspberry Pi 4

- **Kernel 5.10 or newer** (Raspberry Pi OS Bullseye or Bookworm). The GPIO uAPI v2 header is needed at compile time even without the DRDY pin, so Buster (4.19) does not work.
- **Use SPI0** (`/dev/spidev0.0`), enabled by `dtparam=spi=on` in `/boot/config.txt` (`/boot/firmware/config.txt` on Bookworm). The auxiliary SPI1 (`/dev/spidev1.x`) does not work in SPI mode 1, which the ADS1262 needs.
- **DRDY pin**: `/dev/gpiochip0`, line = BCM GPIO number. For example DRDY on GPIO17 (header pin 11): `./ads1262 /dev/gpiochip0 17`.
- **Permissions**: `/dev/gpiochip*` belongs to group `gpio` and `/dev/spidev*` to group `spi`, and the default user is in both, so no udev rule is needed.
- **SCLK** is the 500 MHz core clock divided by an even number and rounded down: 4 MHz becomes about 3.91 MHz.

### Orange Pi 5

- Enable SPI with a device tree overlay (`orangepi-config` or the `overlays=` line in `/boot/orangepiEnv.txt`).
- **DRDY pin**: Rockchip pin `GPIOx_yz` is `/dev/gpiochipx`, line `y * 8 + z` with A=0, B=1, C=2, D=3 (e.g. GPIO1_C6 is `/dev/gpiochip1`, line 22). Check with `sudo gpioinfo`.
- `/dev/gpiochip*` is root-only by default, see the udev rule above.
- **Streaming limits**: not measured yet. With the ADS1256 on the same board, wake-up from `poll()` occasionally took over 1 ms, so expect overruns above roughly 500-1000 SPS on a stock kernel. `read()` and `scan()` restart the conversion and are not affected.

## Requirements

No libraries: only the Linux `spidev` driver and the GPIO character device (uAPI v2, kernel 5.10 or newer, used only with the DRDY pin).

## Build

```bash
make                # Example program ./ads1262
make lib            # Static library libads1262.a
make test           # Hardware-free test with an emulated ADS1262
make install        # libads1262.a to ~/lib, ads1262_lib.h to ~/include
```

Link your program with `-lads1262`, or just compile `ads1262_lib.c` with it.

## Usage

### Single reading

```c
#include <stdio.h>
#include "ads1262_lib.h"

int main(void)
{
    ads1262_config_t cfg = {
        .spi_device = "/dev/spidev0.0",
        .spi_speed_hz = 4000000,              /* Max 8 MHz */
        .drdy_chip = NULL,                    /* Or "/dev/gpiochipN" + .drdy_line */
        .v_ref = 2.5,
        .refmux = ADS1262_REF_INTERNAL,
        .rate = ADS1262_RATE_20,
        .filter = ADS1262_FILTER_FIR,         /* FIR: 2.5, 5, 10, 20 SPS only */
        .gain = ADS1262_GAIN_1,
        .pos = ADS1262_AIN0, .neg = ADS1262_AIN1,
        .chop = false,
        .timeout_ms = 1000,
    };
    ads1262_t adc;
    int32_t raw;

    int result = ads1262_open(&adc, &cfg);
    if (result != ADS1262_OK) {
        fprintf(stderr, "%s\n", ads1262_strerror(result));
        return 1;
    }
    if (ads1262_read(&adc, &raw) == ADS1262_OK)
        printf("AIN0-AIN1: %.9f V\n", ads1262_to_volts(&adc, raw));

    ads1262_close(&adc);
    return 0;
}
```

`ads1262_read()` restarts the conversion and waits for the first data, which are fully settled with every filter (datasheet table 9-13), so the result always matches the current input and settings. The first conversion takes longer than one period with sinc2-sinc4: e.g. 7.9 ms at 400 SPS sinc3.

### Single-ended inputs, internal sensors and scanning

```c
ads1262_set_input(&adc, ADS1262_AIN3, ADS1262_AINCOM);   /* AIN3 against AINCOM */

uint8_t inputs[3][2] = {
    { ADS1262_AIN0, ADS1262_AINCOM },
    { ADS1262_AIN8, ADS1262_AIN9 },
    { ADS1262_TEMP, ADS1262_TEMP },                      /* Internal: same on both sides */
};
int32_t values[3];
ads1262_scan(&adc, inputs, 3, values);
double celsius = ads1262_to_celsius(&adc, values[2]);
```

The scan selects each input and reads one fresh conversion. Writing the input multiplexer clears the previous result in the ADC, so the ADS1256 trick of selecting the next input while reading the previous one is not possible; for fast scanning use sinc1 or a low order filter. The last pair stays selected afterwards.

The temperature sensor and supply monitors need gain 1 and chop off. `ADS1262_AVDD_MON` reads (AVDD - AVSS) / 4 and `ADS1262_DVDD_MON` reads DVDD / 4.

### RTD with IDAC

```c
ads1262_set_idac(&adc, 1, ADS1262_AIN0, ADS1262_IDAC_500UA);   /* IDAC1: 500 uA out of AIN0 */
ads1262_set_idac(&adc, 1, ADS1262_IDAC_NC, ADS1262_IDAC_OFF);  /* Off */
```

### Streaming

```c
int32_t samples[1000];
size_t count;                                /* Valid samples, also on error */
ads1262_read_stream(&adc, samples, 1000, &count);
```

With DRDY wired this reads each conversion right after DRDY and returns `ADS1262_ERROR_OVERRUN` when the program can't keep up: a conversion finished unread (judged by kernel timestamps of the DRDY edges) or was read only after the next one came (the status byte says the data are not new). Linux is not a real-time system: an occasional wake-up delay longer than one conversion period is enough for an overrun. Check the return value and restart the stream if needed. Without the pin, skipped conversions are not detected, so measure the real rate on your hardware.

## API

| Function | Description |
|---|---|
| `ads1262_open(dev, cfg)` | Open SPI (+ GPIO), reset, check the chip, configure, start, self-calibrate |
| `ads1262_close(dev)` | Close file descriptors (call before reopening the handle) |
| `ads1262_set_input(dev, pos, neg)` | Inputs `ADS1262_AIN0..9`, `AINCOM`, internal `TEMP`, `AVDD_MON`, `DVDD_MON`, `TDAC` |
| `ads1262_set_gain(dev, gain)` | PGA gain 1-32, self-calibrates |
| `ads1262_set_rate(dev, rate)` | Data rate, self-calibrates |
| `ads1262_set_filter(dev, filter)` | Sinc1-sinc4 or FIR, self-calibrates |
| `ads1262_set_reference(dev, refmux, v_ref)` | Reference `ADS1262_REF_*` and its voltage, self-calibrates |
| `ads1262_set_chop(dev, on)` | Input chop; off self-calibrates (offset calibration is unused with chop) |
| `ads1262_set_idac(dev, idac, pin, current)` | IDAC 1 or 2 to a pin with a current |
| `ads1262_calibrate(dev, cmd)` | SFOCAL1 (self offset), SYOCAL1 (system offset) or SYGCAL1 (system gain) |
| `ads1262_read(dev, &raw)` | One fresh conversion |
| `ads1262_read_stream(dev, raw, n, &count)` | n consecutive conversions, count of valid ones |
| `ads1262_scan(dev, inputs, n, raw)` | One conversion of each input pair, last pair stays selected |
| `ads1262_to_volts(dev, raw)` | Code to volts: `raw * v_ref / gain / 2^31` |
| `ads1262_to_celsius(dev, raw)` | Temperature sensor code to degrees Celsius |
| `ads1262_sps(rate)` | Data rate in SPS |
| `ads1262_read_register` / `ads1262_write_register` | Low-level register access; bypasses `dev->cfg`, so use the setters for settings kept there |
| `ads1262_strerror(code)` | Error text |

`dev->cfg` always holds the current settings and `dev->status` the status byte of the last conversion read: PGA and reference alarms (`ADS1262_STATUS_*_ALM`) and `ADS1262_STATUS_RESET`, which is set if the chip was reset (e.g. by a supply dip) after `ads1262_open()`. All functions return `ADS1262_OK` (0) or a negative error code: `ADS1262_ERROR_PARAMETER`, `ADS1262_ERROR_COMMUNICATION`, `ADS1262_ERROR_TIMEOUT`, `ADS1262_ERROR_OVERRUN` (`read_stream()` with DRDY pin skipped a conversion) or `ADS1262_ERROR_CHECKSUM` (data corrupted on the bus, read again).

Self-calibration takes 17 to 23 conversion periods: 0.85 s at 20 SPS FIR, up to 9.2 s at 2.5 SPS sinc4 (datasheet table 9-28). Choose the data rate and filter in the configuration rather than with setters right after open to calibrate only once.

## Troubleshooting

- **Permission denied on /dev/spidev0.0**: `sudo usermod -a -G spi $USER`, log in again.
- **Permission denied on /dev/gpiochipN**: see the udev rule above.
- **Communication error at open**: the chip didn't answer with an ADS1262/ADS1263 ID or the registers didn't read back. Check wiring, power, DVDD level and SPI mode 1; try a lower `spi_speed_hz`.
- **Checksum errors**: noise or ringing on SCLK/DOUT; shorter wires, series resistor at SCLK, lower `spi_speed_hz`.
- **Timeouts**: check the START pin (low) and RESET/PWDN (high). With DRDY, check the chip and line number. `timeout_ms` is added to the first conversion time, so it doesn't need to grow with slow data rates.
- **Wrong readings**: check `v_ref` and `refmux` against your reference and the gain against the signal range. After power-on the internal reference needs time to settle (datasheet figure 7-33); call `ads1262_calibrate(&adc, ADS1262_CMD_SFOCAL1)` again once it has.

## Version History

### Version 1.0 (2026-10-06)
- First version, API modelled on the ADS1256 library v4.0

## License

MIT, see the LICENSE file.
