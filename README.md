# ADS1263 Library for Linux SBCs

C library for the TI ADS1263 32-bit ADC with its 24-bit auxiliary ADC2 over Linux `spidev` (Orange Pi, Raspberry Pi and other single-board computers). Built the same way as the [ADS1256 library](../ADS1256): same handle, error codes and style.

## Features

- Any input combination: AIN0-AIN9 differential, single-ended against AINCOM, or any other pair
- Internal temperature sensor and analog/digital supply monitors
- Data rates 2.5 SPS to 38.4 kSPS, sinc1-sinc4 or FIR filter, PGA gain 1 to 32, input chop
- PGA bypass for inputs down to ground (single-ended against a grounded AINCOM)
- Internal 2.5 V reference or external reference on AIN0/1, AIN2/3, AIN4/5 or the analog supply
- Two IDAC excitation current sources (50 uA to 3 mA) for RTDs
- Optional DRDY pin on GPIO; without it the library polls the new-data bit of the status byte
- Every conversion read is checked by the checksum byte
- Raw signed 32-bit codes, conversion to volts and degrees Celsius on request
- Device handle owned by the caller: no global state, any number of devices
- Gain, data rate, filter and reference changes run offset self-calibration automatically
- Auxiliary 24-bit ADC2 running independently: own inputs, reference, gain 1 to 128, 10 to 800 SPS
- Timing per datasheet (TI SBAS661C, fCLK = 7.3728 MHz internal oscillator)

ADS1263 only: the ADS1262 (the same chip without ADC2) is rejected at open.

## Hardware Connection

```
ADS1263      Orange Pi 5 / Raspberry Pi header
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

Pull SCLK down and CS up (about 10 kΩ) so the ADC sees an idle bus while the board boots and the SPI pins float: a glitch on SCLK shifts the command bits.

### DRDY pin (optional)

Without DRDY, the library reads data with RDATA1 and checks the ADC1 bit of the status byte that comes with it, which tells whether the data are new. After a calibration it waits the datasheet time (+10 %), because no command may be sent before it finishes. Streaming works this way too, but skipped conversions are not detected.

With DRDY on a GPIO, the library waits for its falling edge through the kernel GPIO character device (no extra library needed), knows exactly when calibration ends and reports skipped conversions while streaming.

To use it, set `drdy_chip` and `drdy_line` in the configuration. Find the chip and line of your header pin with `sudo gpioinfo`. GPIO chips are root-only by default. To allow your user, add a udev rule, e.g. `/etc/udev/rules.d/99-gpio.rules`:

```
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
```

Then run `sudo groupadd -f gpio && sudo usermod -a -G gpio $USER`, reload udev rules (or reboot) and log in again.

## Platform Notes

The library uses only kernel services, so timing works the same on all boards: `CLOCK_MONOTONIC` and `clock_nanosleep` for delays and timeouts, kernel timestamps of DRDY edges from the same clock. Unlike the ADS1256, the ADS1263 needs no delays inside SPI messages. The ADC clock comes from its internal oscillator (or a crystal on the module), not from the board.

### Raspberry Pi 4

- **Kernel 5.10 or newer** (Raspberry Pi OS Bullseye or Bookworm). The GPIO uAPI v2 header is needed at compile time even without the DRDY pin, so Buster (4.19) does not work.
- **Use SPI0** (`/dev/spidev0.0`), enabled by `dtparam=spi=on` in `/boot/config.txt` (`/boot/firmware/config.txt` on Bookworm). The auxiliary SPI1 (`/dev/spidev1.x`) does not work in SPI mode 1, which the ADS1263 needs.
- **DRDY pin**: `/dev/gpiochip0`, line = BCM GPIO number. For example DRDY on GPIO17 (header pin 11): `./ads1263 /dev/gpiochip0 17`.
- **Permissions**: `/dev/gpiochip*` belongs to group `gpio` and `/dev/spidev*` to group `spi`, and the default user is in both, so no udev rule is needed.
- **SCLK** is the 500 MHz core clock divided by an even number and rounded down: 4 MHz becomes about 3.91 MHz.

### Orange Pi 5

- Enable SPI with a device tree overlay (`orangepi-config` or the `overlays=` line in `/boot/orangepiEnv.txt`).
- **DRDY pin**: Rockchip pin `GPIOx_yz` is `/dev/gpiochipx`, line `y * 8 + z` with A=0, B=1, C=2, D=3 (e.g. GPIO1_C6 is `/dev/gpiochip1`, line 22). Check with `sudo gpioinfo`.
- `/dev/gpiochip*` is root-only by default, see the udev rule above.
- **Measured streaming limits** (Orange Pi OS, kernel 6.1 with `PREEMPT_VOLUNTARY`, `/dev/spidev4.1` at SCLK 4 MHz, DRDY on GPIO1_A3, sinc1 filter, normal priority). `read_stream()` for 2 minutes per rate (2400 SPS and up: 30 s). With DRDY the stream was restarted after each overrun. Without DRDY lost conversions were counted from the stream duration (after the restart, n samples take the first-conversion latency plus n - 1 periods, each lost one adds a period), with the period measured from DRDY edges while the bus was idle (internal oscillator +0.06 %). DRDY edges can't be counted during the stream itself: the first SCLK edge drives DRDY high (datasheet 9.4.5), so a conversion that finishes during a polling read gives a pulse too short for the GPIO.

  | SPS | With DRDY: overruns (reported) | Without DRDY: lost conversions (not reported) |
  |---|---|---|
  | 7200 | 6436 in 30 s, longest clean run 856 samples | 21 % (5670 SPS delivered) |
  | 4800 | 4720 in 30 s, longest clean run 860 samples | 33 % (3210 SPS delivered) |
  | 2400 | 664 in 30 s, longest clean run 1685 samples | 14 % (2070 SPS delivered) |
  | 1200 | 280 in 2 min, longest clean run 3804 samples | 2.2 % (1175 SPS delivered) |
  | 400 | 58 in 2 min, longest clean run 7249 samples | 221 of 48221 |
  | 100 | 16 in 2 min | 26 of 12026 |
  | 60 | 0 | 0 of 7200 |
  | 50 | 0 | 1 of 6001 |
  | 20 | 0 | 0 of 2400 |

  - The limit is the kernel, not the ADC: the results match the [ADS1256 on the same board](../ADS1256/README.md#orange-pi-5), where the process was delayed by 10 to 13 ms now and then (a 7 ms sleep ended after 20 ms) and waking up from `poll()` occasionally took over 1 ms. Above 2400 SPS the spidev overhead adds to it (one transfer takes tens of us).
  - Not repeated with the ADS1263: with the ADS1256, pinning to a big core (`taskset`) didn't help; real-time priority (`chrt -f 50`) with busy-waiting ran 1000-2000 SPS clean, above that the kernel itself delays by 100-300 us.
  - `read()` and `scan()` restart the conversion and are not affected; a delay only makes them slower.

## Requirements

No libraries: only the Linux `spidev` driver and the GPIO character device (uAPI v2, kernel 5.10 or newer, used only with the DRDY pin).

## Build

```bash
make                # Example program ./ads1263
make lib            # Static library libads1263.a
make test           # Hardware-free test with an emulated ADS1263
make so             # Shared library libads1263.so for Python (ads1263.py)
make test-py        # Hardware-free check of the Python binding
make install        # libads1263.a to ~/lib, ads1263_lib.h to ~/include
```

Link your program with `-lads1263`, or just compile `ads1263_lib.c` with it.

## Usage

### Single reading

```c
#include <stdio.h>
#include "ads1263_lib.h"

int main(void)
{
    ads1263_config_t cfg = {
        .spi_device = "/dev/spidev0.0",
        .spi_speed_hz = 4000000,              /* Max 8 MHz */
        .drdy_chip = NULL,                    /* Or "/dev/gpiochipN" + .drdy_line */
        .v_ref = 2.5,
        .refmux = ADS1263_REF_INTERNAL,
        .drate = ADS1263_DRATE_20,
        .filter = ADS1263_FILTER_FIR,         /* FIR: 2.5, 5, 10, 20 SPS only */
        .gain = ADS1263_GAIN_1,
        .pos = ADS1263_AIN0, .neg = ADS1263_AIN1,
        .chop = false,
        .bypass = false,                      /* true for inputs near ground, see below */
        .timeout_ms = 1000,
    };
    ads1263_t adc;
    int32_t raw;

    int result = ads1263_open(&adc, &cfg);
    if (result != ADS1263_OK) {
        fprintf(stderr, "%s\n", ads1263_strerror(result));
        return 1;
    }
    if (ads1263_read(&adc, &raw) == ADS1263_OK)
        printf("AIN0-AIN1: %.9f V\n", ads1263_to_volts(&adc, raw));

    ads1263_close(&adc);
    return 0;
}
```

`ads1263_read()` restarts the conversion and waits for the first data, which are fully settled with every filter (datasheet table 9-13), so the result always matches the current input and settings. The first conversion takes longer than one period with sinc2-sinc4: e.g. 7.9 ms at 400 SPS sinc3.

### Single-ended inputs, internal sensors and scanning

```c
ads1263_set_input(&adc, ADS1263_AIN3, ADS1263_AINCOM);   /* AIN3 against AINCOM */

uint8_t inputs[3][2] = {
    { ADS1263_AIN0, ADS1263_AINCOM },
    { ADS1263_AIN8, ADS1263_AIN9 },
    { ADS1263_TEMP, ADS1263_TEMP },                      /* Internal: same on both sides */
};
int32_t values[3];
ads1263_scan(&adc, inputs, 3, values);
double celsius = ads1263_to_celsius(&adc, values[2]);
```

**Inputs near ground need the PGA bypassed.** With the PGA on, both inputs must stay between AVSS + 0.3 V and AVDD - 0.3 V (datasheet equation 12, narrower with gain): AIN3 against a grounded AINCOM (e.g. the Waveshare High-Precision AD HAT, AINCOM on GND) is outside, reads wrong and sets `ADS1263_STATUS_PGAL_ALM`. Set `.bypass = true` (or `ads1263_set_bypass()`): the range becomes AVSS - 0.1 V to AVDD + 0.1 V, only gain 1, input impedance 40 MΩ (input current about 150 nA instead of 2 nA) and no PGA alarms. Inputs biased mid-supply or a split supply (AVSS = -2.5 V) can keep the PGA and its gain. ADC2 bypasses its PGA by itself at gain 1-4.

The scan selects each input and reads one fresh conversion. Writing the input multiplexer clears the previous result in the ADC, so the ADS1256 trick of selecting the next input while reading the previous one is not possible; for fast scanning use sinc1 or a low order filter. The last pair stays selected afterwards.

The temperature sensor and supply monitors need gain 1 and chop off. `ADS1263_AVDD_MON` reads (AVDD - AVSS) / 4 and `ADS1263_DVDD_MON` reads DVDD / 4.

### RTD with IDAC

```c
ads1263_set_idac(&adc, 1, ADS1263_AIN0, ADS1263_IDAC_500UA);   /* IDAC1: 500 uA out of AIN0 */
ads1263_set_idac(&adc, 1, ADS1263_IDAC_NC, ADS1263_IDAC_OFF);  /* Off */
```

### Streaming

```c
int32_t samples[1000];
size_t count;                                /* Valid samples, also on error */
ads1263_read_stream(&adc, samples, 1000, &count);
```

With DRDY wired this reads each conversion right after DRDY and returns `ADS1263_ERROR_OVERRUN` when the program can't keep up: a conversion finished unread (judged by kernel timestamps of the DRDY edges) or was read only after the next one came (the status byte says the data are not new). Linux is not a real-time system: an occasional wake-up delay longer than one conversion period is enough for an overrun. Check the return value, keep the `count` valid samples and restart the stream if needed. Without the pin, skipped conversions are not detected.

Recommendations from the [platform notes](#orange-pi-5): without DRDY, stream at 30 SPS or less. With DRDY, expect occasional overruns above 30 SPS; they are always reported. If every conversion matters, use DRDY and a data rate that ran clean on your board for the whole measurement time.

### ADC2

The ADS1263 has a second, 24-bit ADC that runs independently of ADC1: its own input multiplexer (same inputs, including the temperature sensor and supply monitors), reference, gain 1-128 and data rate 10, 100, 400 or 800 SPS with a fixed sinc3 filter. Use it to cross-check the main measurement, or for cold-junction temperature and supply monitoring while ADC1 stays on the sensor.

```c
ads1263_adc2_config_t cfg2 = {
    .v_ref = 2.5,
    .refmux = ADS1263_REF_INTERNAL,           /* Paired references only, no mixed REFMUX values */
    .drate = ADS1263_ADC2_DRATE_100,
    .gain = ADS1263_GAIN_1,
    .pos = ADS1263_TEMP, .neg = ADS1263_TEMP,
};
ads1263_adc2_start(&adc, &cfg2);              /* Configure, start, self-calibrate */
ads1263_adc2_read(&adc, &raw);                /* Signed 24-bit code */
printf("%.2f C\n", ads1263_adc2_to_celsius(&adc, raw));
```

Until `ads1263_adc2_start()`, ADC2 is stopped with its reset settings (10 SPS, gain 1, internal reference, AIN0/AIN1). ADC2 has no DRDY pin: the library always polls its new-data bit through RDATA2 and waits out calibration by the datasheet time (table 9-29: 28 ms at 800 SPS, 1.7 s at 10 SPS). `ads1263_adc2_read_stream()` therefore doesn't detect skipped conversions. Codes are `raw * v_ref / gain / 2^23`.

### Python

`ads1263.py` wraps `libads1263.so` (`make so`, kept next to it) with ctypes, no other packages needed. Functions lose the `ads1263_` prefix, constants the `ADS1263_` one, and errors raise `ads1263.ADS1263Error` (`.code`; a failed `read_stream()` keeps its valid samples in `.samples`). Keyword arguments of the constructor are the `ads1263_config_t` fields, `adc.dev` is the C handle (`status`, `cfg`, ...).

```python
import ads1263 as A

with A.ADS1263("/dev/spidev4.1", drdy_chip="/dev/gpiochip1", drdy_line=3,
               pos=A.AIN0, neg=A.AINCOM, bypass=True) as adc:
    print(adc.to_volts(adc.read()), "V")
    raw = adc.read_stream(100)                       # List of 100 codes
    print(adc.scan([(A.AIN1, A.AINCOM), (A.TEMP, A.TEMP)]))
    adc.adc2_start(pos=A.TEMP, neg=A.TEMP)
    print(adc.adc2_to_celsius(adc.adc2_read()), "C")
```

The ctypes structures mirror `ads1263_lib.h`; after changing a structure there, update `ads1263.py` and run `make test-py`.

## API

| Function | Description |
|---|---|
| `ads1263_open(dev, cfg)` | Open SPI (+ GPIO), reset, check the chip, configure, start, self-calibrate |
| `ads1263_close(dev)` | Close file descriptors (call before reopening the handle) |
| `ads1263_set_input(dev, pos, neg)` | Inputs `ADS1263_AIN0..9`, `AINCOM`, internal `TEMP`, `AVDD_MON`, `DVDD_MON`, `TDAC` |
| `ads1263_set_gain(dev, gain)` | PGA gain 1-32, self-calibrates |
| `ads1263_set_drate(dev, rate)` | Data rate, self-calibrates |
| `ads1263_set_filter(dev, filter)` | Sinc1-sinc4 or FIR, self-calibrates |
| `ads1263_set_reference(dev, refmux, v_ref)` | Reference `ADS1263_REF_*` and its voltage, self-calibrates |
| `ads1263_set_chop(dev, on)` | Input chop; off self-calibrates (offset calibration is unused with chop) |
| `ads1263_set_bypass(dev, on)` | PGA bypass for inputs near ground (gain 1 only), self-calibrates |
| `ads1263_set_idac(dev, idac, pin, current)` | IDAC 1 or 2 to a pin with a current |
| `ads1263_calibrate(dev, cmd)` | SFOCAL1 (self offset), SYOCAL1 (system offset) or SYGCAL1 (system gain) |
| `ads1263_read(dev, &raw)` | One fresh conversion |
| `ads1263_read_stream(dev, raw, n, &count)` | n consecutive conversions, count of valid ones |
| `ads1263_scan(dev, inputs, n, raw)` | One conversion of each input pair, last pair stays selected |
| `ads1263_to_volts(dev, raw)` | Code to volts: `raw * v_ref / gain / 2^31` |
| `ads1263_to_celsius(dev, raw)` | Temperature sensor code to degrees Celsius |
| `ads1263_sps(rate)` | Data rate in SPS |
| `ads1263_load_vref(path, &v_ref)` | Measured internal reference from a file (default `~/.config/ads1263/vref`) |
| `ads1263_read_register` / `ads1263_write_register` | Low-level register access; bypasses `dev->cfg`, so use the setters for settings kept there |
| `ads1263_adc2_start(dev, cfg2)` | Configure ADC2, start it, self-calibrate |
| `ads1263_adc2_set_input` / `_set_gain` / `_set_drate` / `_set_reference` | ADC2 settings; gain, rate and reference self-calibrate |
| `ads1263_adc2_calibrate(dev, cmd)` | SFOCAL2, SYOCAL2 or SYGCAL2 |
| `ads1263_adc2_read` / `_read_stream` / `_scan` | Same as for ADC1, signed 24-bit codes |
| `ads1263_adc2_to_volts` / `_to_celsius` / `ads1263_adc2_sps` | Conversions for ADC2 |
| `ads1263_strerror(code)` | Error text |

`dev->cfg` and `dev->cfg2` always hold the current settings of ADC1 and ADC2, `dev->status2` the status byte of the last ADC2 read and `dev->status` that of the last ADC1 read: PGA and reference alarms (`ADS1263_STATUS_*_ALM`) and `ADS1263_STATUS_RESET`, which is set if the chip was reset (e.g. by a supply dip) after `ads1263_open()`. All functions return `ADS1263_OK` (0) or a negative error code: `ADS1263_ERROR_PARAMETER`, `ADS1263_ERROR_COMMUNICATION`, `ADS1263_ERROR_TIMEOUT`, `ADS1263_ERROR_OVERRUN` (`read_stream()` with DRDY pin skipped a conversion) or `ADS1263_ERROR_CHECKSUM` (data corrupted on the bus, read again).

Self-calibration takes 17 to 23 conversion periods: 0.85 s at 20 SPS FIR, up to 9.2 s at 2.5 SPS sinc4 (datasheet table 9-28). Choose the data rate and filter in the configuration rather than with setters right after open to calibrate only once.

## Switching from the ADS1256 library

The API follows the [ADS1256 library](../ADS1256) v4.9: same handle and configuration, function names, error codes and behaviour of `read()`, `read_stream()` and `scan()`. Most code ports by renaming the prefix:

```bash
sed -i 's/ads1256/ads1263/g; s/ADS1256/ADS1263/g' your_program.c
```

Then adjust what differs between the chips:

| ADS1256 | ADS1263 |
|---|---|
| `.buffer`, `set_buffer()` | `.bypass`, `set_bypass()` with the opposite sense: ADS1256 buffer off and ADS1263 bypass on allow inputs down to ground; bypass needs gain 1. New fields `.refmux`, `.filter`, `.chop` are fine at zero: internal reference, sinc1, chop off |
| `v_ref` 0.5-2.6 V, full scale ±2 · v_ref / gain | `v_ref` 0.9-5.25 V (2.5 for the internal reference), full scale ±v_ref / gain; `to_volts()` handles it |
| Data rates 2.5-30000 SPS | 2.5-38400 SPS; only `ADS1263_DRATE_2_5`, `_5`, `_10`, `_50`, `_60`, `_100` exist in both, pick the nearest for others |
| Gain 1-64 | ADC1 gain 1-32 (ADC2 up to 128) |
| AIN0-AIN7, AINCOM | AIN0-AIN9, AINCOM, plus internal `TEMP`, `AVDD_MON`, `DVDD_MON`, `TDAC` |
| Raw codes 24-bit | 32-bit (ADC2: 24-bit); keep `int32_t`, use `to_volts()` |
| `ADS1256_CMD_SELFCAL`, `SELFOCAL` | `ADS1263_CMD_SFOCAL1`: offset only, the gain is factory-trimmed (see [Accuracy](#accuracy)) |
| `ADS1256_CMD_SYSOCAL`, `SYSGCAL` | `ADS1263_CMD_SYOCAL1`, `SYGCAL1` |
| `ADS1256_ERROR_*` | The same, plus `ADS1263_ERROR_CHECKSUM` (data corrupted on the bus) |
| Self-calibration up to 1.2 s (2.5 SPS) | Up to 9.2 s (2.5 SPS sinc4), 0.85 s at 20 SPS FIR |

Not available: `set_buffer()`, `read_ts()` and the calibration files (`get/set/apply/load_calibration()`, `ads1256_cal`); the ADS1263 needs no external calibration for most uses, only the measured reference (`ads1263_load_vref()`, see [Accuracy](#accuracy)).

## Accuracy

No external calibration is needed for most uses. With the internal reference, the uncalibrated error at 25 °C is typically 0.1 % (at most about 0.25 %): reference initial accuracy ±0.1 % typ, ±0.2 % max, plus ADC1 gain error ±50 ppm typ, ±300 ppm max (datasheet table 7.5). The offset is removed by the self-calibration the library runs at open and after each setting change (to about noise / 4), or by chop. With the PGA on, input current is about 2 nA, so input filter resistors on the module cost almost nothing (2 kΩ in series: about 4 µV); with the PGA bypassed it is about 150 nA (0.3 mV over 2 kΩ).

To remove the dominant reference error, measure the REFOUT pin (pin 8, buffered internal reference against AVSS) once with a good meter and save the value; every program then gets it with `ads1263_load_vref()`:

```bash
mkdir -p ~/.config/ads1263
printf '# REFOUT, measured 2026-10-08\n2.4987\n' > ~/.config/ads1263/vref
```

```c
double v_ref;
if (ads1263_load_vref(NULL, &v_ref) == ADS1263_OK)   /* NULL: $XDG_CONFIG_HOME or ~/.config */
    cfg.v_ref = cfg2.v_ref = v_ref;                   /* Before ads1263_open() / ads1263_adc2_start() */
else if (errno != ENOENT)
    fprintf(stderr, "Reference value not loaded: %s\n", strerror(errno));
```

The file holds exactly one number with a decimal point (whatever the locale) between 2.49 and 2.51 V, plus `#` comment lines; anything else is refused (`errno` EINVAL), no file gives ENOENT and the nominal 2.5 V stays. For several ADS1263 pass each its own path. The value belongs to one chip and the library can't tell chips apart: after replacing the board or module, measure REFOUT again (or delete the file). The gain error of about 50 ppm remains; the reference drifts 2-6 ppm/°C, up to 50 ppm in the first 1000 hours and 50 ppm thermal hysteresis, which limits any calibration to about 0.01 % unless repeated. Ratiometric measurements (RTD with IDAC and reference resistor, bridge with AVDD as reference) don't depend on the reference value at all.

ADC2 is less accurate: gain error ±500 ppm typ, ±3000 ppm max.

Errors outside the chip (dividers, shunts, sensor offset) are not covered; calibrate those in your program if needed.

## Troubleshooting

- **Permission denied on /dev/spidev0.0**: `sudo usermod -a -G spi $USER`, log in again.
- **Permission denied on /dev/gpiochipN**: see the udev rule above.
- **Communication error at open**: the chip didn't answer with the ADS1263 ID (an ADS1262 is rejected too) or the registers didn't read back. Check wiring, power, DVDD level and SPI mode 1; try a lower `spi_speed_hz`.
- **Checksum errors**: noise or ringing on SCLK/DOUT; shorter wires, series resistor at SCLK, lower `spi_speed_hz`.
- **Timeouts**: check the START pin (low) and RESET/PWDN (high). With DRDY, check the chip and line number. `timeout_ms` is added to the first conversion time, so it doesn't need to grow with slow data rates.
- **Wrong readings near 0 V or with `ADS1263_STATUS_PGAL_ALM` / `PGAH_ALM` in `dev->status`**: the inputs are outside the PGA range, bypass it (see [Single-ended inputs](#single-ended-inputs-internal-sensors-and-scanning)).
- **Wrong readings**: check `v_ref` and `refmux` against your reference and the gain against the signal range. After power-on the internal reference needs time to settle (datasheet figure 7-33); call `ads1263_calibrate(&adc, ADS1263_CMD_SFOCAL1)` again once it has.

## Version History

### Version 2.3 (2026-10-09)
- Fix: reading right after a conversion restart failed with a checksum error (the cleared holding register reads 00h, checksum too), so ADC2 and ADC1 without DRDY pin didn't work
- Test emulator clears the holding register at restart like the chip
- README: streaming limits measured with the ADS1263 (were taken over from the ADS1256)

### Version 2.2 (2026-10-08)
- Python binding `ads1263.py` (ctypes, no other packages) over the shared library from `make so`
- `make test-py`: hardware-free check that the ctypes structures match `ads1263_lib.h`

### Version 2.1 (2026-10-08)
- PGA bypass (`.bypass`, `ads1263_set_bypass()`): inputs down to ground, e.g. single-ended against a grounded AINCOM (Waveshare High-Precision AD HAT); with the PGA on they read wrong
- Example bypasses the PGA for its single-ended scan

### Version 2.0 (2026-10-08)
- Renamed to ADS1263 (`ads1263_*`), ADS1262 no longer accepted
- ADC2 support: `ads1263_adc2_*()`, registers 15h-1Ah, `ADS1263_GAIN_64` and `_128` for ADC2
- Data rate named `drate` as in the ADS1256 library (`.drate`, `ads1263_drate_t`, `ADS1263_DRATE_*`, `ads1263_set_drate()`, ADC2 likewise); README: switching from the ADS1256 library
- `ads1263_load_vref()`: measured value of the internal reference from `~/.config/ads1263/vref`, shared by all programs; the example uses it
- README: accuracy without external calibration, streaming limits measured with the ADS1256 on the same board; the example keeps valid samples after a stream overrun

### Version 1.0 (2026-10-06)
- First version, API modelled on the ADS1256 library v4.0

## License

MIT, see the LICENSE file.
