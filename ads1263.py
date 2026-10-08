"""
Python binding for the ADS1263 library (ctypes over libads1263.so, build: make so).

    import ads1263
    with ads1263.ADS1263("/dev/spidev0.0", drdy_chip="/dev/gpiochip0", drdy_line=25) as adc:
        print(adc.to_volts(adc.read()), "V")

Methods mirror ads1263_*() without the prefix and raise ADS1263Error instead of
returning error codes; ADC2 ones are adc2_*(). adc.dev is the C handle (status, cfg, ...).
Structures below must match ads1263_lib.h; test_ads1263.py checks them.
"""

import ctypes as C
import os

# Return codes
OK, ERROR_PARAMETER, ERROR_COMMUNICATION, ERROR_TIMEOUT, ERROR_OVERRUN, ERROR_CHECKSUM = 0, -1, -2, -3, -4, -5

# Inputs
(AIN0, AIN1, AIN2, AIN3, AIN4, AIN5, AIN6, AIN7, AIN8, AIN9,
 AINCOM, TEMP, AVDD_MON, DVDD_MON, TDAC, FLOAT) = range(16)
IDAC_NC = 11

# Data rates, filters, gains
(DRATE_2_5, DRATE_5, DRATE_10, DRATE_16_6, DRATE_20, DRATE_50, DRATE_60, DRATE_100,
 DRATE_400, DRATE_1200, DRATE_2400, DRATE_4800, DRATE_7200, DRATE_14400, DRATE_19200, DRATE_38400) = range(16)
FILTER_SINC1, FILTER_SINC2, FILTER_SINC3, FILTER_SINC4, FILTER_FIR = range(5)
ADC2_DRATE_10, ADC2_DRATE_100, ADC2_DRATE_400, ADC2_DRATE_800 = range(4)
GAIN_1, GAIN_2, GAIN_4, GAIN_8, GAIN_16, GAIN_32, GAIN_64, GAIN_128 = 1, 2, 4, 8, 16, 32, 64, 128

# References
REF_INTERNAL, REF_AIN0_AIN1, REF_AIN2_AIN3, REF_AIN4_AIN5, REF_AVDD = 0x00, 0x09, 0x12, 0x1B, 0x24

# IDAC currents
(IDAC_OFF, IDAC_50UA, IDAC_100UA, IDAC_250UA, IDAC_500UA, IDAC_750UA,
 IDAC_1000UA, IDAC_1500UA, IDAC_2000UA, IDAC_2500UA, IDAC_3000UA) = range(11)

# Status byte (adc.dev.status, status2)
STATUS_ADC2, STATUS_ADC1, STATUS_EXTCLK, STATUS_REF_ALM = 0x80, 0x40, 0x20, 0x10
STATUS_PGAL_ALM, STATUS_PGAH_ALM, STATUS_PGAD_ALM, STATUS_RESET = 0x08, 0x04, 0x02, 0x01

# Calibration commands
CMD_SYOCAL1, CMD_SYGCAL1, CMD_SFOCAL1 = 0x16, 0x17, 0x19
CMD_SYOCAL2, CMD_SYGCAL2, CMD_SFOCAL2 = 0x1B, 0x1C, 0x1E

# Registers
(REG_ID, REG_POWER, REG_INTERFACE, REG_MODE0, REG_MODE1, REG_MODE2, REG_INPMUX,
 REG_OFCAL0, REG_OFCAL1, REG_OFCAL2, REG_FSCAL0, REG_FSCAL1, REG_FSCAL2,
 REG_IDACMUX, REG_IDACMAG, REG_REFMUX, REG_TDACP, REG_TDACN,
 REG_GPIOCON, REG_GPIODIR, REG_GPIODAT, REG_ADC2CFG, REG_ADC2MUX,
 REG_ADC2OFC0, REG_ADC2OFC1, REG_ADC2FSC0, REG_ADC2FSC1) = range(27)


class Config(C.Structure):
    """ads1263_config_t (enums are int)"""
    _fields_ = [("spi_device", C.c_char_p), ("spi_speed_hz", C.c_uint32),
                ("drdy_chip", C.c_char_p), ("drdy_line", C.c_uint),
                ("v_ref", C.c_double), ("refmux", C.c_uint8),
                ("drate", C.c_int), ("filter", C.c_int), ("gain", C.c_int),
                ("pos", C.c_uint8), ("neg", C.c_uint8),
                ("chop", C.c_bool), ("bypass", C.c_bool), ("timeout_ms", C.c_uint32)]


class Adc2Config(C.Structure):
    """ads1263_adc2_config_t"""
    _fields_ = [("v_ref", C.c_double), ("refmux", C.c_uint8),
                ("drate", C.c_int), ("gain", C.c_int),
                ("pos", C.c_uint8), ("neg", C.c_uint8)]


class Device(C.Structure):
    """ads1263_t"""
    _fields_ = [("spi_fd", C.c_int), ("drdy_fd", C.c_int), ("cfg", Config), ("status", C.c_uint8),
                ("cfg2", Adc2Config), ("status2", C.c_uint8)]


lib = C.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)), "libads1263.so"), use_errno=True)
_dev = C.POINTER(Device)
_i32p = C.POINTER(C.c_int32)
_pairs = C.POINTER(C.c_uint8 * 2)
for name, restype, argtypes in [
    ("open", C.c_int, [_dev, C.POINTER(Config)]),
    ("close", None, [_dev]),
    ("set_input", C.c_int, [_dev, C.c_uint8, C.c_uint8]),
    ("set_gain", C.c_int, [_dev, C.c_int]),
    ("set_drate", C.c_int, [_dev, C.c_int]),
    ("set_filter", C.c_int, [_dev, C.c_int]),
    ("set_reference", C.c_int, [_dev, C.c_uint8, C.c_double]),
    ("set_chop", C.c_int, [_dev, C.c_bool]),
    ("set_bypass", C.c_int, [_dev, C.c_bool]),
    ("set_idac", C.c_int, [_dev, C.c_int, C.c_uint8, C.c_int]),
    ("calibrate", C.c_int, [_dev, C.c_uint8]),
    ("read", C.c_int, [_dev, _i32p]),
    ("read_stream", C.c_int, [_dev, _i32p, C.c_size_t, C.POINTER(C.c_size_t)]),
    ("scan", C.c_int, [_dev, _pairs, C.c_size_t, _i32p]),
    ("to_volts", C.c_double, [_dev, C.c_int32]),
    ("to_celsius", C.c_double, [_dev, C.c_int32]),
    ("sps", C.c_float, [C.c_int]),
    ("load_vref", C.c_int, [C.c_char_p, C.POINTER(C.c_double)]),
    ("read_register", C.c_int, [_dev, C.c_uint8, C.POINTER(C.c_uint8)]),
    ("write_register", C.c_int, [_dev, C.c_uint8, C.c_uint8]),
    ("adc2_start", C.c_int, [_dev, C.POINTER(Adc2Config)]),
    ("adc2_set_input", C.c_int, [_dev, C.c_uint8, C.c_uint8]),
    ("adc2_set_gain", C.c_int, [_dev, C.c_int]),
    ("adc2_set_drate", C.c_int, [_dev, C.c_int]),
    ("adc2_set_reference", C.c_int, [_dev, C.c_uint8, C.c_double]),
    ("adc2_calibrate", C.c_int, [_dev, C.c_uint8]),
    ("adc2_read", C.c_int, [_dev, _i32p]),
    ("adc2_read_stream", C.c_int, [_dev, _i32p, C.c_size_t, C.POINTER(C.c_size_t)]),
    ("adc2_scan", C.c_int, [_dev, _pairs, C.c_size_t, _i32p]),
    ("adc2_to_volts", C.c_double, [_dev, C.c_int32]),
    ("adc2_to_celsius", C.c_double, [_dev, C.c_int32]),
    ("adc2_sps", C.c_float, [C.c_int]),
    ("strerror", C.c_char_p, [C.c_int]),
]:
    f = getattr(lib, "ads1263_" + name)
    f.restype, f.argtypes = restype, argtypes


def strerror(code):
    return lib.ads1263_strerror(code).decode()


class ADS1263Error(Exception):
    """Library error; .code is ADS1263_ERROR_*, .samples the valid part of a failed read_stream()"""
    def __init__(self, code, samples=None):
        super().__init__(strerror(code))
        self.code, self.samples = code, samples


def _check(result):
    if result != OK:
        raise ADS1263Error(result)


def sps(drate):
    return lib.ads1263_sps(drate)


def adc2_sps(drate):
    return lib.ads1263_adc2_sps(drate)


def load_vref(path=None):
    """Measured internal reference from file (None = ~/.config/ads1263/vref); OSError if missing/bad"""
    v_ref = C.c_double()
    if lib.ads1263_load_vref(path.encode() if path else None, C.byref(v_ref)) != OK:
        err = C.get_errno()
        raise OSError(err, os.strerror(err), path)
    return v_ref.value


class ADS1263:
    """Open device; keyword arguments are ads1263_config_t fields, defaults as in ads1263_example.c"""

    def __init__(self, spi_device, spi_speed_hz=4000000, drdy_chip=None, drdy_line=0,
                 v_ref=2.5, refmux=REF_INTERNAL, drate=DRATE_400, filter=FILTER_SINC3, gain=GAIN_1,
                 pos=AIN0, neg=AIN1, chop=False, bypass=False, timeout_ms=1000):
        self.dev = Device()
        # Config keeps the encoded strings alive as long as self
        self._cfg = Config(spi_device.encode(), spi_speed_hz, drdy_chip.encode() if drdy_chip else None,
                           drdy_line, v_ref, refmux, drate, filter, gain, pos, neg, chop, bypass, timeout_ms)
        _check(lib.ads1263_open(self.dev, self._cfg))

    def close(self):
        lib.ads1263_close(self.dev)
        self.dev.spi_fd = self.dev.drdy_fd = -1  # Second close() is harmless

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def set_input(self, pos, neg): _check(lib.ads1263_set_input(self.dev, pos, neg))
    def set_gain(self, gain): _check(lib.ads1263_set_gain(self.dev, gain))
    def set_drate(self, drate): _check(lib.ads1263_set_drate(self.dev, drate))
    def set_filter(self, filter): _check(lib.ads1263_set_filter(self.dev, filter))
    def set_reference(self, refmux, v_ref): _check(lib.ads1263_set_reference(self.dev, refmux, v_ref))
    def set_chop(self, on): _check(lib.ads1263_set_chop(self.dev, on))
    def set_bypass(self, on): _check(lib.ads1263_set_bypass(self.dev, on))
    def set_idac(self, idac, pin, current): _check(lib.ads1263_set_idac(self.dev, idac, pin, current))
    def calibrate(self, cmd=CMD_SFOCAL1): _check(lib.ads1263_calibrate(self.dev, cmd))
    def to_volts(self, raw): return lib.ads1263_to_volts(self.dev, raw)
    def to_celsius(self, raw): return lib.ads1263_to_celsius(self.dev, raw)

    def adc2_start(self, v_ref=2.5, refmux=REF_INTERNAL, drate=ADC2_DRATE_10, gain=GAIN_1, pos=AIN0, neg=AIN1):
        _check(lib.ads1263_adc2_start(self.dev, Adc2Config(v_ref, refmux, drate, gain, pos, neg)))

    def adc2_set_input(self, pos, neg): _check(lib.ads1263_adc2_set_input(self.dev, pos, neg))
    def adc2_set_gain(self, gain): _check(lib.ads1263_adc2_set_gain(self.dev, gain))
    def adc2_set_drate(self, drate): _check(lib.ads1263_adc2_set_drate(self.dev, drate))
    def adc2_set_reference(self, refmux, v_ref): _check(lib.ads1263_adc2_set_reference(self.dev, refmux, v_ref))
    def adc2_calibrate(self, cmd=CMD_SFOCAL2): _check(lib.ads1263_adc2_calibrate(self.dev, cmd))
    def adc2_to_volts(self, raw): return lib.ads1263_adc2_to_volts(self.dev, raw)
    def adc2_to_celsius(self, raw): return lib.ads1263_adc2_to_celsius(self.dev, raw)

    def read(self, _f=lib.ads1263_read):
        """One fresh conversion, raw code"""
        raw = C.c_int32()
        _check(_f(self.dev, raw))
        return raw.value

    def read_stream(self, n, _f=lib.ads1263_read_stream):
        """n consecutive conversions as a list; on error the valid ones are in ADS1263Error.samples"""
        raw, count = (C.c_int32 * n)(), C.c_size_t()
        result = _f(self.dev, raw, n, count)
        if result != OK:
            raise ADS1263Error(result, raw[:count.value])
        return raw[:]

    def scan(self, inputs, _f=lib.ads1263_scan):
        """One conversion of each (pos, neg) pair, list of raw codes"""
        n = len(inputs)
        raw = (C.c_int32 * n)()
        _check(_f(self.dev, ((C.c_uint8 * 2) * n)(*inputs), n, raw))
        return raw[:]

    def adc2_read(self): return self.read(lib.ads1263_adc2_read)
    def adc2_read_stream(self, n): return self.read_stream(n, lib.ads1263_adc2_read_stream)
    def adc2_scan(self, inputs): return self.scan(inputs, lib.ads1263_adc2_scan)

    def read_register(self, reg):
        value = C.c_uint8()
        _check(lib.ads1263_read_register(self.dev, reg, value))
        return value.value

    def write_register(self, reg, value): _check(lib.ads1263_write_register(self.dev, reg, value))
