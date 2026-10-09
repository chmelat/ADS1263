"""
Hardware-free self-check of the Python binding: the ctypes structures must match
ads1263_lib.h, otherwise values land in wrong fields silently.
With arguments (spidev gpiochip drdy_line) it also reads the connected chip.
Build & run: make test-py (make hwtest runs it with the chip)
"""

import os
import sys
import tempfile
import ads1263 as A

# Every field of a failed open's config is copied into dev->cfg first: read it back
cfg = A.Config(b"/nonexistent/spidev", 1234567, b"/nonexistent/gpiochip", 77, 1.25, A.REF_AIN4_AIN5,
               A.DRATE_10, A.FILTER_FIR, A.GAIN_32, A.AIN7, A.AINCOM, True, False, 4321)
dev = A.Device()
C_bytes = A.C.cast(A.C.byref(dev), A.C.POINTER(A.C.c_uint8 * A.C.sizeof(dev))).contents
for i in range(len(C_bytes)):
    C_bytes[i] = 0xAA                                   # Garbage everywhere before open
assert A.lib.ads1263_open(dev, cfg) == A.ERROR_COMMUNICATION
assert dev.spi_fd == dev.drdy_fd == -1
for name, _ in A.Config._fields_:
    assert getattr(dev.cfg, name) == getattr(cfg, name), name
# ... and the tail is set to reset values: the C structure ends where Device does
assert dev.status == dev.status2 == 0
c2 = dev.cfg2
assert (c2.v_ref, c2.refmux, c2.drate, c2.gain, c2.pos, c2.neg) == (2.5, A.REF_INTERNAL, A.ADC2_DRATE_10,
                                                                   A.GAIN_1, A.AIN0, A.AIN1)

# Parameter checks see the right fields
for bad in ({"timeout_ms": 0}, {"bypass": True, "gain": A.GAIN_2}, {"drate": A.DRATE_400, "filter": A.FILTER_FIR}):
    try:
        A.ADS1263("/nonexistent/spidev", **bad)
        assert False, bad
    except A.ADS1263Error as e:
        assert e.code == A.ERROR_PARAMETER, bad
try:
    A.ADS1263("/nonexistent/spidev")
    assert False
except A.ADS1263Error as e:
    assert e.code == A.ERROR_COMMUNICATION and str(e) == A.strerror(A.ERROR_COMMUNICATION)

# Conversions read v_ref and gain through the handle
dev.cfg.v_ref, dev.cfg.gain = 2.5, A.GAIN_2
assert A.lib.ads1263_to_volts(dev, 1 << 30) == 0.625
dev.cfg2.v_ref, dev.cfg2.gain = 5.0, A.GAIN_4
assert A.lib.ads1263_adc2_to_volts(dev, 1 << 22) == 0.625

# Array arguments reach C (no SPI open: inputs are validated first, then the bus fails)
adc = A.ADS1263.__new__(A.ADS1263)
adc.dev = dev
for scan in (adc.scan, adc.adc2_scan):
    for inputs, code in (([(A.AIN0, A.AIN1), (A.AIN2, 99)], A.ERROR_PARAMETER),
                         ([(A.AIN0, A.AIN1), (A.TEMP, A.TEMP)], A.ERROR_COMMUNICATION)):
        try:
            scan(inputs)
            assert False
        except A.ADS1263Error as e:
            assert e.code == code, (inputs, e.code)
for read_stream in (adc.read_stream, adc.adc2_read_stream):
    try:
        read_stream(10)
        assert False
    except A.ADS1263Error as e:
        assert e.code == A.ERROR_COMMUNICATION and e.samples == []
adc.close()

assert A.sps(A.DRATE_2_5) == 2.5 and A.sps(A.DRATE_38400) == 38400 and A.sps(99) == 0
assert A.adc2_sps(A.ADC2_DRATE_800) == 800

with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, "vref")
    with open(path, "w") as f:
        f.write("# measured\n2.5012\n")
    assert A.load_vref(path) == 2.5012
    try:
        A.load_vref(os.path.join(d, "missing"))
        assert False
    except FileNotFoundError:
        pass

if len(sys.argv) == 4:
    with A.ADS1263(sys.argv[1], drdy_chip=sys.argv[2], drdy_line=int(sys.argv[3])) as adc:
        avdd = 4 * adc.to_volts(adc.scan([(A.AVDD_MON, A.AVDD_MON)])[0])
        assert 4.75 < avdd < 5.25, avdd
        adc.set_input(A.TEMP, A.TEMP)
        t1 = adc.to_celsius(adc.read())
        assert len(adc.read_stream(10)) == 10
        adc.adc2_start(drate=A.ADC2_DRATE_100, pos=A.TEMP, neg=A.TEMP)
        t2 = adc.adc2_to_celsius(adc.adc2_read())
        assert 0 < t1 < 70 and abs(t2 - t1) < 2, (t1, t2)
        assert adc.dev.status & A.STATUS_ADC1          # Status byte of the last read (fresh data) reaches Python
        print(f"Chip: AVDD {avdd:.4f} V, temperature {t1:.2f} C (ADC2 {t2:.2f} C)")

print("Python binding OK")
