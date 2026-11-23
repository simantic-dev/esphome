# Bosch BME280 temperature / pressure / humidity sensor (7-bit 0x76 or 0x77) as a scripted I2C peer.
# Port of crates/engine/src/models/peers/bme280.rs (pyrite), which follows Renode's stock I2C/BME280.cs.
# Timing from pyrite#2943 and #2923. Source: BST-BME280-DS002 Appendix B section 9.1 p.51, pp.28, 30.
#
# Behaviour kept from the C#: a register pointer. A write of one byte sets the pointer; a write of more
# bytes is a (pointer, value) stream. A read returns consecutive registers from the last pointer, and
# wraps at 0xFF. Only ID (0xD0), RESET (0xE0), CTRL_HUM, STATUS, CTRL_MEAS and CONFIG accept writes.
# Writing 0xB6 to RESET re-runs the reset and re-encodes the readings.
#
# Stimulus. Defaults are all 0 (the C# initialisers). An overlay sets them before this file runs, e.g.
# `script: "TEMPERATURE_C = 27.0; PRESSURE_HPA = 1480; HUMIDITY_PCT = 45; exec(open(...).read())"`. The
# part runs the datasheet compensation backwards to produce the raw words, and fixes the calibration block
# so the inversion is exact: a driver's forward compensation lands back on these numbers.
# Config, ns of virtual time (None = the datasheet formula):
#   MEASUREMENT_NS   replaces the measurement time t.
# Config (`config:` JSON in the overlay, ctx.config): measurement_ns (default MEASUREMENT_NS, None = formula);
# temperature_c, pressure_kpa, humidity_pct set the starting environment in place of the globals above.
# PRESSURE_HPA holds kPa despite its name (Renode's unit; fixtures set 101.3 for one atmosphere).
# calibration: "datasheet" loads the datasheet's example pressure trimming and encodes the raw word so a driver's
# compensation reads back the configured pressure. Without it the peer keeps Renode's degenerate trimming and
# encoding, which drivers decode to other values (Zephyr 103.125 kPa for 148, ESPHome ~515 hPa); fixtures
# that pin Renode's readings depend on that.
#
# Measurement time, typical (Appendix B section 9.1 p.51), in ms:
#   t = 1 + [2 * T_os] + [2 * P_os + 0.5] + [2 * H_os + 0.5]
# where a term counts only when that channel's oversampling is enabled (osrs 1..5 = x1, x2, x4, x8, x16).
# Forced mode (CTRL_MEAS mode 01) measures once and returns to sleep. Normal mode (11) repeats every
# t + t_sb, t_sb from CONFIG[7:5] (0.5, 62.5, 125, 250, 500, 1000, 10, 20 ms; p.28, p.30).
# STATUS bit 3 (measuring) is set while a measurement is in flight.
#
# Control port (`controlPort:`): set property "temperature_c", "pressure_kpa" or "humidity_pct" on the peer's
# board name to change the environment; the data registers are re-encoded at once. get property "env" for
# all three.
#
# MODELED SIMPLIFICATION: the environment is constant between control writes, so the data registers hold the encoded sample
# throughout; only STATUS.measuring follows the timer.

TEMPERATURE_C = globals().get("TEMPERATURE_C", 0.0)
PRESSURE_HPA = globals().get("PRESSURE_HPA", 0.0)
HUMIDITY_PCT = globals().get("HUMIDITY_PCT", 0.0)
MEASUREMENT_NS = globals().get("MEASUREMENT_NS", None)

RESET_REQUEST_VAL = 0xB6
R_CALIB00, R_ID, R_RESET, R_CALIB26 = 0x88, 0xD0, 0xE0, 0xE1
R_CTRL_HUM, R_STATUS, R_CTRL_MEAS, R_CONFIG = 0xF2, 0xF3, 0xF4, 0xF5
DIG_P = (36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000)
R_PRESS_MSB, R_PRESS_XLSB, R_TEMP_MSB, R_TEMP_XLSB, R_HUM_MSB, R_HUM_LSB = 0xF7, 0xF9, 0xFA, 0xFC, 0xFD, 0xFE
STATUS_MEASURING = 1 << 3
T_SB_NS = [500_000, 62_500_000, 125_000_000, 250_000_000, 500_000_000, 1_000_000_000, 10_000_000, 20_000_000]

IDLE, RECEIVED_FIRST, WAITING_ADDR, WRITING_WAIT_VALUE, READING = range(5)


def _trunc_div(a, b):
    """C# integer division: truncates toward zero."""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


def _round_half_even(x):
    return float(round(x))


def _s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def _s32(v):
    v &= 0xFFFFFFFF
    return v - 0x100000000 if v & 0x80000000 else v


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        cfg = getattr(ctx, "config", None) or {}
        self.measurement_ns = cfg.get("measurement_ns", MEASUREMENT_NS)
        self.datasheet = cfg.get("calibration") == "datasheet"
        global TEMPERATURE_C, PRESSURE_HPA, HUMIDITY_PCT
        TEMPERATURE_C = float(cfg.get("temperature_c", TEMPERATURE_C))
        PRESSURE_HPA = float(cfg.get("pressure_kpa", PRESSURE_HPA))
        HUMIDITY_PCT = float(cfg.get("humidity_pct", HUMIDITY_PCT))
        self.regs = bytearray(256)
        self.state = IDLE
        self.selected = 0
        self.started_ns = None
        self.cycling = False
        self._do_reset()

    def _do_reset(self):
        self.regs = bytearray(256)
        self.regs[R_HUM_MSB] = 0x80
        self.regs[R_TEMP_MSB] = 0x80
        self.regs[R_PRESS_MSB] = 0x80
        self.regs[R_ID] = 0x60
        dig_t1t2 = 2 << 14
        self.regs[R_CALIB00:R_CALIB00 + 4] = bytes([dig_t1t2 & 0xFF, dig_t1t2 >> 8, dig_t1t2 & 0xFF, dig_t1t2 >> 8])
        if self.datasheet:
            # dig_P1..P9: the datasheet's example trimming (section 8.2), so a driver's pressure compensation has
            # its real resolution and reads back the configured pressure.
            for i, v in enumerate(DIG_P):
                self.regs[R_CALIB00 + 6 + 2 * i:R_CALIB00 + 8 + 2 * i] = (v & 0xFFFF).to_bytes(2, "little")
        else:
            # Renode's degenerate set (P1 = P2 = 1, P8 = 16384): the fixtures that pin Renode's readings expect it.
            dig_p1p2, dig_p8 = 1, 2 << 13
            self.regs[R_CALIB00 + 6:R_CALIB00 + 10] = bytes([dig_p1p2 & 0xFF, dig_p1p2 >> 8, dig_p1p2 & 0xFF, dig_p1p2 >> 8])
            self.regs[R_CALIB00 + 20] = dig_p8 & 0xFF
            self.regs[R_CALIB00 + 21] = dig_p8 >> 8
        dig_h2, dig_h4, dig_h5, dig_h6 = 361, 321, 50, 30
        self.regs[R_CALIB26] = dig_h2 & 0xFF
        self.regs[R_CALIB26 + 1] = dig_h2 >> 8
        self.regs[R_CALIB26 + 3] = (dig_h4 >> 4) & 0xFF
        self.regs[R_CALIB26 + 4] = ((dig_h4 & 0x0F) | ((dig_h5 & 0x0F) << 4)) & 0xFF
        self.regs[R_CALIB26 + 5] = (dig_h5 >> 4) & 0xFF
        self.regs[R_CALIB26 + 6] = dig_h6 & 0xFF
        self.selected = 0
        self.started_ns = None
        self._encode_temperature()
        self._encode_humidity()
        self._encode_pressure()
        self.state = IDLE

    def set_property(self, name, value):
        global TEMPERATURE_C, PRESSURE_HPA, HUMIDITY_PCT
        if name == "temperature_c":
            TEMPERATURE_C = float(value)
        elif name == "pressure_kpa":
            PRESSURE_HPA = float(value)
        elif name == "humidity_pct":
            HUMIDITY_PCT = float(value)
        else:
            raise KeyError(name)
        self._encode_temperature()
        self._encode_humidity()
        self._encode_pressure()

    def get_property(self, name):
        if name != "env":
            raise KeyError(name)
        return {"temperature_c": TEMPERATURE_C, "pressure_kpa": PRESSURE_HPA, "humidity_pct": HUMIDITY_PCT}

    def _u16(self, lo):
        return self.regs[lo] | (self.regs[lo + 1] << 8)

    def _i16(self, lo):
        return _s16(self._u16(lo))

    def _adc_temperature(self):
        dig_t1 = self._u16(R_CALIB00)
        dig_t2 = self._i16(R_CALIB00 + 2)
        return int(_round_half_even(
            ((TEMPERATURE_C * 100.0 * 256.0 - 128.0) / (5.0 * dig_t2) * 2048.0 + dig_t1 * 2.0) * 8.0))

    def _encode_temperature(self):
        t = self._adc_temperature()
        self.regs[R_TEMP_XLSB] = ((t & 0x0F) << 4) & 0xFF
        self.regs[R_TEMP_XLSB - 1] = (t >> 4) & 0xFF
        self.regs[R_TEMP_MSB] = (t >> 12) & 0xFF

    def _t_fine(self):
        """t_fine as a driver computes it from the temperature word (datasheet 4.2.3, 32-bit form)."""
        t1, t2, t3 = self._u16(R_CALIB00), self._i16(R_CALIB00 + 2), self._i16(R_CALIB00 + 4)
        adc = self._adc_temperature()
        v1 = (((adc >> 3) - (t1 << 1)) * t2) >> 11
        v2 = (((((adc >> 4) - t1) * ((adc >> 4) - t1)) >> 12) * t3) >> 14
        return v1 + v2

    def _pressure_q24_8(self, adc_p, t_fine):
        """A driver's pressure in Pa as Q24.8 from a raw word (datasheet 4.2.3, 64-bit form)."""
        p1, p2, p3, p4, p5, p6, p7, p8, p9 = (self._u16(R_CALIB00 + 6),) + tuple(self._i16(R_CALIB00 + 8 + 2 * i) for i in range(8))
        v1 = t_fine - 128000
        v2 = v1 * v1 * p6 + ((v1 * p5) << 17) + (p4 << 35)
        v1 = ((v1 * v1 * p3) >> 8) + ((v1 * p2) << 12)
        v1 = (((1 << 47) + v1) * p1) >> 33
        if v1 == 0:
            return 0
        p = _trunc_div((((1048576 - adc_p) << 31) - v2) * 3125, v1)
        v1 = (p9 * (p >> 13) * (p >> 13)) >> 25
        v2 = (p8 * p) >> 19
        return ((p + v1 + v2) >> 8) + (p7 << 4)

    def _encode_pressure(self):
        if not self.datasheet:
            self._encode_pressure_renode()
            return
        # The raw word whose compensated value is nearest PRESSURE_HPA (in kPa, see the header): the forward
        # formula falls as the word rises, so bisect the 20-bit range.
        target, t_fine = PRESSURE_HPA * 1000.0 * 256.0, self._t_fine()
        lo, hi = 0, (1 << 20) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self._pressure_q24_8(mid, t_fine) > target:
                lo = mid + 1
            else:
                hi = mid
        p = lo
        self.regs[R_PRESS_XLSB] = ((p & 0x0F) << 4) & 0xFF
        self.regs[R_PRESS_XLSB - 1] = (p >> 4) & 0xFF
        self.regs[R_PRESS_MSB] = (p >> 12) & 0xFF

    def _encode_pressure_renode(self):
        dig_t1 = self._u16(R_CALIB00)
        dig_t2 = self._i16(R_CALIB00 + 2)
        dig_p1 = self._u16(R_CALIB00 + 6)
        dig_p2 = self._i16(R_CALIB00 + 8)
        dig_p8 = self._i16(R_CALIB00 + 20)
        adc_temp = self._adc_temperature()
        inner = _s32(_s32(_trunc_div(adc_temp, 8) - dig_t1 * 2) * dig_t2)
        inner = _trunc_div(inner, 2048)
        v1 = _trunc_div(((2 << 47) + inner - 128_000) * dig_p2 * 4096 * dig_p1, 2 << 33)
        p = int(_round_half_even(
            -((PRESSURE_HPA - 52.0) * float(2 << 27) / float(dig_p8 + 1) * float(v1))
            / (3125.0 * float(2 << 31)) * 2.0 + 1_048_576.0))
        self.regs[R_PRESS_XLSB] = ((p & 0x0F) << 4) & 0xFF
        self.regs[R_PRESS_XLSB - 1] = (p >> 4) & 0xFF
        self.regs[R_PRESS_MSB] = (p >> 12) & 0xFF

    def _encode_humidity(self):
        h0, h100 = 20650, 38550
        h = int(h0 + (h100 - h0) * HUMIDITY_PCT / 100.0) & 0xFFFF
        self.regs[R_HUM_LSB] = h & 0xFF
        self.regs[R_HUM_MSB] = h >> 8

    def _now_ns(self):
        return self.ctx.now_us * 1000

    def _measure_ns(self):
        if self.measurement_ns is not None:
            return self.measurement_ns
        # osrs 1..5 -> x1, x2, x4, x8, x16; 0 = skipped (the term is left out). Tenths of ms keep 0.5 integral.
        def factor(osrs):
            return 0 if osrs == 0 else 1 << (min(osrs, 5) - 1)
        t = factor(self.regs[R_CTRL_MEAS] >> 5 & 7)
        p = factor(self.regs[R_CTRL_MEAS] >> 2 & 7)
        h = factor(self.regs[R_CTRL_HUM] & 7)
        ms_x10 = 10
        if t > 0:
            ms_x10 += 20 * t
        if p > 0:
            ms_x10 += 20 * p + 5
        if h > 0:
            ms_x10 += 20 * h + 5
        return ms_x10 * 100_000

    def _standby_ns(self):
        return T_SB_NS[self.regs[R_CONFIG] >> 5 & 7]

    def _measuring(self):
        if self.started_ns is None:
            return False
        elapsed = self._now_ns() - self.started_ns
        t = self._measure_ns()
        if self.cycling:
            return elapsed % (t + self._standby_ns()) < t
        return elapsed < t

    def _start_measurement(self):
        self.started_ns = self._now_ns()
        self.cycling = self.regs[R_CTRL_MEAS] & 3 == 3
        if not self.cycling:
            self.regs[R_CTRL_MEAS] &= 0xFC   # forced mode returns to sleep at once

    def _register(self, r):
        if r == R_STATUS:
            stored = self.regs[R_STATUS] & ~STATUS_MEASURING & 0xFF
            return stored | STATUS_MEASURING if self._measuring() else stored
        return self.regs[r]

    def _write_register(self, address, value):
        if address in (R_CTRL_HUM, R_STATUS, R_CONFIG):
            self.regs[address] = value
        elif address == R_CTRL_MEAS:
            self.regs[R_CTRL_MEAS] = value
            if value & 3 != 0:
                self._start_measurement()
        elif address == R_RESET:
            self.regs[R_RESET] = value
            if value == RESET_REQUEST_VAL:
                self._do_reset()
        # every other register is read-only

    def on_write(self, data):
        if self.state == READING:
            self.state = IDLE   # a new transfer after a read: the STOP the controller did not report
        for b in data:
            if self.state == IDLE:
                self.selected = b
                self.state = RECEIVED_FIRST
            elif self.state in (RECEIVED_FIRST, WRITING_WAIT_VALUE):
                self._write_register(self.selected, b)
                self.state = WAITING_ADDR
            elif self.state == WAITING_ADDR:
                self.selected = b
                self.state = WRITING_WAIT_VALUE
        return len(data)

    def on_read(self, count):
        # A read may start in any state and reuses the pointer the last write set.
        self.state = READING
        out = bytearray()
        for _ in range(count):
            out.append(self._register(self.selected))
            self.selected = (self.selected + 1) & 0xFF
        return bytes(out)

    def on_finish(self):
        # A STOP before the read phase (documentation permits it) leaves the pointer; otherwise go idle.
        if self.state != RECEIVED_FIRST:
            self.state = IDLE
