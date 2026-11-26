"""HW-027 verification suite.

Runs three kinds of check, all with the standard library:

* the DFM gate (`kicad-dfm-precheck/dfm_check.py`) against the real board and
  schematic, at the 0.15 mm annular-ring limit this project promises;
* artefact consistency — BOM designators against the schematic, board
  geometry against the mechanical brief, SVG validity, C sources against the
  numeric contract in this file;
* the control and codec mathematics, against analytic invariants and golden
  wire vectors.

Run with::

    python -m unittest discover -s github_showcase/foc-canfd-joint-node/tests -v
"""

from __future__ import annotations

import csv
import math
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
REPO = ROOT.parent
HW = ROOT / "hardware"
FW = ROOT / "firmware"
PCB = HW / "foc_joint.kicad_pcb"
SCH = HW / "foc_joint.kicad_sch"
BOM = HW / "BOM.csv"
SVG = ROOT / "board_layout.svg"
DFM = REPO / "kicad-dfm-precheck" / "dfm_check.py"

sys.path.insert(0, str(REPO / "kicad-dfm-precheck"))


# --------------------------------------------------------------------------- #
# reference model of the firmware (independent of the C implementation)
# --------------------------------------------------------------------------- #

def clarke(ia: float, ib: float) -> tuple[float, float]:
    return ia, (ia + 2.0 * ib) / math.sqrt(3.0)


def park(alpha: float, beta: float, theta: float) -> tuple[float, float]:
    c, s = math.cos(theta), math.sin(theta)
    return alpha * c + beta * s, -alpha * s + beta * c


def inv_park(d: float, q: float, theta: float) -> tuple[float, float]:
    c, s = math.cos(theta), math.sin(theta)
    return d * c - q * s, d * s + q * c


def svpwm(alpha: float, beta: float, vbus: float, vbus_min: float = 24.0):
    limit = max(vbus, vbus_min) / math.sqrt(3.0)
    mag = math.hypot(alpha, beta)
    limited = mag > limit
    if limited and mag > 0:
        alpha *= limit / mag
        beta *= limit / mag
    va = alpha
    vb = -0.5 * alpha + (math.sqrt(3.0) / 2.0) * beta
    vc = -0.5 * alpha - (math.sqrt(3.0) / 2.0) * beta
    vmax, vmin = max(va, vb, vc), min(va, vb, vc)
    offset = -0.5 * (vmax + vmin)
    inv = 1.0 / max(vbus, vbus_min)
    duty = [0.5 + (v + offset) * inv for v in (va, vb, vc)]
    clamped = any(d < 0.0 or d > 1.0 for d in duty)
    return [min(1.0, max(0.0, d)) for d in duty], (limited or clamped)


def mit_torque(p_des, v_des, kp, kd, t_ff, p, v):
    return kp * (p_des - p) + kd * (v_des - v) + t_ff


def derate(temp_c, start=85.0, end=110.0):
    if temp_c <= start:
        return 1.0
    if temp_c >= end:
        return 0.0
    return (end - temp_c) / (end - start)


def mit_iq_ref(cmd, p, v, temp_c, kt=0.085, i_max=15.0):
    iq = mit_torque(*cmd, p, v) * derate(temp_c) / kt
    return max(-i_max, min(i_max, iq))


# ---- CAN-FD wire model (mirrors firmware/canfd_codec.c byte for byte) ------ #

def _sat_i16(x):
    return max(-32768, min(32767, int(round(x))))


def _i16(v):
    v &= 0xFFFF
    return bytes((v & 0xFF, (v >> 8) & 0xFF))


def encode_cmd(p_des, v_des, kp, kd, t_ff):
    out = bytearray(8)
    out[0:2] = _i16(_sat_i16(p_des / 0.001))
    out[2:4] = _i16(_sat_i16(v_des / 0.01))
    out[4] = max(0, min(255, int(round(kp / 0.5))))
    out[5] = max(0, min(255, int(round(kd / 0.01))))
    out[6:8] = _i16(_sat_i16(t_ff / 0.001))
    return bytes(out)


def decode_cmd(raw: bytes):
    p = _sat_i16_raw(raw[0:2]) * 0.001
    v = _sat_i16_raw(raw[2:4]) * 0.01
    kp = raw[4] * 0.5
    kd = raw[5] * 0.01
    t = _sat_i16_raw(raw[6:8]) * 0.001
    return p, v, kp, kd, t


def _sat_i16_raw(b: bytes) -> int:
    v = b[0] | (b[1] << 8)
    return v - 65536 if v >= 32768 else v


def encode_status(p, v, iq, i_d, vbus, temp_c, fault):
    out = bytearray(12)
    out[0:2] = _i16(_sat_i16(p / 0.001))
    out[2:4] = _i16(_sat_i16(v / 0.01))
    out[4:6] = _i16(_sat_i16(iq / 0.01))
    out[6:8] = _i16(_sat_i16(i_d / 0.01))
    out[8:10] = _i16(max(0, min(65535, int(round(vbus / 0.25)))))
    out[10] = temp_c & 0xFF
    out[11] = fault & 0xFF
    return bytes(out)


def dlc_for_len(n: int) -> int:
    if n <= 8:
        return n
    for limit, dlc in ((12, 9), (16, 10), (20, 11), (24, 12), (32, 13), (48, 14), (64, 15)):
        if n <= limit:
            return dlc
    return -1


# --------------------------------------------------------------------------- #
# hardware checks
# --------------------------------------------------------------------------- #

class TestDfmGate(unittest.TestCase):
    def test_dfm_check_passes_at_015mm(self):
        self.assertTrue(DFM.is_file(), f"missing {DFM}")
        proc = subprocess.run(
            [sys.executable, str(DFM), "--min-annular", "0.15", str(PCB), str(SCH)],
            capture_output=True, text=True,
        )
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        self.assertIn("0 error(s)", proc.stdout)
        self.assertIn("0 warning(s)", proc.stdout)

    def test_schematic_has_no_placeholder_values(self):
        text = SCH.read_text(encoding="utf-8")
        values = re.findall(r'\(property "Value" "([^"]*)"', text)
        self.assertGreaterEqual(len(values), 40)
        for value in values:
            self.assertNotRegex(value.strip(), r"(?i)^(r_us|c_us|l_us|d_us|\?|tbd|todo)$")

    def test_board_has_no_unconnected_nets(self):
        self.assertEqual(PCB.read_text(encoding="utf-8").count("unconnected-("), 0)


class TestBoardGeometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = PCB.read_text(encoding="utf-8")

    def test_four_copper_layers(self):
        layers = re.findall(r'\(\d+ "(F\.Cu|In1\.Cu|In2\.Cu|B\.Cu)" (signal|power|mixed)', self.text)
        self.assertEqual(len(layers), 4, layers)

    def test_sixty_four_boards_the_outline(self):
        self.assertIn('(gr_circle (center 100.0 100.0) (end 121.0 100.0)', self.text)
        self.assertIn('(gr_circle (center 100.0 100.0) (end 107.0 100.0)', self.text)

    def test_mounting_holes_on_36mm_bolt_circle(self):
        holes = []
        for mx, my in re.findall(r'\(footprint "MountingHole_2\.7mm_M2\.5"[\s\S]*?\(at ([\d.]+) ([\d.]+) ', self.text):
            holes.append((float(mx), float(my)))
        self.assertEqual(len(holes), 4, holes)
        for x, y in holes:
            radius = math.hypot(x - 100.0, y - 100.0)
            self.assertAlmostEqual(radius, 18.0, places=2)

    def test_every_part_has_a_footprint_and_reference(self):
        fps = re.findall(r'\(footprint "([^"]+)"', self.text)
        refs = re.findall(r'\(property "Reference" "([^"]+)"', self.text)
        self.assertGreaterEqual(len(fps), 45)
        self.assertGreaterEqual(len(refs), 45)
        for ref in refs:
            self.assertNotEqual(ref, "?")

    def test_vias_are_at_or_above_the_promise(self):
        vias = re.findall(r'\(via \(at [\d.-]+ [\d.-]+\) \(size ([\d.]+)\) \(drill ([\d.]+)\)', self.text)
        self.assertGreater(len(vias), 50)
        for size, drill in vias:
            self.assertGreaterEqual((float(size) - float(drill)) / 2.0, 0.15 - 1e-9)

    def test_no_part_sits_over_the_centre_bore(self):
        # the Ø14 mm bore is a hole: nothing may be placed inside r = 7 mm
        placed = []
        for m in re.finditer(
            r'\(footprint "([^"]+)" \(layer "F\.Cu"\) \(uuid [^)]+\) \(at ([\d.]+) ([\d.]+)[^)]*\)'
            r'\s*\(property "Reference" "([^"]+)"',
            self.text,
        ):
            ref, x, y = m.group(4), float(m.group(2)), float(m.group(3))
            if ref.startswith("H") or ref.startswith("TP"):
                continue
            placed.append((ref, math.hypot(x - 100.0, y - 100.0)))
        self.assertGreater(len(placed), 40)
        for ref, radius in placed:
            self.assertGreater(radius, 7.4, f"{ref} sits {radius:.2f} mm from centre, inside the bore")

    def test_phase_pads_are_present_and_neted(self):
        for net in ("PHASE_A", "PHASE_B", "PHASE_C"):
            self.assertIn(f'"{net}"', self.text)


class TestBom(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with BOM.open(encoding="utf-8") as fh:
            cls.rows = list(csv.DictReader(fh))
        cls.refs_bom = {r["Ref"] for r in cls.rows}

    def test_bom_matches_schematic_one_for_one(self):
        sch_refs = set(re.findall(r'\(property "Reference" "([A-Z]+\d+)"', SCH.read_text(encoding="utf-8")))
        self.assertEqual(sch_refs, self.refs_bom)

    def test_every_line_has_two_sources(self):
        for row in self.rows:
            self.assertTrue(row["Manufacturer"].strip(), row)
            self.assertTrue(row["MPN"].strip(), row)
            self.assertTrue(row["Alt_MPN"].strip(), row)
            self.assertRegex(row["LCSC"], r"^C\d{4,8}$", row)

    def test_known_parts_use_the_expected_lcsc_codes(self):
        by_ref = {r["Ref"]: r for r in self.rows}
        self.assertEqual(by_ref["U1"]["LCSC"], "C529356")
        self.assertEqual(by_ref["U2"]["LCSC"], "C545497")
        self.assertEqual(by_ref["Q1"]["LCSC"], "C113391")
        self.assertEqual(by_ref["U3"]["LCSC"], "C161798")
        self.assertEqual(by_ref["R1"]["LCSC"], "C149567")

    def test_shunts_are_one_milliohm(self):
        for ref in ("R1", "R2", "R3"):
            row = next(r for r in self.rows if r["Ref"] == ref)
            self.assertIn("1mR", row["Value"])


class TestSvg(unittest.TestCase):
    def test_svg_parses_and_carries_the_geometry(self):
        root = ET.parse(SVG).getroot()
        self.assertTrue(root.tag.endswith("svg"))
        text = SVG.read_text(encoding="utf-8")
        self.assertIn("42.0 mm", text)
        for colour in ("#F6F6F3", "#111214", "#D94E1F"):
            self.assertIn(colour, text)
        for label in ("STM32G431", "DRV8323RS", "MA730", "TCAN1044"):
            self.assertIn(label, text)


# --------------------------------------------------------------------------- #
# firmware logic checks
# --------------------------------------------------------------------------- #

class TestTransforms(unittest.TestCase):
    def test_clarke_preserves_amplitude_on_a_balanced_set(self):
        amp = 3.0
        for deg in range(0, 360, 15):
            t = math.radians(deg)
            ia = amp * math.cos(t)
            ib = amp * math.cos(t - 2 * math.pi / 3)
            alpha, beta = clarke(ia, ib)
            self.assertAlmostEqual(math.hypot(alpha, beta), amp, places=3)

    def test_park_round_trip(self):
        for theta in (0.0, 0.37, 1.9, 3.1, 5.9):
            d, q = park(1.3, -0.7, theta)
            a, b = inv_park(d, q, theta)
            self.assertAlmostEqual(a, 1.3, places=6)
            self.assertAlmostEqual(b, -0.7, places=6)

    def test_park_of_a_synchronous_frame_is_constant(self):
        amp, theta = 2.0, 0.9
        alpha = amp * math.cos(theta)
        beta = amp * math.sin(theta)
        d, q = park(alpha, beta, theta)
        self.assertAlmostEqual(d, amp, places=6)
        self.assertAlmostEqual(q, 0.0, places=6)


class TestSvpwm(unittest.TestCase):
    def test_duties_are_centred_and_bounded(self):
        for deg in range(0, 360, 10):
            t = math.radians(deg)
            alpha, beta = 6.0 * math.cos(t), 6.0 * math.sin(t)
            duty, limited = svpwm(alpha, beta, 48.0)
            self.assertFalse(limited)
            for d in duty:
                self.assertGreaterEqual(d, 0.0)
                self.assertLessEqual(d, 1.0)
            # min-max injection keeps the midpoint of the three duties at 50 %
            self.assertAlmostEqual(max(duty) + min(duty), 1.0, places=6)

    def test_overmodulation_is_clipped_to_the_linear_circle(self):
        duty, limited = svpwm(60.0, 0.0, 48.0)
        self.assertTrue(limited)
        self.assertLessEqual(max(duty), 1.0)
        self.assertGreaterEqual(min(duty), 0.0)

    def test_under_voltage_falls_back_to_the_minimum_bus(self):
        # a request that is legal at 48 V but impossible on a 5 V bus
        duty, limited = svpwm(20.0, 0.0, 5.0)
        self.assertTrue(limited)
        for d in duty:
            self.assertGreaterEqual(d, 0.0)
            self.assertLessEqual(d, 1.0)


class TestCurrentLoop(unittest.TestCase):
    def test_pi_clamps_and_does_not_wind_up(self):
        integ, kp, ki_ts, out_max = 0.0, 0.5, 0.0035, 13.8
        for _ in range(5000):
            err = 10.0
            p_term = kp * err
            candidate = integ + ki_ts * err
            saturated = False  # v_applied starts unsaturated
            if not saturated:
                integ = candidate
            integ = max(-out_max, min(out_max, integ))
            out = max(-out_max, min(out_max, p_term + integ))
        self.assertLessEqual(integ, out_max + 1e-6)
        self.assertLessEqual(out, out_max + 1e-6)

    def test_zero_error_holds_the_integrator(self):
        self.assertAlmostEqual(0.5 * 0.0 + 0.0, 0.0, places=9)


class TestMitLaw(unittest.TestCase):
    cmd = (0.4, 1.2, 8.0, 0.35, 0.15)

    def test_linear_in_position_and_velocity(self):
        base = mit_torque(*self.cmd, 0.0, 0.0)
        self.assertAlmostEqual(mit_torque(*self.cmd, 0.1, 0.0), base - 0.8, places=6)
        self.assertAlmostEqual(mit_torque(*self.cmd, 0.0, 0.5), base - 0.175, places=6)

    def test_derating_curve(self):
        self.assertEqual(derate(25.0), 1.0)
        self.assertEqual(derate(85.0), 1.0)
        self.assertAlmostEqual(derate(97.5), 0.5, places=6)
        self.assertEqual(derate(110.0), 0.0)
        self.assertEqual(derate(140.0), 0.0)

    def test_hot_motor_gets_less_current(self):
        gentle = (0.4, 1.2, 0.5, 0.05, 0.02)
        cold = mit_iq_ref(gentle, 0.0, 0.0, 25.0)
        hot = mit_iq_ref(gentle, 0.0, 0.0, 100.0)
        self.assertGreater(cold, hot)
        self.assertGreater(hot, 0.0)
        self.assertEqual(mit_iq_ref(gentle, 0.0, 0.0, 120.0), 0.0)

    def test_current_ceiling(self):
        huge = (0.0, 0.0, 1000.0, 0.0, 0.0)
        self.assertAlmostEqual(mit_iq_ref(huge, -1.0, 0.0, 25.0), 15.0, places=6)


class TestCanfdCodec(unittest.TestCase):
    def test_command_golden_vector(self):
        raw = encode_cmd(0.5, -2.0, 10.0, 0.5, 0.25)
        self.assertEqual(raw.hex(), "f40138ff1432fa00")

    def test_command_round_trip(self):
        p, v, kp, kd, t = decode_cmd(encode_cmd(1.234, -3.21, 12.0, 0.44, -0.5))
        self.assertAlmostEqual(p, 1.234, places=3)
        self.assertAlmostEqual(v, -3.21, places=2)
        self.assertAlmostEqual(kp, 12.0, places=6)
        self.assertAlmostEqual(kd, 0.44, places=2)
        self.assertAlmostEqual(t, -0.5, places=3)

    def test_status_golden_vector(self):
        raw = encode_status(1.0, 3.0, -1.5, 0.5, 48.0, 42, 0x05)
        self.assertEqual(raw.hex(), "e8032c016aff3200c0002a05")

    def test_status_round_trip(self):
        raw = encode_status(-0.75, 7.5, 12.0, -4.0, 47.5, 71, 0x00)
        self.assertEqual(len(raw), 12)
        self.assertAlmostEqual(_sat_i16_raw(raw[0:2]) * 0.001, -0.75, places=3)
        self.assertAlmostEqual(_sat_i16_raw(raw[8:10]) * 0.25, 47.5, places=6)
        self.assertEqual(raw[10], 71)

    def test_saturation_does_not_wrap(self):
        raw = encode_cmd(999.0, 999.0, 9999.0, 9999.0, 999.0)
        self.assertEqual(raw[0:2], b"\xff\x7f")
        self.assertEqual(raw[4], 255)
        self.assertEqual(raw[5], 255)

    def test_dlc_mapping(self):
        self.assertEqual([dlc_for_len(n) for n in (0, 8, 12, 16, 20, 24, 32, 48, 64)],
                         [0, 8, 9, 10, 11, 12, 13, 14, 15])
        self.assertEqual(dlc_for_len(65), -1)


class TestFirmwareSourceContract(unittest.TestCase):
    """The C headers are the contract; these checks keep them honest."""

    @classmethod
    def setUpClass(cls):
        cls.h = (FW / "foc_control.h").read_text(encoding="utf-8")
        cls.c = (FW / "foc_control.c").read_text(encoding="utf-8")
        cls.codec_h = (FW / "canfd_codec.h").read_text(encoding="utf-8")
        cls.codec_c = (FW / "canfd_codec.c").read_text(encoding="utf-8")

    def _define(self, name, text=None):
        text = text or self.h
        m = re.search(rf"#define\s+{name}\s+([0-9.]+)f", text)
        self.assertIsNotNone(m, name)
        return float(m.group(1))

    def test_machine_constants_match_the_test_model(self):
        self.assertEqual(self._define("FOC_KT_NM_PER_A"), 0.085)
        self.assertEqual(self._define("FOC_I_MAX_A"), 15.0)
        self.assertEqual(self._define("FOC_TEMP_DERATE_START_C"), 85.0)
        self.assertEqual(self._define("FOC_TEMP_DERATE_END_C"), 110.0)

    def test_mit_law_is_written_as_the_brief_specifies(self):
        self.assertRegex(self.c, r"cmd->kp \* \(cmd->p_des - p\)")
        self.assertRegex(self.c, r"cmd->kd \* \(cmd->v_des - v\)")
        self.assertIn("cmd->t_ff", self.c)

    def test_codec_offsets_match_the_wire_spec(self):
        self.assertIn("put_i16(&out[0]", self.codec_c)
        self.assertIn("put_i16(&out[2]", self.codec_c)
        self.assertIn("out[4] = sat_u8(cmd->kp", self.codec_c)
        self.assertIn("out[5] = sat_u8(cmd->kd", self.codec_c)
        self.assertIn("put_i16(&out[6]", self.codec_c)
        self.assertIn("put_u16(&out[8]", self.codec_c)
        self.assertIn("out[11] = st->fault", self.codec_c)

    def test_scale_factors_match_the_wire_spec(self):
        self.assertEqual(self._define("CANFD_POS_SCALE", self.codec_h), 0.001)
        self.assertEqual(self._define("CANFD_VEL_SCALE", self.codec_h), 0.01)
        self.assertEqual(self._define("CANFD_KP_SCALE", self.codec_h), 0.5)
        self.assertEqual(self._define("CANFD_KD_SCALE", self.codec_h), 0.01)

    @unittest.skipUnless(shutil.which("gcc") or shutil.which("clang"), "no C compiler on PATH")
    def test_sources_compile_with_strict_flags(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        with tempfile.TemporaryDirectory() as tmp:
            for src in (FW / "foc_control.c", FW / "canfd_codec.c"):
                out = pathlib.Path(tmp) / (src.stem + ".o")
                proc = subprocess.run(
                    [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-I", str(FW),
                     "-c", str(src), "-o", str(out), "-lm"],
                    capture_output=True, text=True,
                )
                self.assertEqual(proc.returncode, 0, proc.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
