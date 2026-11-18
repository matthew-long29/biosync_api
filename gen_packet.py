#!/usr/bin/env python3
"""
Alexander Ranschaert - Oct 2026

Interactive packet generator for the BIOSYNC core.

Asks which command to send and how every datafield should be configured, then writes an
.sv file containing the answers (as comments) and a 96-bit vector:

    {seq[2:0], slave_fwd, opcode[3:0], datafield[87:0]}

which is exactly the `data96` argument of tb_helpers_pkg::make_valid_frame() (the helper
appends the CRC-16). Usage in a testbench:

    `include "pkt_flash_seq1.sv"            // declares localparam PKT_FLASH_SEQ1
    ...
    frame = make_valid_frame(PKT_FLASH_SEQ1);
    send_frame(clk_32k, clk_1k, clk_4k, sampler_out, WAKEUP_CODE_DEF, frame);

Field order and widths are parsed from default_settings_packet_structs_pkg.sv, so the
script follows changes to the package. Only the descriptions below are hand-written; a
new field without a description still works, it just shows a generic help text.

With --stim FILE the packet is instead appended as one line to a stimulus file for the AMS
imux_driver (core/ams/imux_driver.sv): "<wait_us> <wakeup_code_hex> <data96_hex>  # comment".
Run the script once per packet to build up a multi-packet sequence.

Usage:
    python gen_packet.py [--pkg PATH] [--outdir DIR] [--stim FILE]
"""

import argparse
import datetime
import os
import re
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PKG = os.path.join(SCRIPT_DIR, "..", "rtl", "default_settings_packet_structs_pkg.sv")
DEFAULT_OUTDIR = os.path.join(SCRIPT_DIR, "packets")

CLK_HZ = 32768  # clk_32k: global timer / delay counters tick at this rate


# ----------------------------------------------------------------------------------------
# Hand-written documentation
# ----------------------------------------------------------------------------------------

# Command descriptions, keyed by opcode localparam name.
COMMAND_DOC = {
    "OP_HALT":           "Abort the running command; every sub-FSM returns to idle. No datafield.",
    "OP_CONF_RST":       "Write ALL configuration registers at once (radio, PMIC, TX, FLL, WPT).",
    "OP_POLL":           "Chip replies with preamble + battery ADC, WPT status and all config registers. No datafield.",
    "OP_RUN_FLL":        "Run the FLL calibration (uses the configured fll_cal_word). No datafield.",
    "OP_FLASH":          "Optical stimulation: pulse the selected red/blue LEDs (on/off, repeated).",
    "OP_ESTIM":          "Electrical stimulation: biphasic pulse (cathodic, inter-phase, anodic, ground), repeated.",
    "OP_OXY":            "Oxygen generation: apply a voltage between the selected working and ground electrodes (repeated). "
                         "NOT implemented in the core yet.",
    "OP_SETUP_REC_SPAD": "Select which SPAD macros (rows/cols) are enabled during a later REC_SPAD.",
    "OP_REC_SPAD":       "Record with the SPAD array (optionally with blue LED illumination).",
    "OP_READ_SPAD":      "Read out the SPAD counters of the selected macros and transmit them.",
}

# Field kinds:
#   int  : plain number
#   time : number of clk_32k cycles; the script also accepts "<x>ms" / "<x>us"
#   mask : one bit per row/column; the script accepts index lists ("0,2,4-6", "all", "none")
#   bool : single bit
# Keys are field names; (struct, field) keys override for one struct only.
FIELD_DOC = {
    # ---- CONF_RST ----
    "wakeup_code": ("int", "16-bit wake-up code the radio correlates against. WARNING: every frame "
                           "AFTER this one must be sent with the new code (send_frame wakeup_code arg)."),
    "pmic_fast_f": ("int", "PMIC fast-mode frequency control word."),
    "pmic_slow_f": ("int", "PMIC slow-mode frequency control word."),
    "rx_bw": ("int", "RX front-end bandwidth setting."),
    "rx_error_thresh": ("int", "Number of bit errors the wake-up correlator tolerates (false-positive threshold)."),
    "aoc_zt": ("int", "AOC: number of accumulated zeros (not necessarily consecutive) that triggers "
                      "comparator-threshold tuning."),
    "aoc_ot": ("int", "AOC: number of consecutive ones that triggers comparator-threshold tuning."),
    "override_aoc": ("bool", "1 = force the comparator threshold to its highest setting (OR'ed with the "
                             "pad_override_aoc pad)."),
    "tx_bias": ("int", "Transmitter bias current setting."),
    "tx_disable_bias_after_bit": ("bool", "1 = TX bias only on while sending a '1' (saves power); "
                                          "0 = bias on for the whole TX window."),
    "fll_cal_word": ("int", "FLL calibration word (target used by RUN_FLL)."),
    "wpt_dac_conf": ("int", "Wireless-power-transfer DAC setting."),

    # ---- LEDs (FLASH, REC_SPAD) ----
    "dac_conf": ("int", "LED driver DAC setting (LED current)."),
    "led_dac_conf": ("int", "LED driver DAC setting (LED current) used during the recording."),
    "en_row_r": ("mask", "Small RED LEDs: bit i enables row i."),
    "en_col_r": ("mask", "Small RED LEDs: bit i enables column i."),
    "en_row_r_l": ("mask", "Large RED LEDs: bit i enables row i."),
    "en_col_r_l": ("mask", "Large RED LEDs: bit i enables column i."),
    "en_row_b": ("mask", "Small BLUE LEDs: bit i enables row i."),
    "en_col_b": ("mask", "Small BLUE LEDs: bit i enables column i."),
    "en_row_b_l": ("mask", "Large BLUE LEDs: bit i enables row i."),
    "en_col_b_l": ("mask", "Large BLUE LEDs: bit i enables column i."),
    "time_on": ("time", "LED on-time per period."),
    "time_off": ("time", "LED off-time per period."),
    ("flash_datafield_t", "periods"): ("int", "Extra repetitions: total number of flashes = periods + 1."),

    # ---- ESTIM ----
    "cp_freq": ("int", "Stimulation charge-pump clock frequency setting."),
    "en_row_c": ("mask", "Cathode electrodes: bit i enables row i."),
    "en_col_c": ("mask", "Cathode electrodes: bit i enables column i."),
    "en_row_a": ("mask", "Anode electrodes: bit i enables row i."),
    "en_col_a": ("mask", "Anode electrodes: bit i enables column i."),
    "time_c": ("time", "Cathodic phase duration (cathodes sink, anodes source)."),
    "amp_c": ("int", "Cathodic phase current DAC setting."),
    "time_int": ("time", "Inter-phase gap duration (electrodes high-Z, charge pump stays on)."),
    "time_a": ("time", "Anodic phase duration (cathodes source, anodes sink)."),
    "amp_a": ("int", "Anodic phase current DAC setting."),
    "time_gnd": ("time", "Ground phase duration (all used electrodes grounded) - ends one period."),
    ("estim_datafield_t", "periods"): ("int", "Extra repetitions: total number of pulses = periods + 1."),
    ("estim_datafield_t", "gnd_ce"): ("bool", "1 = ground the counter electrode during the whole stimulation sequence."),

    # ---- OXY ----
    "en_row_we": ("mask", "Working electrodes: bit i enables row i."),
    "en_col_we": ("mask", "Working electrodes: bit i enables column i."),
    "en_row_gnd": ("mask", "Ground electrodes: bit i enables row i."),
    "en_col_gnd": ("mask", "Ground electrodes: bit i enables column i."),
    "set_v": ("int", "Electrode voltage setting."),
    "set_time": ("time", "Duration the voltage is applied per repetition."),
    "repeats": ("int", "Extra back-to-back repetitions without a break: total time = (repeats + 1) * set_time."),
    "gnd_time": ("time", "Ground phase duration after the last repetition (all used electrodes grounded); 0 = skip."),
    ("oxy_datafield_t", "gnd_ce"): ("bool", "1 = ground the counter electrode during the oxygenation and its ground phase."),

    # ---- SPAD ----
    "en_row_spad": ("mask", "Macro rows enabled during REC_SPAD: bit i = macro row i (bit 0 = top-left)."),
    "en_col_spad": ("mask", "Macro columns enabled during REC_SPAD: bit i = macro column i (bit 0 = top-left)."),
    "row_sel": ("mask", "Macro rows to read: bit i = macro row i (bit 0 = top-left)."),
    "col_sel": ("mask", "Macro columns to read: bit i = macro column i (bit 0 = top-left)."),
    "disable_counters": ("bool", "1 = turn the counter LDO off when the command ends (DESTROYS the counts); "
                                 "0 = keep it on so the counts survive for a later READ_SPAD."),
    "rst_before_rec": ("bool", "1 = reset the SPAD counters right before recording starts."),
    ("rec_spad_datafield_t", "stride"): ("int", "In-macro stride: enable SPAD 0, stride, 2*stride, ... of each "
                                                "macro (1 = all). 0 is treated as 1."),
    ("read_spad_datafield_t", "stride"): ("int", "In-macro readout stride: read SPAD 0, stride, 2*stride, ... of "
                                                 "each macro (1 = all). 0 is treated as 1."),
    "time_rec": ("time", "Recording time (LED on, SPADs counting)."),
    "delay_led": ("time", "Charge pump on -> LED on delay. 0 = skip this phase."),
    "delay_rec": ("time", "LED on -> start recording delay. 0 = skip this phase."),
    "cp_v_conf": ("int", "SPAD charge-pump output voltage setting."),
    "cp_freq_conf": ("int", "SPAD charge-pump clock frequency setting."),
}

# Upper bound overrides (default is 2^width - 1). Values are package localparam names or ints.
FIELD_MAX = {
    "stride": "SPAD_M",   # stride > SPAD_M only reads SPAD 0 of each macro
}

# Default-value localparams whose names don't follow <FIELD>_DEF.
DEFAULT_ALIAS = {
    "override_aoc": "AOC_OVERRIDE_DEF",
}


# ----------------------------------------------------------------------------------------
# Package parser
# ----------------------------------------------------------------------------------------

def clog2(n):
    return max(0, (int(n) - 1).bit_length())


SV_BASE = {"b": 2, "o": 8, "d": 10, "h": 16}


def eval_sv_expr(expr, params):
    """Evaluate a simple SV constant expression (literals, params, + - * /, $clog2)."""
    e = expr.strip()
    e = re.sub(r"\d*'[sS]?([bodhBODH])([0-9a-fA-F_]+)",
               lambda m: str(int(m.group(2).replace("_", ""), SV_BASE[m.group(1).lower()])), e)
    e = re.sub(r"'0\b", "0", e)
    e = e.replace("$clog2", "clog2")
    e = e.replace("/", "//")
    return int(eval(e, {"__builtins__": {}, "clog2": clog2}, dict(params)))


def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def parse_package(path):
    with open(path, "r") as f:
        text = strip_comments(f.read())

    params = {}
    for m in re.finditer(r"localparam\s+(?:int|logic)?\s*(?:\[[^\]]*\])?\s*(\w+)\s*=\s*([^;]+);", text):
        name, expr = m.group(1), m.group(2)
        try:
            params[name] = eval_sv_expr(expr, params)
        except Exception:
            pass  # not needed for packet forming

    structs = {}
    for m in re.finditer(r"typedef\s+struct\s+packed\s*\{(.*?)\}\s*(\w+)\s*;", text, flags=re.S):
        body, sname = m.group(1), m.group(2)
        fields = []
        for fm in re.finditer(r"logic\s*(?:\[([^:\]]+):([^\]]+)\])?\s*(\w+)\s*;", body):
            if fm.group(1) is None:
                width = 1
            else:
                width = eval_sv_expr(fm.group(1), params) - eval_sv_expr(fm.group(2), params) + 1
            fields.append((fm.group(3), width))
        structs[sname] = fields

    opcodes = sorted(((v, k) for k, v in params.items() if k.startswith("OP_")))
    return params, structs, opcodes


def struct_for_opcode(op_name, structs):
    """OP_FLASH -> flash_datafield_t (naming convention of the package)."""
    sname = op_name[3:].lower() + "_datafield_t"
    return sname if sname in structs else None


# ----------------------------------------------------------------------------------------
# Input helpers
# ----------------------------------------------------------------------------------------

def parse_number(s):
    """Accept 123, 0x7B, 0b1111011, 8'h7B, 8'd123, 8'b0111_1011."""
    s = s.strip().replace("_", "")
    m = re.fullmatch(r"(\d*)'([bodhBODH])([0-9a-fA-F]+)", s)
    if m:
        return int(m.group(3), SV_BASE[m.group(2).lower()])
    if s.isdigit():
        return int(s, 10)  # int(s, 0) rejects leading zeros such as "007"
    return int(s, 0)


def parse_mask(s, width):
    """Accept 'all', 'none', '0,2,4-6' (bit indices) or a raw number with 0x/0b/SV prefix."""
    s = s.strip().lower()
    if s == "all":
        return (1 << width) - 1
    if s in ("none", "-"):
        return 0
    if s.startswith(("0x", "0b", "0o")) or "'" in s:
        return parse_number(s)
    value = 0
    for part in s.replace(" ", "").split(","):
        if not part:
            continue
        if "-" in part:
            lo, hi = (int(x) for x in part.split("-", 1))
            if lo > hi:
                lo, hi = hi, lo
            idx = range(lo, hi + 1)
        else:
            idx = [int(part)]
        for i in idx:
            if not 0 <= i < width:
                raise ValueError("bit index %d out of range 0..%d" % (i, width - 1))
            value |= 1 << i
    return value


def parse_time(s):
    """Accept a cycle count or '<x>ms' / '<x>us' (converted to clk_32k cycles, rounded)."""
    t = s.strip().lower().replace(" ", "")
    for suffix, scale in (("ms", 1e-3), ("us", 1e-6)):
        if t.endswith(suffix):
            return int(round(float(t[:-len(suffix)]) * scale * CLK_HZ))
    return parse_number(t)


def parse_us(s):
    """Accept '<x>ms' / '<x>us' or a plain number of microseconds."""
    t = s.strip().lower().replace(" ", "")
    if t.endswith("ms"):
        return float(t[:-2]) * 1e3
    if t.endswith("us"):
        t = t[:-2]
    return float(t)


def mask_to_indices(value, width):
    idx = [i for i in range(width) if (value >> i) & 1]
    if not idx:
        return "none"
    if len(idx) == width:
        return "all"
    # compress into ranges
    out, start, prev = [], idx[0], idx[0]
    for i in idx[1:] + [None]:
        if i is not None and i == prev + 1:
            prev = i
            continue
        out.append(str(start) if start == prev else "%d-%d" % (start, prev))
        if i is not None:
            start = prev = i
    return ",".join(out)


def fmt_time(cycles):
    us = cycles * 1e6 / CLK_HZ
    return "%.3f ms" % (us / 1000) if us >= 1000 else "%.2f us" % us


def describe_value(kind, value, width):
    if kind == "mask":
        return "%s (bits: %s)" % (format(value, "0%db" % width), mask_to_indices(value, width))
    if kind == "time":
        return "%d cycles = %s" % (value, fmt_time(value))
    if kind == "bool":
        return str(value)
    return "%d (0x%X)" % (value, value)


def ask(prompt):
    try:
        return input(prompt)
    except EOFError:
        print()
        sys.exit("Aborted (EOF).")


def ask_value(name, width, kind, doc, default, vmax):
    """Prompt until a valid value is entered. Empty input -> default."""
    print()
    print("  %s  [%d bit%s]" % (name, width, "" if width == 1 else "s"))
    print("    %s" % doc)
    if kind == "mask":
        print("    Allowed: bit indices 0..%d, e.g. '0,2,4-6', 'all', 'none', or raw 0x../0b.." % (width - 1))
    elif kind == "bool":
        print("    Allowed: 0 or 1 (y/n)")
    elif kind == "time":
        print("    Allowed: 0..%d clk_32k cycles (max %s); also accepts e.g. '2ms', '500us'"
              % (vmax, fmt_time(vmax)))
    else:
        print("    Allowed: 0..%d (0x%X); decimal, 0x.., 0b.. or SV literal" % (vmax, vmax))
    print("    Default: %s" % describe_value(kind, default, width))

    while True:
        s = ask("    > ").strip()
        if s == "":
            return default
        try:
            if kind == "mask":
                v = parse_mask(s, width)
            elif kind == "bool":
                v = {"y": 1, "yes": 1, "n": 0, "no": 0}.get(s.lower())
                if v is None:
                    v = parse_number(s)
            elif kind == "time":
                v = parse_time(s)
            else:
                v = parse_number(s)
        except ValueError as e:
            print("    ! could not parse '%s' (%s), try again" % (s, e))
            continue
        if not 0 <= v <= vmax:
            print("    ! %d is out of range 0..%d, try again" % (v, vmax))
            continue
        if kind in ("time", "mask"):
            print("    = %s" % describe_value(kind, v, width))
        if kind == "mask" and s == "0":
            print("    (note: '0' is bit index 0; type 'none' for no bits)")
        return v


# ----------------------------------------------------------------------------------------
# Packet building
# ----------------------------------------------------------------------------------------

def crc16_ccitt_false(value, nbits, poly=0x1021, init=0xFFFF):
    """Same bitwise CRC as tb_helpers_pkg::make_valid_frame (MSB first)."""
    crc = init
    for i in range(nbits - 1, -1, -1):
        b = (value >> i) & 1
        top = (crc >> 15) & 1
        crc = (crc << 1) & 0xFFFF
        if top ^ b:
            crc ^= poly
    return crc


def field_doc(sname, fname):
    entry = FIELD_DOC.get((sname, fname)) or FIELD_DOC.get(fname)
    if entry:
        return entry
    return (None, "(no description in gen_packet.py FIELD_DOC)")


def field_default(fname, params):
    pname = DEFAULT_ALIAS.get(fname, fname.upper() + "_DEF")
    return params.get(pname, 0)


def sanity_checks(sname, values):
    """Non-fatal warnings for configurations that are legal but probably unintended."""
    w = []
    if sname == "estim_datafield_t":
        if values["en_row_a"] & values["en_row_c"] and values["en_col_a"] & values["en_col_c"]:
            w.append("an electrode row AND column are selected as both anode and cathode -> possible short.")
        if not (values["en_row_a"] and values["en_col_a"]):
            w.append("no anode selected (row or column mask is 0).")
        if not (values["en_row_c"] and values["en_col_c"]):
            w.append("no cathode selected (row or column mask is 0).")
        if values["amp_c"] == 0 or values["amp_a"] == 0:
            w.append("amp_c and/or amp_a is 0.")
    if sname == "oxy_datafield_t":
        if values["en_row_we"] & values["en_row_gnd"] and values["en_col_we"] & values["en_col_gnd"]:
            w.append("an electrode row AND column are selected as both working and ground -> possible short.")
        if not (values["en_row_we"] and values["en_col_we"]):
            w.append("no working electrode selected (row or column mask is 0).")
        if not (values["en_row_gnd"] and values["en_col_gnd"]):
            w.append("no ground electrode selected (row or column mask is 0).")
    if sname == "flash_datafield_t":
        red = (values["en_row_r"] and values["en_col_r"]) or (values["en_row_r_l"] and values["en_col_r_l"])
        blue = (values["en_row_b"] and values["en_col_b"]) or (values["en_row_b_l"] and values["en_col_b_l"])
        if not (red or blue):
            w.append("no LED selected (needs at least one row AND one column bit of the same LED group).")
        if values["time_on"] == 0:
            w.append("time_on is 0.")
    if sname == "rec_spad_datafield_t" and values["time_rec"] == 0:
        w.append("time_rec is 0.")
    if sname in ("setup_rec_spad_datafield_t", "read_spad_datafield_t"):
        rows = values.get("en_row_spad", values.get("row_sel"))
        cols = values.get("en_col_spad", values.get("col_sel"))
        if not (rows and cols):
            w.append("no SPAD macro selected (row or column mask is 0).")
    if sname == "conf_rst_datafield_t":
        w.append("CONF_RST rewrites every register. After this packet use wakeup_code=0x%04X in send_frame()."
                 % values["wakeup_code"])
    return w


def write_stim_line(path, P, cmd, seq, vec, head_w, crc, warnings):
    """Append '<wait_us> <wakeup_hex> <data96_hex>  # comment' to an imux_driver stimulus file."""
    wu_w = P["WAKEUP_CODE_SIZE"]
    wu_def = P.get("WAKEUP_CODE_DEF", 0)

    print()
    print("  wait before this packet")
    print("    Time between the end of the previous packet (or reset release, for the first one) and")
    print("    the start of this one. Leave room for the previous command and its ACK/DONE/data.")
    print("    Allowed: e.g. '5ms', '500us' or a plain number of microseconds")
    print("    Default: 5 ms")
    while True:
        s = ask("    > ").strip()
        try:
            wait_us = 5000.0 if s == "" else parse_us(s)
        except ValueError:
            print("    ! could not parse '%s', try again" % s)
            continue
        if wait_us < 0:
            print("    ! must be >= 0, try again")
            continue
        break

    wu = ask_value("wakeup_code", wu_w, "int",
                   "Wake-up code to send in front of this packet. Use the new code if an earlier "
                   "CONF_RST changed it.", wu_def, (1 << wu_w) - 1)

    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    lines = ["# NOTE: %s" % w for w in warnings]
    lines.append("%g %0*X %0*X   # %s seq=%d crc=%04X (gen_packet.py %s)"
                 % (wait_us, (wu_w + 3) // 4, wu, (head_w + 3) // 4, vec, cmd, seq, crc, stamp))

    new_file = not os.path.exists(path)
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    with open(path, "a", newline="\n") as f:
        if new_file:
            f.write("# imux_driver stimulus file (core/ams/imux_driver.sv)\n")
            f.write("# <wait_us> <wakeup_code_hex> <data%d_hex: {seq, slave_fwd, opcode, datafield}, no CRC>\n"
                    % head_w)
        f.write("\n".join(lines) + "\n")

    print("\n" + "\n".join(lines))
    print("\nAppended to %s" % os.path.normpath(path))


def main():
    ap = argparse.ArgumentParser(description="Interactive BIOSYNC packet generator.")
    ap.add_argument("--pkg", default=DEFAULT_PKG, help="path to default_settings_packet_structs_pkg.sv")
    ap.add_argument("--outdir", default=DEFAULT_OUTDIR, help="directory for the generated .sv file")
    ap.add_argument("--stim", metavar="FILE",
                    help="append the packet as a line to this imux_driver stimulus file (core/ams) "
                         "instead of writing an .sv file")
    args = ap.parse_args()

    params, structs, opcodes = parse_package(args.pkg)
    P = params
    seq_w, fwd_w, op_w, df_w = P["SEQ_SIZE"], P["SLAVE_FWD_SIZE"], P["OPCODE_SIZE"], P["DATAFIELD_SIZE"]
    head_w = seq_w + fwd_w + op_w + df_w
    if head_w + P["CRC_SIZE"] != P["PACKET_SIZE"]:
        sys.exit("ERROR: PACKET_SIZE != SEQ_SIZE + SLAVE_FWD_SIZE + OPCODE_SIZE + DATAFIELD_SIZE + CRC_SIZE "
                 "in the package.")
    for sname, fields in structs.items():
        total = sum(wd for _, wd in fields)
        if total != df_w:
            sys.exit("ERROR: %s is %d bits but DATAFIELD_SIZE is %d. Adjust its padding field by %+d bits "
                     "in the package." % (sname, total, df_w, df_w - total))

    print("=" * 78)
    print(" BIOSYNC packet generator")
    print(" package: %s" % os.path.normpath(args.pkg))
    print(" Press <Enter> to accept a default.  Ctrl-C to abort.")
    print("=" * 78)

    # ---- command ----
    print("\nAvailable commands:")
    for val, name in opcodes:
        print("  0x%X  %-15s %s" % (val, name[3:], COMMAND_DOC.get(name, "")))
    while True:
        s = ask("\nWhich command do you want to send? (opcode, e.g. 5 / 0xA, or name) > ").strip()
        sel = None
        for val, name in opcodes:
            if s.upper() in (name, name[3:], "%X" % val, "0X%X" % val):
                sel = (val, name)
        if sel:
            break
        print("  ! unknown command '%s'" % s)
    op_val, op_name = sel
    cmd = op_name[3:]
    print("\n--> %s (opcode 0x%X)" % (cmd, op_val))

    # ---- header ----
    answers = []  # (name, width, kind, value)
    seq = ask_value("seq", seq_w, "int",
                    "Sequence number. The radio drops a packet whose seq equals the previous one "
                    "(duplicate) and ACKs it with DUP. After reset the last seq is 0, so start at 1 "
                    "and change it for every new packet.",
                    1, (1 << seq_w) - 1)
    fwd = ask_value("slave_fwd", fwd_w, "bool",
                    "1 = forward this command to the downstream slave chip instead of executing it here.",
                    0, (1 << fwd_w) - 1)
    answers.append(("seq", seq_w, "int", seq))
    answers.append(("slave_fwd", fwd_w, "bool", fwd))
    answers.append(("opcode", op_w, "int", op_val))

    # ---- datafield ----
    sname = struct_for_opcode(op_name, structs)
    df_fields = []  # (name, width, kind, value)
    values = {}
    if sname is None:
        print("\n%s has no datafield; all %d datafield bits are 0." % (cmd, df_w))
        df_fields.append(("datafield (unused)", df_w, "int", 0))
    else:
        print("\nDatafield %s (%d bits):" % (sname, df_w))
        for fname, width in structs[sname]:
            if fname.startswith("padding"):
                df_fields.append((fname, width, "pad", 0))
                continue
            kind, doc = field_doc(sname, fname)
            if kind is None:
                kind = "bool" if width == 1 else "int"
            vmax = (1 << width) - 1
            if fname in FIELD_MAX:
                lim = FIELD_MAX[fname]
                vmax = min(vmax, P[lim] if isinstance(lim, str) else lim)
            default = field_default(fname, P) if sname == "conf_rst_datafield_t" else 0
            v = ask_value(fname, width, kind, doc, default, vmax)
            values[fname] = v
            df_fields.append((fname, width, kind, v))

    # ---- assemble ----
    all_fields = answers + df_fields
    vec, segs = 0, []
    for name, width, kind, v in all_fields:
        vec = (vec << width) | v
        segs.append(format(v, "0%db" % width))
    assert sum(wd for _, wd, _, _ in all_fields) == head_w
    crc = crc16_ccitt_false(vec, head_w)
    frame = (vec << P["CRC_SIZE"]) | crc

    warnings = sanity_checks(sname, values) if sname else []
    if fwd:
        warnings.append("slave_fwd=1: this chip forwards the command to its slave and does not run it.")

    # ---- output: imux_driver stimulus line ----
    if args.stim:
        write_stim_line(args.stim, P, cmd, seq, vec, head_w, crc, warnings)
        return

    # ---- output: .sv localparam ----
    print()
    default_name = "PKT_%s_SEQ%d" % (cmd, seq)
    pname = ask("Localparam name [%s] > " % default_name).strip() or default_name
    if not re.fullmatch(r"[A-Za-z_]\w*", pname):
        sys.exit("Invalid SystemVerilog identifier: %s" % pname)
    default_file = os.path.join(args.outdir, pname.lower() + ".sv")
    fpath = ask("Output file [%s] > " % os.path.normpath(default_file)).strip() or default_file
    os.makedirs(os.path.dirname(os.path.abspath(fpath)), exist_ok=True)
    if os.path.exists(fpath):
        if ask("%s exists. Overwrite? [y/N] > " % fpath).strip().lower() not in ("y", "yes"):
            sys.exit("Not written.")

    lines = []
    lines.append("// Generated by gen_packet.py on %s" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M"))
    lines.append("// Package : default_settings_packet_structs_pkg.sv")
    lines.append("// Command : %s (opcode 0x%X)" % (cmd, op_val))
    if op_name in COMMAND_DOC:
        lines.append("//           %s" % COMMAND_DOC[op_name])
    lines.append("//")
    lines.append("// Layout  : {seq[%d], slave_fwd[%d], opcode[%d], datafield[%d]} = %d bits (no CRC)."
                 % (seq_w, fwd_w, op_w, df_w, head_w))
    lines.append("//           Pass to make_valid_frame(), which appends the CRC-16.")
    lines.append("//")
    lines.append("//   %-28s %-10s %s" % ("field", "bits", "value"))
    bit = head_w
    for name, width, kind, v in all_fields:
        msb, lsb = bit - 1, bit - width
        bit = lsb
        rng = "[%d]" % msb if width == 1 else "[%d:%d]" % (msb, lsb)
        val = "0 (padding)" if kind == "pad" else describe_value(kind, v, width)
        lines.append("//   %-28s %-10s %s" % (name, rng, val))
    lines.append("//")
    lines.append("// hex        : %d'h%0*X" % (head_w, (head_w + 3) // 4, vec))
    lines.append("// CRC-16     : 16'h%04X (for reference; added by make_valid_frame)" % crc)
    lines.append("// full frame : %d'h%0*X" % (P["PACKET_SIZE"], (P["PACKET_SIZE"] + 3) // 4, frame))
    for wmsg in warnings:
        lines.append("// NOTE: %s" % wmsg)
    lines.append("// Underscores below separate the fields listed above.")
    lines.append("localparam logic [%d:0] %s = %d'b%s;" % (head_w - 1, pname, head_w, "_".join(segs)))

    with open(fpath, "w", newline="\n") as f:
        f.write("\n".join(lines) + "\n")

    print("\n" + "\n".join(lines))
    print("\nWritten to %s" % os.path.normpath(fpath))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        sys.exit("Aborted.")
