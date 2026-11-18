"""
BIOSYNC packet API.

Builds over-the-air command frames for the BIOSYNC core and decodes the chip's ACK/DONE replies.
How the frames reach the chip (UART to the FPGA today, the radio dongle later) lives in
biosync_link.py.

Runs unchanged under CPython 3 and MicroPython (no f-strings, dataclasses, re, typing or
int.to_bytes/bit_length). All field layouts come from biosync_defs.py, which
../gen_biosync_defs.py generates from the RTL package.

Frame layout (PACKET_SIZE = 112 bits, MSB first on air, Manchester coded by the transmitter):

    {seq[2:0], slave_fwd, opcode[3:0], datafield[87:0], crc16[15:0]}

Quick start:

    from biosync_packet import Session, ms, us
    s = Session()                                  # seq starts at 1, default wake-up code
    p = s.flash(dac_conf=0x80, en_row_r="all", en_col_r="all",
                time_on=ms(500), time_off=ms(500), periods=3)
    p.hex()                    # '85...'  one line of packets.mem
    p.to_bytes()               # 14 bytes, MSB first
    p.summary()                # 'FLASH seq=1 dac_conf=0x80 ...'

Field values accept:
    int / bool            for every field
    "all" / "none"        mask fields
    [0, 2, "4-6"] / "0,2,4-6"   mask fields: bit indices
    ms(x) / us(x) / "2ms" / "500us"   time fields (converted to clk_32k cycles)
"""

import biosync_defs as D

HEAD_SIZE = D.SEQ_SIZE + D.SLAVE_FWD_SIZE + D.OPCODE_SIZE + D.DATAFIELD_SIZE   # 96, CRC input
PACKET_BYTES = D.PACKET_SIZE // 8                                                # 14
SEQ_MAX = (1 << D.SEQ_SIZE) - 1

CMD_NAMES = {}                       # opcode -> name
for _n in D.OPCODES:
    CMD_NAMES[D.OPCODES[_n]] = _n

ACK_NAMES = {
    D.ACK_STATUS_OK: "OK",
    D.ACK_STATUS_CRC: "CRC",
    D.ACK_STATUS_DUP: "DUP",
    D.ACK_STATUS_DONE: "DONE",
}


class PacketError(ValueError):
    pass


# ------------------------------------------------------------------------------------------------
# Value helpers
# ------------------------------------------------------------------------------------------------

def ms(x):
    """Milliseconds -> clk_32k cycles (rounded)."""
    return int(x * D.CLK_HZ / 1000.0 + 0.5)


def us(x):
    """Microseconds -> clk_32k cycles (rounded)."""
    return int(x * D.CLK_HZ / 1000000.0 + 0.5)


def cycles_to_us(c):
    return c * 1000000.0 / D.CLK_HZ


def mask(bits, width=None):
    """Bit-index list / "0,2,4-6" / "all" / "none" / int -> int mask. width needed for "all"."""
    if isinstance(bits, bool):
        return int(bits)
    if isinstance(bits, int):
        return bits
    if isinstance(bits, str):
        s = bits.strip().lower()
        if s == "all":
            if width is None:
                raise PacketError("'all' needs the field width")
            return (1 << width) - 1
        if s in ("none", "", "-"):
            return 0
        if s[:2] in ("0x", "0b"):
            return int(s, 0)
        bits = s.replace(" ", "").split(",")
    value = 0
    for part in bits:
        if isinstance(part, int):
            lo = hi = part
        elif "-" in part:
            a, b = part.split("-", 1)
            lo, hi = int(a), int(b)
            if lo > hi:
                lo, hi = hi, lo
        elif part == "":
            continue
        else:
            lo = hi = int(part)
        for i in range(lo, hi + 1):
            if i < 0 or (width is not None and i >= width):
                raise PacketError("bit index %d out of range 0..%d" % (i, (width or 1) - 1))
            value |= 1 << i
    return value


def _time(v):
    if isinstance(v, str):
        t = v.strip().lower().replace(" ", "")
        if t.endswith("ms"):
            return ms(float(t[:-2]))
        if t.endswith("us"):
            return us(float(t[:-2]))
        return int(t, 0)
    if isinstance(v, float):
        raise PacketError("time fields take clk_32k cycles (int); use ms()/us() for real time")
    return int(v)


def _coerce(kind, width, v):
    if kind == "mask":
        return mask(v, width)
    if kind == "time":
        return _time(v)
    if isinstance(v, str):
        s = v.strip().lower()
        if kind == "bool" and s in ("y", "yes", "true", "n", "no", "false"):
            return 1 if s[0] in "yt" else 0
        return int(s, 0)
    return int(v)


def _indices(value, width):
    return [i for i in range(width) if (value >> i) & 1]


# ------------------------------------------------------------------------------------------------
# CRC / bit helpers
# ------------------------------------------------------------------------------------------------

def crc16(value, nbits=HEAD_SIZE, poly=0x1021, init=0xFFFF):
    """CRC-16/CCITT-FALSE, bitwise MSB first: same as rx_crc.sv and tb_helpers_pkg."""
    crc = init
    for i in range(nbits - 1, -1, -1):
        top = (crc >> 15) & 1
        crc = (crc << 1) & 0xFFFF
        if top ^ ((value >> i) & 1):
            crc ^= poly
    return crc


def int_to_bytes(value, nbytes):
    """Big-endian, works for any int size under MicroPython too."""
    return bytes([(value >> (8 * i)) & 0xFF for i in range(nbytes - 1, -1, -1)])


def bytes_to_int(b):
    v = 0
    for x in b:
        v = (v << 8) | x
    return v


def to_hex(value, ndigits):
    s = "%X" % value
    if len(s) > ndigits:
        raise PacketError("value does not fit in %d hex digits" % ndigits)
    return "0" * (ndigits - len(s)) + s


def to_bin(value, nbits):
    s = bin(value)[2:]
    return "0" * (nbits - len(s)) + s


# ------------------------------------------------------------------------------------------------
# Packet
# ------------------------------------------------------------------------------------------------

class Packet:
    """One command frame. Immutable after construction; build through Session or Packet(...)."""

    def __init__(self, cmd, seq=1, slave_fwd=0, crc=None, strict=True, **fields):
        cmd = cmd.upper()
        if cmd.startswith("OP_"):
            cmd = cmd[3:]
        if cmd not in D.OPCODES:
            raise PacketError("unknown command %r (known: %s)" % (cmd, ", ".join(sorted(D.OPCODES))))
        if not 0 <= seq <= SEQ_MAX:
            raise PacketError("seq %d out of range 0..%d" % (seq, SEQ_MAX))
        if slave_fwd not in (0, 1, True, False):
            raise PacketError("slave_fwd must be 0 or 1")
        self.cmd = cmd
        self.opcode = D.OPCODES[cmd]
        self.seq = seq
        self.slave_fwd = int(slave_fwd)
        self.layout = D.DATAFIELDS.get(cmd, ())
        self.fields = self._resolve(fields, strict)
        self.datafield = self._pack_datafield()
        self.head = (((((seq << D.SLAVE_FWD_SIZE) | self.slave_fwd) << D.OPCODE_SIZE) | self.opcode)
                     << D.DATAFIELD_SIZE) | self.datafield
        self.crc_ok = crc16(self.head)
        self.crc = self.crc_ok if crc is None else crc
        self.frame = (self.head << D.CRC_SIZE) | self.crc

    def _resolve(self, given, strict=True):
        names = [f[0] for f in self.layout if f[2] != "pad"]
        unknown = [k for k in given if k not in names]
        if unknown:
            if not self.layout:
                raise PacketError("%s has no datafield, got %s" % (self.cmd, ", ".join(unknown)))
            raise PacketError("%s: unknown field(s) %s; valid: %s"
                              % (self.cmd, ", ".join(unknown), ", ".join(names)))
        out = {}
        for name, width, kind, vmax in self.layout:
            if kind == "pad":
                continue
            if name in given:
                try:
                    v = _coerce(kind, width, given[name])
                except (ValueError, TypeError) as e:
                    raise PacketError("%s.%s: %s" % (self.cmd, name, e))
            elif self.cmd == "CONF_RST":
                v = D.CONF_RST_DEFAULTS[name]
            else:
                v = 0
            if not 0 <= v <= (vmax if strict else (1 << width) - 1):
                raise PacketError("%s.%s = %d out of range 0..%d" % (self.cmd, name, v, vmax))
            out[name] = v
        return out

    def _pack_datafield(self):
        v, total = 0, 0
        for name, width, kind, _ in self.layout:
            v = (v << width) | (0 if kind == "pad" else self.fields[name])
            total += width
        if self.layout and total != D.DATAFIELD_SIZE:
            raise PacketError("layout of %s is %d bits, expected %d" % (self.cmd, total, D.DATAFIELD_SIZE))
        return v

    # ---- encodings ----
    def hex(self):
        """28 hex digits: one line of packets.mem ($readmemh in packet_tx.sv)."""
        return to_hex(self.frame, D.PACKET_SIZE // 4)

    def head_hex(self):
        """24 hex digits without CRC: data96 of make_valid_frame / imux_driver stimulus."""
        return to_hex(self.head, HEAD_SIZE // 4)

    def to_bytes(self):
        return int_to_bytes(self.frame, PACKET_BYTES)

    def bits(self):
        return to_bin(self.frame, D.PACKET_SIZE)

    def sv_literal(self):
        """96-bit SV literal with '_' between fields, like gen_packet.py writes."""
        segs = [to_bin(self.seq, D.SEQ_SIZE), to_bin(self.slave_fwd, D.SLAVE_FWD_SIZE),
                to_bin(self.opcode, D.OPCODE_SIZE)]
        if self.layout:
            for name, width, kind, _ in self.layout:
                segs.append(to_bin(0 if kind == "pad" else self.fields[name], width))
        else:
            segs.append(to_bin(0, D.DATAFIELD_SIZE))
        return "%d'b%s" % (HEAD_SIZE, "_".join(segs))

    # ---- variants ----
    def corrupted(self, bit=0):
        """Same packet with one frame bit flipped (bit 0 = CRC LSB). The chip must ACK with CRC."""
        return Packet.decode(self.frame ^ (1 << bit), check_crc=False)

    def with_seq(self, seq):
        return Packet(self.cmd, seq, self.slave_fwd, None, False, **self.fields)

    # ---- decoding ----
    @staticmethod
    def decode(frame, check_crc=True):
        """112-bit int / 14 bytes / 28-hex string -> Packet."""
        if isinstance(frame, (bytes, bytearray)):
            frame = bytes_to_int(frame)
        elif isinstance(frame, str):
            frame = int(frame.strip(), 16)
        crc = frame & ((1 << D.CRC_SIZE) - 1)
        head = frame >> D.CRC_SIZE
        df = head & ((1 << D.DATAFIELD_SIZE) - 1)
        op = (head >> D.DATAFIELD_SIZE) & ((1 << D.OPCODE_SIZE) - 1)
        fwd = (head >> (D.DATAFIELD_SIZE + D.OPCODE_SIZE)) & 1
        seq = head >> (D.DATAFIELD_SIZE + D.OPCODE_SIZE + D.SLAVE_FWD_SIZE)
        if op not in CMD_NAMES:
            raise PacketError("unknown opcode 0x%X" % op)
        cmd = CMD_NAMES[op]
        fields = {}
        pos = D.DATAFIELD_SIZE
        for name, width, kind, _ in D.DATAFIELDS.get(cmd, ()):
            pos -= width
            if kind != "pad":
                fields[name] = (df >> pos) & ((1 << width) - 1)
        if check_crc and crc != crc16(head):
            raise PacketError("CRC mismatch: frame has %04X, expected %04X" % (crc, crc16(head)))
        # strict=False: foreign frames may hold values above the documented maximum (stride > SPAD_M)
        return Packet(cmd, seq, fwd, crc, False, **fields)

    @property
    def crc_valid(self):
        return self.crc == self.crc_ok

    # ---- human readable ----
    def warnings(self):
        """Legal but probably unintended settings (ported from gen_packet.sanity_checks)."""
        f, w = self.fields, []
        if self.cmd == "ESTIM":
            if f["en_row_a"] & f["en_row_c"] and f["en_col_a"] & f["en_col_c"]:
                w.append("an electrode is selected as both anode and cathode -> possible short")
            if not (f["en_row_a"] and f["en_col_a"]):
                w.append("no anode selected")
            if not (f["en_row_c"] and f["en_col_c"]):
                w.append("no cathode selected")
            if f["amp_c"] == 0 or f["amp_a"] == 0:
                w.append("amp_c and/or amp_a is 0")
        elif self.cmd == "FLASH":
            red = (f["en_row_r"] and f["en_col_r"]) or (f["en_row_r_l"] and f["en_col_r_l"])
            blue = (f["en_row_b"] and f["en_col_b"]) or (f["en_row_b_l"] and f["en_col_b_l"])
            if not (red or blue):
                w.append("no LED selected (needs a row AND a column bit of the same group)")
            if f["time_on"] == 0:
                w.append("time_on is 0")
        elif self.cmd == "REC_SPAD":
            if f["time_rec"] == 0:
                w.append("time_rec is 0")
        elif self.cmd == "SETUP_REC_SPAD":
            if not (f["en_row_spad"] and f["en_col_spad"]):
                w.append("no SPAD macro selected")
        elif self.cmd == "READ_SPAD":
            if not (f["row_sel"] and f["col_sel"]):
                w.append("no SPAD macro selected")
        elif self.cmd == "CONF_RST" and f["wakeup_code"] != D.WAKEUP_CODE_DEF:
            w.append("wakeup_code changes to 0x%04X: later frames must use it" % f["wakeup_code"])
        if self.slave_fwd:
            w.append("slave_fwd=1: forwarded to the slave chip, not executed here")
        if not self.crc_valid:
            w.append("CRC is deliberately wrong (%04X, valid %04X)" % (self.crc, self.crc_ok))
        return w

    def summary(self):
        """One line, e.g. 'FLASH seq=4 dac_conf=0x80 en_row_r=all time_on=16384(500ms) ...'."""
        parts = ["%s seq=%d" % (self.cmd, self.seq)]
        if self.slave_fwd:
            parts.append("fwd")
        for name, width, kind, _ in self.layout:
            if kind == "pad":
                continue
            v = self.fields[name]
            if kind == "mask":
                idx = _indices(v, width)
                s = "all" if len(idx) == width else ("none" if not idx else ",".join([str(i) for i in idx]))
            elif kind == "time":
                t = cycles_to_us(v)
                s = "%d(%.3gms)" % (v, t / 1000.0) if t >= 1000 else "%d(%.3gus)" % (v, t)
            elif kind == "bool":
                s = str(v)
            else:
                s = "0x%X" % v if v > 9 else str(v)
            parts.append("%s=%s" % (name, s))
        if not self.crc_valid:
            parts.append("BADCRC")
        return " ".join(parts)

    def __repr__(self):
        return "<Packet %s %s>" % (self.hex(), self.summary())


# ------------------------------------------------------------------------------------------------
# Session: sequence numbers + wake-up code tracking, one method per command
# ------------------------------------------------------------------------------------------------

class Session:
    """
    Keeps the state the chip keeps about the link, so every packet you build is accepted:
      - seq: the radio drops a frame whose seq equals the previous one (ACK DUP). After reset the
        chip's last seq is 0, so the first frame uses seq 1; seq then increments and wraps.
      - wakeup_code: a CONF_RST that changes it takes effect for every LATER frame.
    `history` lists every packet built, in order.
    """

    def __init__(self, first_seq=1, wakeup_code=D.WAKEUP_CODE_DEF):
        self.next_seq = first_seq
        self.wakeup_code = wakeup_code
        self.history = []

    def reset(self, first_seq=1, wakeup_code=D.WAKEUP_CODE_DEF):
        """Call after a chip POR: seq memory and wake-up code return to reset values."""
        self.next_seq = first_seq
        self.wakeup_code = wakeup_code

    def build(self, cmd, slave_fwd=0, seq=None, corrupt_bit=None, **fields):
        """corrupt_bit=n flips frame bit n after the CRC is computed (CRC-error test frame).
        The seq is consumed either way, so the next frame can never be mistaken for a DUP."""
        if seq is None:
            seq = self.next_seq
        p = Packet(cmd, seq, slave_fwd, **fields)
        if corrupt_bit is not None:
            p = p.corrupted(corrupt_bit)
        p.wakeup_code = self.wakeup_code          # code this frame must be sent with
        self.next_seq = (seq + 1) & SEQ_MAX
        if p.cmd == "CONF_RST" and p.crc_valid:
            self.wakeup_code = p.fields["wakeup_code"]
        self.history.append(p)
        return p

    def add(self, packet):
        """Append a hand-made packet (e.g. packet.corrupted()) without changing the sequence."""
        if not hasattr(packet, "wakeup_code"):
            packet.wakeup_code = self.wakeup_code
        self.history.append(packet)
        return packet

    # ---- one method per command ----
    def halt(self, **kw):
        return self.build("HALT", **kw)

    def conf_rst(self, **kw):
        return self.build("CONF_RST", **kw)

    def poll(self, **kw):
        return self.build("POLL", **kw)

    def run_fll(self, **kw):
        return self.build("RUN_FLL", **kw)

    def flash(self, **kw):
        return self.build("FLASH", **kw)

    def estim(self, **kw):
        return self.build("ESTIM", **kw)

    def setup_rec_spad(self, **kw):
        return self.build("SETUP_REC_SPAD", **kw)

    def rec_spad(self, **kw):
        return self.build("REC_SPAD", **kw)

    def read_spad(self, **kw):
        return self.build("READ_SPAD", **kw)

    def mem_lines(self):
        """packets.mem lines for packet_tx.sv, one per packet built so far."""
        return [p.hex() for p in self.history]


# ------------------------------------------------------------------------------------------------
# Chip -> base station messages
# ------------------------------------------------------------------------------------------------

def decode_ack(value):
    """23-bit ACK/DONE {wakeup_code[15:0], seq[2:0], status[3:0]} (int or '0101...' string)."""
    if isinstance(value, str):
        value = int(value, 2)
    status = value & ((1 << D.ACK_STATUS_SIZE) - 1)
    seq = (value >> D.ACK_STATUS_SIZE) & SEQ_MAX
    wu = value >> (D.ACK_STATUS_SIZE + D.SEQ_SIZE)
    return {"wakeup_code": wu, "seq": seq, "status": ACK_NAMES.get(status, "0x%X" % status)}


def expected_ack(packet, status=D.ACK_STATUS_OK, wakeup_code=None):
    """The 23-bit ACK the chip should send for `packet`, as a bit string (MSB first)."""
    wu = getattr(packet, "wakeup_code", D.WAKEUP_CODE_DEF) if wakeup_code is None else wakeup_code
    v = (((wu << D.SEQ_SIZE) | packet.seq) << D.ACK_STATUS_SIZE) | status
    return to_bin(v, D.ACK_SIZE)
