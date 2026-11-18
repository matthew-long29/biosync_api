"""
BIOSYNC UART link: Raspberry Pi Pico <-> FPGA (fpga/rtl/uart_link.sv).

The Pico sends command frames; the FPGA radiates them into the core (packet_tx.sv) and streams
every bit the core transmits back to the Pico. This module prints those replies as hex and
decodes ACK / DONE / POLL data.

    >>> link = Link(open_uart())
    >>> link.ping()
    >>> link.send(s.poll())          # s = biosync_packet.Session()

UART: 1,000,000 baud 8N1. The FPGA runs everything (core, packet_tx, uart_link) on one 10 MHz
MMCM clock, and uart_link uses CLKS_PER_BIT = 10 (exact); the Pico's UART divider hits 1 Mbaud
exactly too. The core therefore runs ~305x faster than the 32.768 kHz chip: command time fields
are clock cycles, so on the FPGA they last 305x less than ms()/us() suggest.

Wiring (3.3 V): Pico GP0 (TX) -> FPGA uart_rxd, Pico GP1 (RX) <- FPGA uart_txd, GND -> GND.

PROTOCOL
    Pico -> FPGA
        0x00                                PING
        0x01 wakeup[15:0] frame[111:0]      SEND  (17 bytes, MSB first)
    FPGA -> Pico
        0xA0 accepted   0xA1 busy (frame dropped)   0xA2 pong   0xA3 unknown command
        0xA4 frame finished radiating (sent)
        0x00-0x7F       7 transmitted bits, first bit in bit 6; the first data byte after an
                        END starts a new TX window (rx_en low)
        0x90|{o,v}      TX window closed; the last data byte holds v (1..7) valid bits in its
                        MSBs; o = 1: the FPGA FIFO overflowed, bits are missing
"""

import biosync_defs as D
import biosync_packet as bp

try:
    from time import ticks_ms, ticks_diff, sleep_ms          # MicroPython
except ImportError:                                           # CPython (desktop testing)
    import time as _t

    def ticks_ms():
        return int(_t.monotonic() * 1000)

    def ticks_diff(a, b):
        return a - b

    def sleep_ms(n):
        _t.sleep(n / 1000.0)


# ---- configuration: must match core_fpga_top.sv / uart_link.sv parameters ----
CORE_CLK_HZ = 10000000                      # MMCM output: core, packet_tx and uart_link
CLKS_PER_BIT = 10
BAUD = CORE_CLK_HZ // CLKS_PER_BIT          # 1,000,000
UART_ID, TX_PIN, RX_PIN = 0, 0, 1
CHIP_MS = 4 * 1000.0 / (CORE_CLK_HZ / 8)    # packet_tx: 4 periods of clk_sample_4k per chip
FRAME_CHIPS = 2 + D.WAKEUP_CODE_SIZE + 2 * D.PACKET_SIZE + 3   # LEAD + wake-up + Manchester + TRAIL

CMD_PING, CMD_SEND = 0x00, 0x01
WIN_END = 0x90
REPLIES = {0xA0: "ACCEPTED", 0xA1: "BUSY", 0xA2: "PONG", 0xA3: "BADCMD", 0xA4: "SENT"}


def open_uart(baud=BAUD):
    """MicroPython only: UART0 on GP0/GP1."""
    from machine import UART, Pin
    return UART(UART_ID, baudrate=baud, bits=8, parity=None, stop=1,
                tx=Pin(TX_PIN), rx=Pin(RX_PIN), rxbuf=8192, timeout=0)


def send_bytes(packet, wakeup_code=None):
    """The 17-byte SEND command for one packet."""
    if wakeup_code is None:
        wakeup_code = getattr(packet, "wakeup_code", D.WAKEUP_CODE_DEF)
    return (bytes([CMD_SEND]) + bp.int_to_bytes(wakeup_code, D.WAKEUP_CODE_SIZE // 8)
            + bp.int_to_bytes(packet.frame, bp.PACKET_BYTES))


# ------------------------------------------------------------------------------------------------
# What the core transmitted
# ------------------------------------------------------------------------------------------------

# POLL payload after the 20-bit preamble (poll_message in core.sv), MSB first
POLL_FIELDS = (("battery_voltage", 4), ("wpt_active_n", 1)) + tuple(
    (n, w) for n, w, k, _ in D.DATAFIELDS["CONF_RST"] if k != "pad") + (
    ("spad_row_sel", 27), ("spad_col_sel", 35))
POLL_BITS = sum([w for _, w in POLL_FIELDS])


SHOW_BITS = 512                             # longer windows print as a truncated hex preview
NO_DONE = ("CONF_RST", "SETUP_REC_SPAD")    # finish in one cycle: the radio never sends DONE


class Window:
    """One TX window (or part of one): bits the core radiated while rx_en was low.

    Kept as the FPGA's raw 7-bit groups (1 byte of RAM per 7 bits), so a long SPAD readout fits
    in the Pico's RAM. get(pos, count) returns any slice as an int; `bits` / hex() build text
    only for windows up to SHOW_BITS long.
    """

    def __init__(self, raw, start, n, overflow=False):
        self.raw, self.start, self.n, self.overflow = raw, start, n, overflow
        self.kind = "RAW"
        self.ack = None
        self.poll = None
        if n == D.ACK_SIZE:
            self.ack = bp.decode_ack(self.get(0, n))
            self.kind = "DONE" if self.ack["status"] == "DONE" else "ACK"
        elif (n > D.DATA_PREAMBLE_SIZE and
              self.get(D.WAKEUP_CODE_SIZE, D.DATA_PREAMBLE_STATUS_SIZE) == D.DATA_PREAMBLE_STATUS):
            self.kind = "DATA"
            if n - D.DATA_PREAMBLE_SIZE == POLL_BITS:
                self.poll, pos = {}, D.DATA_PREAMBLE_SIZE
                for name, w in POLL_FIELDS:
                    self.poll[name] = self.get(pos, w)
                    pos += w

    def get(self, pos, count):
        """Bits [pos, pos+count) of this window as an int, first bit = MSB."""
        v, q = 0, self.start + pos
        raw = self.raw
        for _ in range(count):
            v = (v << 1) | ((raw[q // 7] >> (6 - q % 7)) & 1)
            q += 1
        return v

    @property
    def payload_bits(self):
        """DATA: number of bits after the 20-bit preamble."""
        return self.n - D.DATA_PREAMBLE_SIZE if self.kind == "DATA" else 0

    @property
    def bits(self):
        """'0'/'1' string (None for windows longer than SHOW_BITS)."""
        return bp.to_bin(self.get(0, self.n), self.n) if 0 < self.n <= SHOW_BITS else ("" if not self.n else None)

    def hex(self):
        if not self.n:
            return ""
        k = min(self.n, SHOW_BITS)
        h = bp.to_hex(self.get(0, k), (k + 3) // 4)
        return h + ("..." if self.n > SHOW_BITS else "")

    def describe(self):
        s = "%-4s %4d bits  0x%s" % (self.kind, self.n, self.hex())
        if self.ack is not None:
            s += "  seq=%d %s" % (self.ack["seq"], self.ack["status"])
        if self.overflow:
            s += "  [FPGA FIFO OVERFLOW: bits missing]"
        if self.poll is not None:
            for name, _ in POLL_FIELDS:
                s += "\n         %-26s 0x%X" % (name, self.poll[name])
        return s

    def __repr__(self):
        return "<Window %s>" % self.describe().split("\n")[0]


def split_window(raw, n, overflow, wakeup_code):
    """The core can append DONE to whatever it was transmitting without raising rx_en (POLL
    data + DONE, FLL calibration carrier + DONE), so one TX window may hold both. Split off a
    trailing 23-bit DONE that carries the current wake-up code."""
    if n > D.ACK_SIZE and not overflow:
        tail = Window(raw, n - D.ACK_SIZE, D.ACK_SIZE)
        if tail.kind == "DONE" and tail.ack["wakeup_code"] == wakeup_code:
            return [Window(raw, 0, n - D.ACK_SIZE), tail]
    return [Window(raw, 0, n, overflow)]


class Parser:
    """FPGA -> Pico byte stream -> Window objects and reply names ('ACCEPTED', 'SENT', ...)."""

    def __init__(self):
        self.buf = bytearray()              # 7-bit groups of the window being received
        self.wakeup_code = D.WAKEUP_CODE_DEF

    def feed(self, data):
        out = []
        for b in data:
            if b < 0x80:
                self.buf.append(b)
            elif b & 0xF0 == WIN_END:
                if self.buf:
                    n = 7 * len(self.buf) - 7 + (b & 0x07)
                    out.extend(split_window(self.buf, n, bool(b & 0x08), self.wakeup_code))
                else:
                    out.append(Window(b"", 0, 0, True))     # all data bytes were dropped
                self.buf = bytearray()
            elif b in REPLIES:
                out.append(REPLIES[b])
            else:
                out.append("STRAY 0x%02X" % b)
        return out


# ------------------------------------------------------------------------------------------------
# Expected timing
# ------------------------------------------------------------------------------------------------

def reply_bits(p):
    """Rough number of bits the core sends back for packet p (ACK + DATA + DONE)."""
    n = 2 * D.ACK_SIZE
    if p.cmd == "POLL":
        n += D.DATA_PREAMBLE_SIZE + POLL_BITS
    elif p.cmd == "READ_SPAD":
        f = p.fields
        macros = (len(bp._indices(f["row_sel"], D.SPAD_ROWS_MACRO))
                  * len(bp._indices(f["col_sel"], D.SPAD_COLS_MACRO)))
        stride = max(f["stride"], 1)
        per_side = (D.SPAD_M + stride - 1) // stride
        n += D.DATA_PREAMBLE_SIZE + macros * per_side * per_side * D.SPAD_BITS
    return n


def command_ms(p):
    """Rough run time of the command itself, in ms."""
    f, c = p.fields, 0
    if p.cmd == "FLASH":
        c = (f["periods"] + 1) * (f["time_on"] + f["time_off"])
    elif p.cmd == "ESTIM":
        c = (f["periods"] + 1) * (f["time_c"] + f["time_int"] + f["time_a"] + f["time_gnd"])
    elif p.cmd == "REC_SPAD":
        c = f["time_rec"] + f["delay_led"] + f["delay_rec"]
    elif p.cmd == "RUN_FLL":
        c = CORE_CLK_HZ // 10
    return c * 1000 // CORE_CLK_HZ


def response_timeout_ms(p):
    """After SENT: command time + time for the core to radiate the reply + UART drain + margin."""
    bits = reply_bits(p)
    radiate = bits * 1000 // CORE_CLK_HZ
    drain = (bits // 7 + 6) * 10 * 1000 // BAUD
    return command_ms(p) + radiate + drain + 1000


# ------------------------------------------------------------------------------------------------
# Link
# ------------------------------------------------------------------------------------------------

class Response:
    """Everything that came back for one send()."""

    def __init__(self, packet):
        self.packet = packet
        self.accepted = False
        self.sent = False
        self.windows = []
        self.problems = []

    def _first(self, kind):
        for w in self.windows:
            if w.kind == kind:
                return w
        return None

    @property
    def ack(self):
        return self._first("ACK")

    @property
    def done(self):
        return self._first("DONE")

    @property
    def data(self):
        return [w for w in self.windows if w.kind == "DATA"]

    @property
    def ok(self):
        return not self.problems

    def report(self):
        p = self.packet
        lines = ["%s  ->  %s" % (p.summary(), p.hex())]
        for w in self.windows:
            lines.append("  " + w.describe())
        for msg in self.problems:
            lines.append("  ! " + msg)
        lines.append("  " + ("PASS" if self.ok else "FAIL"))
        return "\n".join(lines)

    def check(self):
        p, ack = self.packet, self.ack
        if not self.accepted or not self.windows:
            return                                  # problem already recorded by Link.send
        if ack is None:
            if p.cmd != "HALT":                     # HALT is answered with DONE only
                self.problems.append("no ACK")
            return
        expect = "OK" if p.crc_valid else "CRC"
        if ack.ack["status"] != expect:
            self.problems.append("ACK status %s, expected %s" % (ack.ack["status"], expect))
        if p.crc_valid and ack.ack["seq"] != p.seq:
            self.problems.append("ACK seq %d, expected %d" % (ack.ack["seq"], p.seq))
        if ack.ack["status"] == "OK" and p.cmd not in NO_DONE and self.done is None:
            self.problems.append("no DONE")
        for w in self.windows:
            if w.overflow:
                self.problems.append("FPGA FIFO overflow: reply incomplete")
                break


class Link:
    """`uart` needs write(bytes), read(n) and any() (machine.UART, or a pyserial wrapper)."""

    def __init__(self, uart, verbose=False):
        self.uart = uart
        self.parser = Parser()
        self.queue = []
        self.verbose = verbose              # True: also print the raw bytes received

    def _pump(self):
        n = self.uart.any()
        data = self.uart.read(n) if n else None
        if data:
            if self.verbose:
                print("  <- " + " ".join(["%02X" % b for b in data]))
            self.queue.extend(self.parser.feed(data))

    def _next(self, timeout_ms):
        t0 = ticks_ms()
        while True:
            self._pump()
            if self.queue:
                return self.queue.pop(0)
            if ticks_diff(ticks_ms(), t0) >= timeout_ms:
                return None
            sleep_ms(2)

    def flush(self):
        """Drop anything already received (reported if it was a TX window)."""
        self._pump()
        for item in self.queue:
            if isinstance(item, Window):
                print("  (earlier) " + item.describe())
        self.queue = []

    def ping(self, timeout_ms=200):
        """True if the FPGA answers."""
        self.flush()
        self.uart.write(bytes([CMD_PING]))
        item = self._next(timeout_ms)
        while item is not None:
            if item == "PONG":
                print("FPGA answered PING (%d baud)" % BAUD)
                return True
            item = self._next(timeout_ms)
        print("No PONG: check GP0->uart_rxd / GP1<-uart_txd / GND, the bitstream and the baud rate (%d)" % BAUD)
        return False

    def send(self, packet, show=True):
        """Send one packet and collect the core's reply windows. Returns a Response."""
        r = Response(packet)
        self.flush()
        self.parser.wakeup_code = getattr(packet, "wakeup_code", D.WAKEUP_CODE_DEF)
        self.uart.write(send_bytes(packet))

        # 1. FPGA receives the 17 bytes (0.17 ms at 1 Mbaud) and answers ACCEPTED / BUSY
        item = self._next(17 * 10 * 1000 // BAUD + 200)
        if item != "ACCEPTED":
            r.problems.append("FPGA did not accept the frame (%s)" %
                              (item or "no answer: check wiring / bitstream / baud"))
            return self._finish(r, show)
        r.accepted = True

        # 2. packet_tx radiates the frame, then SENT; 3. the core replies. The core may start its
        #    ACK before packet_tx finishes the trailing chips, so an ACK also proves "sent".
        t0 = ticks_ms()
        deadline = FRAME_CHIPS * CHIP_MS + 500
        while True:
            left = deadline - ticks_diff(ticks_ms(), t0)
            item = self._next(max(int(left), 0))
            if item is None:
                break
            if item == "SENT" or (isinstance(item, Window) and not r.sent):
                if not r.sent:
                    r.sent = True
                    t0, deadline = ticks_ms(), response_timeout_ms(packet)
                if item == "SENT":
                    continue
            if isinstance(item, Window):
                r.windows.append(item)
                if item.kind == "DONE":
                    break
                if item.kind == "ACK" and (item.ack["status"] != "OK" or packet.cmd in NO_DONE):
                    break
            else:
                r.problems.append("unexpected " + item)

        if not r.sent:
            r.problems.append("FPGA never reported the frame as sent")
        elif not r.windows:
            r.problems.append("NO RESPONSE: the core did not transmit within %d ms" % deadline)
        r.check()
        return self._finish(r, show)

    def _finish(self, r, show):
        if show:
            print(r.report())
        return r

    def run(self, packets, gap_ms=100):
        """send() each packet in order; prints a summary. Returns the Responses."""
        out = []
        for p in packets:
            out.append(self.send(p))
            sleep_ms(gap_ms)
        print("%d / %d passed" % (len([r for r in out if r.ok]), len(out)))
        return out

    def listen(self, seconds=5):
        """Print every TX window that arrives (e.g. frames started from packets.mem by a button)."""
        t0 = ticks_ms()
        while ticks_diff(ticks_ms(), t0) < seconds * 1000:
            item = self._next(50)
            if isinstance(item, Window):
                print(item.describe())
            elif item is not None:
                print(item)
