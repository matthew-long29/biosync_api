"""
BIOSYNC Pico console. Runs at power-up; afterwards these are ready in the REPL:

    ping()                      is the FPGA there?
    send(s.poll())              send one command, print the core's reply in hex (+ decoded)
    send(s.flash(dac_conf=0x80, en_row_r="all", en_col_r="all", time_on=ms(100), periods=3))
    run("smoke")                every packet of sequences.smoke, with PASS/FAIL per packet
    listen(10)                  print whatever the core transmits for 10 s
    new_session()               after resetting the core: seq back to 1, default wake-up code

    s        biosync_packet.Session (one method per command; tracks seq + wake-up code)
    link     biosync_link.Link      (link.verbose = True also prints the raw UART bytes)
"""

import biosync_link as L
import sequences
from biosync_packet import Session, ms, us

link = L.Link(L.open_uart())
s = Session()


def ping():
    return link.ping()


def send(packet):
    return link.send(packet)


def run(seq="smoke", gap_ms=100):
    """Build a sequence (name or function from sequences.py) on `s` and send it."""
    fn = getattr(sequences, seq) if isinstance(seq, str) else seq
    first = len(s.history)
    fn(s)
    return link.run(s.history[first:], gap_ms)


def listen(seconds=5):
    link.listen(seconds)


def new_session():
    global s
    s = Session()
    return s


print("BIOSYNC console: UART%d GP%d/GP%d @ %d baud. Try ping(), send(s.poll()), run('smoke')."
      % (L.UART_ID, L.TX_PIN, L.RX_PIN, L.BAUD))
