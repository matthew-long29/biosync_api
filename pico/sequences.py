"""
Named packet sequences. Each function takes a biosync_packet.Session and builds packets on it.

    On the Pico:   link.run(sequences.smoke)        sends them one by one and prints the replies
    On the PC:     python build_packets.py build smoke   writes them to packets.mem instead

Add your own sequence by writing another function here.
"""

from biosync_packet import ms, us


def smoke(s):
    """Every implemented command once, plus one CRC-error frame.

    Packets 0-2 match fpga/scripts/test_generator.py, so expected_results.txt still describes the
    chip's replies to them. The wake-up code is never changed.
    """
    # 0: CONF_RST seq 1 (same register values as test_generator.py) -> ACK only
    s.conf_rst(pmic_fast_f=0x5, pmic_slow_f=0x2, rx_bw=0x1, rx_error_thresh=3, aoc_zt=300, aoc_ot=2,
               override_aoc=0, tx_bias=0xA, tx_disable_bias_after_bit=0, fll_cal_word=0x1234,
               wpt_dac_conf=0x5A)

    # 1: POLL seq 2 -> ACK, DATA (status), DONE
    s.poll()

    # 2: POLL seq 3 with the CRC LSB flipped -> ACK CRC, not executed
    s.poll(corrupt_bit=0)

    # 3: RUN_FLL seq 4
    s.run_fll()

    # 4: FLASH seq 5: all small red LEDs, 100 ms on / 100 ms off, 4 flashes
    s.flash(dac_conf=0x80, en_row_r="all", en_col_r="all",
            time_on=ms(100), time_off=ms(100), periods=3)

    # 5: ESTIM seq 6: cathode (r0,c0), anode (r1,c1); 2 ms / 0.5 ms / 2 ms / 4 ms, 10 pulses
    s.estim(cp_freq=0x8, en_row_c=[0], en_col_c=[0], en_row_a=[1], en_col_a=[1],
            time_c=ms(2), amp_c=0x40, time_int=us(500), time_a=ms(2), amp_a=0x40,
            time_gnd=ms(4), periods=9)

    # 6: SETUP_REC_SPAD seq 7: macro row 0, macro columns 0-1
    s.setup_rec_spad(en_row_spad=[0], en_col_spad="0-1")

    # 7: REC_SPAD seq 0: reset counters, blue LEDs on, record 10 ms, keep the counts
    s.rec_spad(disable_counters=0, rst_before_rec=1, led_dac_conf=0x80,
               en_row_b="all", en_col_b="all", stride=1, time_rec=ms(10),
               delay_led=4, delay_rec=4, cp_v_conf=0x20, cp_freq_conf=0x10)

    # 8: READ_SPAD seq 1: read what was recorded, then turn the counter LDO off
    s.read_spad(disable_counters=1, row_sel=[0], col_sel="0-1", stride=1)


def basic(s):
    """Minimal link check: CONF_RST then POLL."""
    s.conf_rst()
    s.poll()

