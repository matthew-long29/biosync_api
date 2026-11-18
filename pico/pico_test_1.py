import time, biosync_link as L
from biosync_packet import ms, us
from biosync_packet import Session

# Scope test: send the same ESTIM frame over and over (no waiting for replies).
uart = L.open_uart()
p = Session().estim(cp_freq=0x8, en_row_c=[0], en_col_c=[0], en_row_a=[1], en_col_a=[1],
            time_c=ms(2), amp_c=0x40, time_int=us(500), time_a=ms(2), amp_a=0x40,
            time_gnd=ms(4), periods=9)
while True:
    uart.write(L.send_bytes(p)); time.sleep_ms(100)
