# Untitled - By: 李欣燚 - 周五 4月 16 2021

import sensor
import image
import lcd
import time
from fpioa_manager import fm
from machine import UART

fm.register(15,fm.fpioa.UART1_TX)
fm.register(16,fm.fpioa.UART1_RX)
uart = UART(UART.UART1, 9600, 8, None, 1, timeout=1000, read_buf_len=4096)

lcd.init()
sensor.reset()

sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames()
sensor.run(1)

green_threshold   = (0,   80,  -70,   -10,   -0,   30)
while True:
    img=sensor.snapshot()
    blobs = img.find_blobs([green_threshold],area_threshold=15,pixels_threshold=15)
    if blobs:
        for b in blobs:
            tmp=img.draw_rectangle(b[0:4])
            tmp=img.draw_cross(b[5], b[6])
            c=img.get_pixel(b[5], b[6])
            tem='%d' %b[5]
            uart.write(tem)

    lcd.display(img)
