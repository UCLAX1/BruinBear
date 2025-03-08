import board
import datetime
import digitalio
import os
from PIL import Image, ImageDraw, ImageFont
import subprocess
import time
import adafruit_rgb_display.ili9341 as ili9341
import adafruit_rgb_display.st7789 as st7789

BAUDRATE = 24000000
spi = board.SPI()
cs_pin = digitalio.DigitalInOut(board.CE0)
dc_pin = digitalio.DigitalInOut(board.D25)
reset_pin = digitalio.DigitalInOut(board.D24)
disp = st7789.ST7789(spi, rotation=90, cs=cs_pin, dc=dc_pin, rst=reset_pin, baudrate=BAUDRATE)

#Draw stuff
draw.rectangle((0, 0, width, height), outline=0, fill="#FFFFFF")
disp.image(original)

time.sleep(0.02)

#Next frame