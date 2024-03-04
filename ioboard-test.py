# test stepper board
from machine import Pin
import time

# RPi Pico builtin LED on GPIO 25. Turn it on at start.
ledPico = Pin(25, mode=Pin.OUT, value=1)

# manual I/O interface board
# Buttons
buttonA = Pin(22, Pin.IN, Pin.PULL_UP)
buttonB = Pin(21, Pin.IN, Pin.PULL_UP)
buttonC = Pin(20, Pin.IN, Pin.PULL_UP)
buttonRED = Pin(19, Pin.IN, Pin.PULL_UP)
# LEDs. Turn off at start
ledA = Pin(18, Pin.OUT, value=0)
ledB = Pin(16, Pin.OUT, value=0)
ledC = Pin(17, Pin.OUT, value=0)
ledD = Pin(15, Pin.OUT, value=0)

# MERG CBUS I/O
# CANACE8C
canACE8C1 = Pin(10, Pin.OUT)
canACE8C2 = Pin(11, Pin.OUT)
canACE8C3 = Pin(12, Pin.OUT)
canACE8C4 = Pin(13, Pin.OUT)
# CANACC8
canACC81 = Pin(0, Pin.IN, Pin.PULL_UP)
canACC82 = Pin(1, Pin.IN, Pin.PULL_UP)
canACC83 = Pin(8, Pin.IN, Pin.PULL_UP)
canACC84 = Pin(9, Pin.IN, Pin.PULL_UP)

def positionUpdate(led, button, ace8c, acc8):
    if button.value() == 1:
        ace8c.on()
    else:
        ace8c.off()
        
    if acc8.value() == 0:
        led.on()
    else:
        led.off()

def mainLoop():
    while True:
        time.sleep_us(100000)
    
        ledPico.toggle()
        
        positionUpdate(ledA, buttonA, canACE8C1, canACC81)    
        positionUpdate(ledB, buttonB, canACE8C2, canACC82)    
        positionUpdate(ledC, buttonC, canACE8C3, canACC83)        
        positionUpdate(ledD, buttonRED, canACE8C4, canACC84)    

mainLoop()