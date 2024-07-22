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

# Stepper Driver Carrier
stepperNotFault = Pin(2, Pin.IN)
stepperStep = Pin(4, Pin.OUT)
stepperDirection = Pin(3, Pin.OUT, value = 0) # always start same direction
stepperNotSleep = Pin(5, Pin.OUT)
stepperNotSleep.high()


def set_motor_step(step_size):
    if step_size == 1/4 or step_size == 1/32:
        m0 = Pin(7, Pin.OPEN_DRAIN)
    else:
        m0 = Pin(7, Pin.OUT)
        
    m1 = Pin(6, Pin.OUT)

    if step_size == 1:
        m0.value(0)
        m1.value(0)
    elif step_size == 1/2:
        m0.value(1)
        m1.value(0)
    elif step_size == 1/4:
        m0.value(1)
        m1.value(0)
    elif step_size == 1/8:
        m0.value(0)
        m1.value(1)
    elif step_size == 1/16:
        m0.value(1)
        m1.value(1)
    elif step_size == 1/32:
        m0.value(1)
        m1.value(1)

# stepper steps
degreePerStep = 1.8
flStepsPerRotation = 360.0 / degreePerStep
stepsPerRotation = int(flStepsPerRotation)
incrementsPerRotation = stepsPerRotation * 32
fractionPerSubStepSize = int(incrementsPerRotation / 5)
stepperResponseTime_us = 10

def mainloop():
    
    # number of possible increments to move
    clicks = 0
    step_size = 32
    
    while True:
    
        time.sleep_us(stepperResponseTime_us)
    
        ledPico.toggle()
        
        if stepperNotFault.value() == 0:
            ledD.on()
        else:
            ledD.off()
        
        stepperStep.low()
        time.sleep_us(stepperResponseTime_us)
        
        # start two rotations
        if buttonA.value() == 0:
            ledA.on()
            clicks = incrementsPerRotation
        
        # stop immediately
        if buttonB.value() == 0:
            clicks = 0
        
        # swap direction
        if buttonC.value() == 0:
            ledC.toggle()
            stepperDirection.toggle()
        
        # depower the motor. will report stepperFault when not powered
        if buttonRED.value() == 0:
            stepperNotSleep.toggle()
        
        # one rotation at full step
        if clicks == incrementsPerRotation * 2:
           set_motor_step(1)
           step_size = int(32/1)
        # one rotation, in 5 segments of finer steps
        elif clicks == incrementsPerRotation:
            set_motor_step(1/2)
            step_size = int(32/2)
        elif clicks == incrementsPerRotation - 1 * fractionPerSubStepSize:
            set_motor_step(1/4)
            step_size = int(32/4)
        elif clicks == incrementsPerRotation - 2 * fractionPerSubStepSize:
            set_motor_step(1/8)
            step_size = int(32/8)
        elif clicks == incrementsPerRotation - 3 * fractionPerSubStepSize:
            set_motor_step(1/16)
            step_size = int(32/16)
        elif clicks == incrementsPerRotation - 4 * fractionPerSubStepSize:
            set_motor_step(1/32)
            step_size = int(32/32)
    
        if clicks > 0 and clicks % step_size == 0:
            stepperStep.high()
            
        if clicks == 0:
            ledA.off()
        
        clicks -= 1

mainloop()