from machine import Pin
import time
import json

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

position1 = 1000
position2 = 39000
position3 = 73000

currentPosition = 0
currentDestination = currentPosition

def store_cursor(cursor):
    with open("cursor.json", "w") as f:
        json.dump(cursor, f)

def read_cursor():
    with open("cursor.json", "r") as f:
        return json.load(f)

try:
    currentPosition = read_cursor()
except:
    currentPosition = 0
    
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

currStepperPower = True
prevStepperPower = True

def mainloop():
    
    global currentPosition
    global currentDestination
    global position1
    global position2
    global position3
    global currStepperPower
    global prevStepperPower
    
    set_motor_step(1/8)
    step_size = int(32/8)
    
    while True:
        
        time.sleep_us(stepperResponseTime_us)
    
        ledPico.toggle()
        
        #print(currentPosition)
        
        if stepperNotFault.value() == 0:
            ledD.on()
        else:
            ledD.off()
            
        if bool(currStepperPower) and not bool(prevStepperPower):
            print("Zeroing position")
            currentPosition = 0
            currentDestination = 0
            prevStepperPower = True
            
        if buttonA.value() == 0:
            print("desination 1", currentPosition)
            currentDestination = position1
        
        if buttonB.value() == 0:
            currentDestination = position2
        
        if buttonC.value() == 0:
            currentDestination = position3
            
        if currentPosition != currentDestination:
            if currentPosition > currentDestination:
                stepperDirection.low()
                currentPosition -= 1
                
            elif currentPosition < currentDestination:
                stepperDirection.high()
                currentPosition += 1
            
            stepperStep.low()
            time.sleep_us(stepperResponseTime_us)
            stepperStep.high()
        else:
            try:
                if read_cursor() != currentPosition:
                    store_cursor(currentPosition)
                    print("cursor stored")
            except:
                store_cursor(currentPosition)
        
        # depower the motor. will report stepperFault when not powered
        if buttonRED.value() == 0:
            stepperNotSleep.toggle()
            prevStepperPower = currStepperPower
            currStepperPower = not bool(currStepperPower)
        
mainloop()