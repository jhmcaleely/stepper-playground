from machine import Pin, Timer
import time
import micropython

micropython.alloc_emergency_exception_buf(100)


led1 = Pin(15, Pin.OUT)
led2 = Pin(13, Pin.OUT)
led3 = Pin(11, Pin.OUT)
led4 = Pin(9, Pin.OUT)

button1 = Pin(14, Pin.IN, Pin.PULL_DOWN)
button2 = Pin(12, Pin.IN, Pin.PULL_DOWN)
button3 = Pin(10, Pin.IN, Pin.PULL_DOWN)
button4 = Pin(8, Pin.IN, Pin.PULL_DOWN)


fault = Pin(19, Pin.IN)
fault.irq(lambda p: led4.off() if p.value() else led4.on())


step = Pin(16, Pin.OUT)
direction = Pin(17, Pin.OUT)
sleep = Pin(18, Pin.OUT)
sleep.on()


def set_step(step_size):
    if step_size == 4 or step_size == 32:
        m0 = Pin(26, Pin.OPEN_DRAIN)
    else:
        m0 = Pin(26, Pin.OUT)
        
    m1 = Pin(27, Pin.OUT)

    if step_size == 1:
        m0.value(0)
        m1.value(0)
    elif step_size == 2:
        m0.value(1)
        m1.value(0)
    elif step_size == 4:
        m0.value(1)
        m1.value(0)
    elif step_size == 8:
        m0.value(0)
        m1.value(1)
    elif step_size == 16:
        m0.value(1)
        m1.value(1)
    elif step_size == 32:
        m0.value(1)
        m1.value(1)
    



moves = 200
def movecallback(t):
    global moves
    moves -= 1
    if moves <= 0:
        led2.off()
        t.deinit()
    step.toggle()


def syncstep():
    step.on()
    time.sleep_us(750)
    step.off()

accel = 8000
def accelmove(t):
    global accel
    accel -= 1
    print(accel)
    if accel > 7900:
        led2.on()
        if accel % 32 == 0:
            syncstep()
            return
    elif accel < 7900 and accel > 7800 and accel % 16 == 0:
        print("HELLO")
        syncstep()
        return
    elif accel < 7800 and accel > 7600 and accel % 8 == 0:
        print("HELLO8")
        syncstep()
        return
    elif accel < 7600 and accel > 7400 and accel % 4 == 0:
        print("HELLO4")
        syncstep()
        return
    elif accel < 7400 and accel > 7200 and accel % 2 == 0:
        print("HELLO2")
        led2.on()
        syncstep()
        return
    elif accel < 7200 and accel > 1500:
        led2.off()
        syncstep()
        return
    elif accel < 1500 and accel > 1000:
        led2.on()
        if accel % 2 == 0:
            syncstep()
        return
    elif accel < 1000 and accel > 600 and accel % 4 == 0:
        syncstep()
    elif accel < 600 and accel > 300 and accel % 8 == 0:
        syncstep()
    elif accel < 300 and accel > 100 and accel % 16 == 0:
        syncstep()
    elif accel < 100 and accel > 0 and accel % 32 == 0:
        syncstep()
    elif accel <= 0:
        led2.off()
        t.deinit()
    

clicks = 6400
def testrotate(t):
    global clicks
    if clicks > 5344:
        set_step(1)
        clicks -= 1
        if clicks % 32 == 0:
            step.on()
            time.sleep_us(750)
            step.off()
    elif clicks > 4288:
        set_step(2)
        clicks -= 1
        if clicks % 16 == 0:
            step.on()
            time.sleep_us(750)
            step.off()
    elif clicks > 3232:
        set_step(4)
        clicks -= 1
        if clicks % 8 == 0:
            step.on()
            time.sleep_us(750)
            step.off()
    elif clicks > 2176:
        set_step(8)
        clicks -= 1
        if clicks % 4 == 0:
            step.on()
            time.sleep_us(750)
            step.off()
    elif clicks > 1120:
        set_step(16)
        clicks -= 1
        if clicks % 2 == 0:
            step.on()
            time.sleep_us(750)
            step.off()
    elif clicks > 64:
        set_step(32)
        clicks -= 1
        step.on()
        time.sleep_us(750)
        step.off()
    elif clicks > 0:
        set_step(1)
        clicks -= 1
        if clicks % 32 == 0:
            step.on()
            time.sleep_us(750)
            step.off()
    else:
        led1.off()
        t.deinit()

def startaccelrotate(p):
    global accel
    set_step(2)
    accel = 4000
    mover = Timer(period = 2, mode=Timer.PERIODIC, callback=lambda t: micropython.schedule(accelmove, t))

def startfastrotate(p):
    global moves
    set_step(1)
    led2.on()
    moves = 400
    mover = Timer(period = 1, mode=Timer.PERIODIC, callback=lambda t: micropython.schedule(movecallback, t))

def starttestrotate(p):
    global clicks
    led1.on()
    clicks = 6400
    rotate = Timer(period = 10, mode=Timer.PERIODIC, callback=lambda t: micropython.schedule(testrotate, t))
    
def toggledirection(p):
    led3.toggle()
    direction.toggle()
    
def togglesleep(p):
    sleep.toggle()
    

# crude debounce
debouncer = Timer()       
def debounce(pin, button_press):
    global debouncer
    debouncer.init(mode=Timer.ONE_SHOT, period=200, callback=lambda t:micropython.schedule(button_press, pin))

def setupbutton(button, callback):
    button.irq(lambda p:micropython.schedule(lambda z:debounce(z, callback), p))

#setupbutton(button1, starttestrotate)
setupbutton(button1, startaccelrotate)
setupbutton(button2, startfastrotate)
setupbutton(button3, toggledirection)
setupbutton(button4, togglesleep)

        