from machine import UART, Pin, I2C
from rotary_irq_pico import RotaryIRQ
import MCP23017
import time, utime

switchAddresses = [
    [
    14, # Encoder 5 Button
    20, # Switch 1
    21, # Switch 2
    22, # Switch 3
    23, # Switch 4
    24, # Switch 5
    25, # Switch 6
    26, # Switch 7
    27, # Switch 8
    28, # Switch 9
    35, # Switch 16
    34, # Switch 15
    33, # Switch 14
    32, # Switch 13
    31, # Switch 12
    30  # Switch 11
    ],
    [
    0,  # not defined
    0,  # not defined
    0,  # not defined
    15, # External PTT
    11, # Encoder 2 Button
    12, # Encoder 3 Button
    13, # Encoder 4 Button
    29, # Switch 10
    17, # MIC Button Down
    18, # MIC Button Up
    16, # MIC PTT
    19, # MIC Fst Scan Button
    0,  # not defined
    0,  # not defined
    0,  # not defined
    0   # not defined
    ]
    ]

expander = [ None, None ]
expanderValues = [ 0, 0 ] 

# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__

def encoderCallback(encoder):
    curValue = encoder.value()
    encoder.reset()
    
    if curValue < 0:
        direction = 0
    else:
        direction = 1
    
    print("{0:b}".format(encoder._encoderAddress << 1 | direction))
    uart0.write(bytes([encoder._encoderAddress << 1 | direction]))

def expanderCallback(pin):
    expanderId = 0 if pin == Pin(22, Pin.IN) else 1
    
    if expander[expanderId] != None:
        expander[expanderId].interrupt_captured
        
        oldVal = expanderValues[expanderId]
        newVal = expander[expanderId].gpio
        
        changedParts = oldVal ^ newVal
        
        if changedParts == 0:
            return

        newValshift = newVal
        for x in range(32):
            if changedParts & 1 == 1:
                print(switchAddresses[expanderId][x])
                print("{0:b}".format(switchAddresses[expanderId][x] << 1 | newValshift & 1))
                uart0.write(bytes( [ switchAddresses[expanderId][x] << 1 | newValshift & 1 ] ) )
            
            changedParts = changedParts >> 1
            newValshift = newValshift >> 1
            
        expanderValues[expanderId] = newVal
        
        
def init_encoder():
    print("Start initializing encoders")
    # VFO encoder
    vfoencoder = RotaryIRQ(pin_num_clk=2, pin_num_dt=3, min_val=1, max_val=2, reverse=False, range_mode=RotaryIRQ.RANGE_UNBOUNDED, pull_up=True, encoderAddress=1)
    vfoencoder.add_listener(encoderCallback)
    
    # Encoder 2
    encoder2 = RotaryIRQ(pin_num_clk=13, pin_num_dt=12, min_val=1, max_val=2, reverse=False, range_mode=RotaryIRQ.RANGE_UNBOUNDED, pull_up=True, encoderAddress=2)
    encoder2.add_listener(encoderCallback)
    
    encoder6 = RotaryIRQ(pin_num_clk=15, pin_num_dt=14, min_val=1, max_val=2, reverse=False, range_mode=RotaryIRQ.RANGE_UNBOUNDED, pull_up=True, encoderAddress=6)
    encoder6.add_listener(encoderCallback)

    # Encoder 3
    encoder3 = RotaryIRQ(pin_num_clk=9, pin_num_dt=8, min_val=1, max_val=2, reverse=False, range_mode=RotaryIRQ.RANGE_UNBOUNDED, pull_up=True, encoderAddress=3)
    encoder3.add_listener(encoderCallback)
    
    encoder7 = RotaryIRQ(pin_num_clk=11, pin_num_dt=10, min_val=1, max_val=2, reverse=False, range_mode=RotaryIRQ.RANGE_UNBOUNDED, pull_up=True, encoderAddress=7)
    encoder7.add_listener(encoderCallback)
    
    # Encoder 4
    encoder4 = RotaryIRQ(pin_num_clk=5, pin_num_dt=4, min_val=1, max_val=2, reverse=False, range_mode=RotaryIRQ.RANGE_UNBOUNDED, pull_up=True, encoderAddress=4)
    encoder4.add_listener(encoderCallback)
    
    encoder8 = RotaryIRQ(pin_num_clk=6, pin_num_dt=7, min_val=1, max_val=2, reverse=False, range_mode=RotaryIRQ.RANGE_UNBOUNDED, pull_up=True, encoderAddress=8)
    encoder8.add_listener(encoderCallback)
    
    # Dual Encoder 5
    encoder5 = RotaryIRQ(pin_num_clk=19, pin_num_dt=18, min_val=1, max_val=2, reverse=False, range_mode=RotaryIRQ.RANGE_UNBOUNDED, pull_up=False, encoderAddress=5)
    encoder5.add_listener(encoderCallback)
    
    encoder9 = RotaryIRQ(pin_num_clk=17, pin_num_dt=16, min_val=1, max_val=2, reverse=False, range_mode=RotaryIRQ.RANGE_UNBOUNDED, pull_up=False, encoderAddress=9)
    encoder9.add_listener(encoderCallback)

def initI2C():
    print("Start initializing I2C Bus")
    
    global i2c
    i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=100000)
    devices = i2c.scan()

    for x in range(2):    
        expander[x] = MCP23017.MCP23017(i2c, 0x20 + x)
        print(0x20+x)
        expander[x].config(interrupt_open_drain=0, sequential_operation=1, interrupt_mirror=1, bank=1)

        expander[x][0].input()
        expander[x][1].input()

        expander[x].input_polarity = 0xFFFF
        expander[x].interrupt_enable = 0xFFFF
        expander[x].interrupt_captured_gpio(expander[x].porta)
        expander[x].interrupt_captured_gpio(expander[x].portb)

        expanderValues[x] = expander[x].gpio

    global expander1irq
    expander1irq = Pin(22, Pin.IN)
    expander1irq.irq(expanderCallback, trigger=Pin.IRQ_FALLING)
    
    global expander2irq
    expander2irq = Pin(26, Pin.IN)
    expander2irq.irq(expanderCallback, trigger=Pin.IRQ_FALLING)
    
    expanderValues[1] = expander[1].gpio
    
def initSerial():
    print("Start initializing serial port")
    global uart0
    uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
    

def init():
    initSerial()
    initI2C()
    init_encoder()
    
    data = [0xFF]
    uart0.write(bytes(data))
    
def main():    
    init()
 
    while True:
        utime.sleep(1)

main()