import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)


pins = [18,21,22,23] # controller inputs: in1, in2, in3, in4
for pin in pins:
  GPIO.setup(pin, GPIO.OUT, initial=0)

"""# Define the pin sequence for counter-clockwise motion, noting that
# two adjacent phases must be actuated together before stepping to
# a new phase so that the rotor is pulled in the right direction:
ccw = [ [1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],
        [0,0,1,0],[0,0,1,1],[0,0,0,1],[1,0,0,1] ]
# Make a copy of the ccw sequence. This is needed since simply
# saying cw = ccw would point both variables to the same list object:
cw = ccw[:]  # use slicing to copy list (could also use ccw.copy() in Python 3)
cw.reverse() # reverse the new cw sequence """

# Define the pin sequence for counter-clockwise motion, noting that 2 adjacent
# phases must be actuated together before stepping to a new phase so taht
# the rotor is pulled in the right direction:
sequence = [ [1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],
             [0,0,1,0],[0,0,1,1],[0,0,0,1],[1,0,0,1] ]

global state
state = 0 #Current position in stator sequence 


def delay_us(tus): # use microseconds to improve time resolution
  endTime = time.time() + float(tus)/ float(1E6)
  while time.time() < endTime:
    pass

def halfstep(dir):
    # dir = +/- 1 (ccw/cw)
    state += dir 
    if state > 7: state = 0
    elif state < 0: state = 7
    for halfstep in range(8):
        for pin in range(4): # 4 pins that need to be energized
            GPIO.output(pins[pin], sequence[state][pin])
    delay_us(1000)
    
def moveSteps(steps, dir):
    # move the acuation sequence a given number of half steps
    for step in range(steps):
        halfstep(dir)   
try:
    moveSteps(1000,1)
except:
    pass
GPIO.cleanup()

"""# Make a full rotation of the output shaft:
def loop(dir): # dir = rotation direction (cw or ccw)
  for i in range(512): # full revolution (8 cycles/rotation * 64 gear ratio)
    for halfstep in range(8): # 8 half-steps per cycle
      for pin in range(4):    # 4 pins that need to be energized
        GPIO.output(pins[pin], dir[halfstep][pin])
      delay_us(1000)
try:
  loop(cw)
  loop(ccw)
except:
  pass
GPIO.cleanup() """

class Stepper:
  def goAngle(): #move to a specified angle, taking the shortest path at a user-defined speed
    
       
