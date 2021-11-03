import RPi.GPIO as GPIO
import time
import smbus


class PCF8591:
  def __init__(self,address):
    self.bus = smbus.SMBus(1)
    self.address = address

  def read(self,chn): #channel
      try:
          self.bus.write_byte(self.address, 0x40 | chn)  # 01000000
          self.bus.read_byte(self.address) # dummy read to start conversion
      except Exception as e:
          print ("Address: %s \n%s" % (self.address,e))
      return self.bus.read_byte(self.address)
    
  def write(self,val):
      try:
          self.bus.write_byte_data(self.address, 0x40, int(val))
      except Exception as e:
          print ("Error: Device address: 0x%2X \n%s" % (self.address,e))

class Stepper:
    def __init__(self,pins,ledPin):
        GPIO.setmode(GPIO.BCM)
        pins = [18,21,22,23] # controller inputs: in1, in2, in3, in4
        for pin in pins:
          GPIO.setup(pin, GPIO.OUT, initial=0)
        LEDpin = 26
        GPIO.setup(LEDpin.GPIO.OUT, initial=0)
        

"""# Define the pin sequence for counter-clockwise motion, noting that
# two adjacent phases must be actuated together before stepping to
# a new phase so that the rotor is pulled in the right direction:
ccw = [ [1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],
        [0,0,1,0],[0,0,1,1],[0,0,0,1],[1,0,0,1] ]
# Make a copy of the ccw sequence. This is needed since simply
# saying cw = ccw would point both variables to the same list object:
cw = ccw[:]  # use slicing to copy list (could also use ccw.copy() in Python 3)
cw.reverse() # reverse the new cw sequence 
Define the pin sequence for counter-clockwise motion, noting that 2 adjacent
phases must be actuated together before stepping to a new phase so that
the rotor is pulled in the right direction: """

        self.sequence = [ [1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],
                           [0,0,1,0],[0,0,1,1],[0,0,0,1],[1,0,0,1] ]
        self.state = 0 #Current position in stator sequence
        self.angle = 0 #Current position of angle 
        self.ADC = PCF8591(0x48) 

    def delay_us(self,tus): # use microseconds to improve time resolution
      endTime = time.time() + float(tus)/ float(1E6)
      while time.time() < endTime:
        pass

    def halfstep(self,dir):
    # dir = +/- 1 (ccw/cw)
        self.state += dir 
        if self.state > 7: state = 0
        elif self.state < 0: state = 7
        for halfstep in range(8):
            for pin in range(4): # 4 pins that need to be energized
                GPIO.output(pins[pin], sequence[state][pin])
        self.delay_us(1000)
    
    def moveSteps(steps, dir):
    # move the acuation sequence a given number of half steps
      for step in range(steps):
        self.halfstep(dir)
          
    def goAngle(self,GoAngle): 
      #move to a specified angle, taking the shortest path at a user-defined speed
      self.InputAngle = str(self.angle)
      math = (GoAngle - self.angle + 180) % 360 - 180
      if math <= 180: 
         Stepz= (math * (512*8))/360 #ccw
         dir = +1
        self.moveSteps(int(abs(Stepz)),dir)
        self.angle = GoAngle
      else:   
         Stepz= (math * (512*8))/360 #cw
         dir = -1
         self.moveSteps(int(abs(Stepz)),dir)
         self.angle = GoAngle
       
    def zero(self): 
    #turn the motor until the photoresistor is occluded by the cardboard piece. 
    #Only actuate the LED while the zeroing process is ongoing.
      GPIO.output(self.LEDpin,GPIO.HIGH)
      self.delay_us(1000)
      ON = self.ADC.read(0)
      while(self.ADC.read(0) - ON /ON <= .09
            # while the photoresistor is dark so CHANGE THIS 
            self.halfstep(1)
      GPIO.output(self.LEDpin, GPIO.LOW)
      self.angle = 0
      
    
    """try:
        moveSteps(1000,1)
    except:
        pass
    GPIO.cleanup() """

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

     
