import RPi.GPIO as GPIO
import time 
import json 

from stepper5 import Stepper

Object = Stepper([18,21,22,23],26) #input pins and LED  

while True:
  try:
    with open("lab5.txt", 'r') as f 
    form = json.load(f) 
    time.sleep(0.5)
    print(form) 
    
 if form['slider1'] != None:
  angle = int(form['slider1'])
  Object.goAngle(angle)
  
 if str(form['zerobutton']) == "Change back to zero":
  Object.zero()


