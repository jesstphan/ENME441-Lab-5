import RPi.GPIO as GPIO
import time 
import json 

from stepper5 import Stepper

Object = Stepper([18,21,22,23],26) #input pins and LED  

while True:
  try:
    with open("


