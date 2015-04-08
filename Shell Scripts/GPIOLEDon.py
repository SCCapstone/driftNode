#!/usr/bin/env python

#********WoooHooo!!! Imports!!********#
import RPi.GPIO as GPIO
#***********End 'O Imports************#

GPIOpin = 26

#print(GPIOpin)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIOpin, GPIO.OUT)
#while (1 == 1):
GPIO.output(GPIOpin, GPIO.HIGH)

print("LED on pin ", GPIOpin, "Illuminated")