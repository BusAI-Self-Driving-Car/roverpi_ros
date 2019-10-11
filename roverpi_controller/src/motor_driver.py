#!/usr/bin/python
import RPi.GPIO as GPIO

'''
GPIO PORTS
'''
in1 = 20
in2 = 21
in3 = 23
in4 = 24
ena = 25
enb = 12

class MotorDriver:

    def __init__(self, dc=100, freq=1500):
        self.dc = dc
        self.freq = freq

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(in3, GPIO.OUT)
        GPIO.setup(in4, GPIO.OUT)
        GPIO.setup(ena, GPIO.OUT)
        GPIO.setup(enb, GPIO.OUT)

        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)

        self.pa = GPIO.PWM(ena, freq)
        self.pb = GPIO.PWM(enb, freq)

        self.pa.start(dc) # where dc is the duty cycle (0.0 <= dc <= 100.0)
        self.pb.start(dc) # where dc is the duty cycle (0.0 <= dc <= 100.0)

    def forward(self, dc=100, freq=1500):
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.HIGH)
        GPIO.output(in4, GPIO.LOW)

        self.pa.ChangeDutyCycle(dc)
        self.pb.ChangeDutyCycle(dc)
        self.pa.ChangeFrequency(freq)
        self.pb.ChangeFrequency(freq)

    def backward(self, dc=100, freq=1500):
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.HIGH)

        self.pa.ChangeDutyCycle(dc)
        self.pb.ChangeDutyCycle(dc)
        self.pa.ChangeFrequency(freq)
        self.pb.ChangeFrequency(freq)

    def leftward(self, dc=100, freq=1500):
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        GPIO.output(in3, GPIO.HIGH)
        GPIO.output(in4, GPIO.LOW)

        self.pa.ChangeDutyCycle(dc)
        self.pb.ChangeDutyCycle(dc)
        self.pa.ChangeFrequency(freq)
        self.pb.ChangeFrequency(freq)

    def rightward(self, dc=100, freq=1500):
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.HIGH)

        self.pa.ChangeDutyCycle(dc)
        self.pb.ChangeDutyCycle(dc)
        self.pa.ChangeFrequency(freq)
        self.pb.ChangeFrequency(freq)

    def left_wheel(self, dc=100, freq=1500, reverse=False):
        if not reverse:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)

        self.pa.ChangeDutyCycle(dc)
        self.pa.ChangeFrequency(freq)

    def right_wheel(self, dc=100, freq=1500, reverse=False):
        if not reverse:
            GPIO.output(in3, GPIO.HIGH)
            GPIO.output(in4, GPIO.LOW)
        else:
            GPIO.output(in3, GPIO.LOW)
            GPIO.output(in4, GPIO.HIGH)

        self.pb.ChangeDutyCycle(dc)
        self.pb.ChangeFrequency(freq)

    def e_stop(self):
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)

        self.pa.stop()
        self.pb.stop()

    def exit(self):
        GPIO.cleanup()
