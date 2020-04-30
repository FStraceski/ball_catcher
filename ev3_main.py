#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import UARTDevice
import SystemLink
import SystemLink2

# Write your program here
ev3 = EV3Brick()
#ev3.speaker.beep()

# Set SystemLink Tag to False
#SystemLink.Put_SL('Start27', 'BOOLEAN', 'false')
SystemLink.Get_SL('Start27')
print(SystemLink.Get_SL('Start27'))

# initialize motors
motorA = Motor(Port.A) # Outer motor, ccw negative
motorB = Motor(Port.B) # Base motor, ccw positive
motorC = Motor(Port.C) # Base motor, ccw positive
motorD = Motor(Port.D) # Base motor, negative closes claw

# initialize sensors
touch1 = TouchSensor(Port.S1)

# Throw ball to user
def throw_ball():
    motorA.track_target(0)
    motorB.track_target(0)
    motorC.track_target(0)
    motorD.track_target(-70)
    wait(1000)

    motorA.track_target(-100)
    motorB.track_target(100)
    motorC.track_target(100)
    wait(1000)

    motorA.track_target(0)
    motorB.track_target(0)
    motorC.track_target(0)
    wait(200)
    motorD.track_target(0)
    wait(500)


# setup serial port
serial = UARTDevice(Port.S4, baudrate=9600, timeout=None)
serial.clear()
while(1):
    if SystemLink.Get_SL('Start27') == 'true':
        throw_ball()
        SystemLink.Put_SL('Start27', 'BOOLEAN', 'false')
    if touch1.pressed() == True:
        throw_ball()
        SystemLink.Put_SL('Start28', 'BOOLEAN', 'true')

    if len(ev3.buttons.pressed()) > 0:
        print("Catching Ball")
        wait(3000)
        motorD.track_target(40)

        # collect data from serial port and move armw
        while(1):
            serial.clear()
            ang1 = ""
            ang2 = ""
            curr_byte = serial.read()
            #print(curr_byte)
            # decode instructions once b'a' is seen
            if curr_byte == b'a':
                # get angles
                curr_byte = serial.read()
                while(curr_byte != b'b'):
                    ang1 += curr_byte.decode('utf-8')
                    curr_byte = serial.read()
                curr_byte = serial.read()
                while(curr_byte != b'y' and curr_byte != b'n'):
                    ang2 += curr_byte.decode('utf-8')
                    curr_byte = serial.read()

                if curr_byte == b'y':
                    # move motor and close gripper
                    #print(ang1, ang2)
                    motorA.track_target(-int(ang2))
                    motorB.track_target(int(ang1))
                    motorC.track_target(int(ang1))
                    motorD.track_target(-70)
                    #serial.clear()
                    break
                elif curr_byte == b'n':
                    # move motor
                    #print(ang1, ang2)
                    motorA.track_target(-int(ang2))
                    motorB.track_target(int(ang1))
                    motorC.track_target(int(ang1))
                    motorD.track_target(0)
                    #serial.clear()


'''
while(True):
    motorA.track_target(0)
    motorB.track_target(0)
    motorC.track_target(0)
    motorD.track_target(0)


    
    motorB.track_target(100)
    motorC.track_target(100)
    wait(100)
    motorA.track_target(80)
    motorD.track_target(100)
    wait(500)
    motorB.track_target(70)
    motorC.track_target(70)
    motorA.track_target(40)
    wait(700)
    motorD.track_target(-10)

    wait(3000)
    motorA.track_target(20)
    motorB.track_target(20)
    motorC.track_target(20)
    motorD.track_target(0)
    wait(300)
    motorA.track_target(0)
    motorB.track_target(0)
    motorC.track_target(0)
    wait(1500)

    motorA.track_target(-100)
    motorB.track_target(100)
    motorC.track_target(100)
    wait(1000)


    motorA.track_target(0)
    motorB.track_target(0)
    motorC.track_target(0)
    wait(200)
    motorD.track_target(80)
    wait(500)
    motorD.track_target(0)
    
    break




    if len(ev3.buttons.pressed()) > 0:
         SystemLink.Put_SL('Start27', 'BOOLEAN', 'true')
    
    if SystemLink.Get_SL('Start27') != 'false':
        print("Started")
        if SystemLink2.Get_SL('chomp') == 'true':
            base_pos = 70
            pos = 0
            for i in range(20):
                pos = base_pos + -pos
                motorB.track_target(45)
                motorC.track_target(45)
                motorA.track_target(-45)
                motorD.track_target(pos)
                wait(100)
            SystemLink2.Put_SL('chomp', 'BOOLEAN', 'false')
            motorA.track_target(0)
            motorB.track_target(0)
            motorC.track_target(0)
            motorD.track_target(0)


        if touch1.pressed() == True:
            motorA.track_target(0)
            motorB.track_target(0)
            motorC.track_target(0)
            motorD.track_target(-70)


            motorA.track_target(-100)
            motorB.track_target(100)
            motorC.track_target(100)
            wait(1000)


            motorA.track_target(0)
            motorB.track_target(0)
            motorC.track_target(0)
            wait(200)
            motorD.track_target(0)
            wait(500)
            SystemLink.Put_SL('Start27', 'BOOLEAN', 'false')
        
    wait(100)
'''  
    
'''
motorB.track_target(-70)
motorC.track_target(-70)
motorA.track_target(0)
motorD.track_target(0)
wait(500)
motorD.track_target(-70)
wait(1000)
motorB.track_target(0)
motorC.track_target(0)
wait(10000)
'''

'''
pos = 70
while(True):
    pos = pos * -1
    motorB.track_target(45)
    motorC.track_target(45)
    motorA.track_target(-45)
    motorD.track_target(pos)
    wait(100)
'''
