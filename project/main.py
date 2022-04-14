#!/usr/bin/env pybricks-micropython
import sys
import __init__

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

def follow_line(speed: int, light_sensor: ColorSensor,color: Color, robot: DriveBase) -> None:

    if light_sensor.color() == color:
        robot.drive(speed, 45)
    else:
        robot.drive(speed, -45)
    return


def object_detected(robot: DriveBase, ultra_sensor: UltrasonicSensor, min_distance: int, ev3: EV3Brick) -> None:

    while ultra_sensor.distance() < min_distance:
        ev3.screen.clear()
        ev3.screen.print("Get out")
        robot.stop()
        ev3.speaker.play_notes(["A2/8", "A2/8", "A2/8", "A3/16"])
        wait(10)
    return



def move_to_pallet():
    """
    moves the robot to the specified item
    """
    pass

def pickup_pallet():
    """
    picks a item upp safe
    """
    pass

def leave_area():
    """
    moves back to the track safe
    """
    pass



def main():
    ev3 = EV3Brick()
    left_drive = Motor(Port.C)
    right_drive = Motor(Port.B)
    crane_motor = Motor(Port.A)
    front_button =TouchSensor(Port.S1)
    light_sensor = ColorSensor( Port.S3)
    ultrasonic_sensor =ultrasonic_sensor( Port.S4)
    robot = DriveBase(left_drive, right_drive,
                      wheel_diameter=47, axle_track=128)
    #positive_direction = Direction.COUNTERCLOCKWISE, gears = [12, 20]
    color = Color.YELLOW
    run = True
    speed = 70


    while run:
        follow_line(speed, light_sensor, color, robot)

        if Button.CENTER in ev3.buttons.pressed() or light_sensor.color()==Color.BLACK:
            run = False


    return 0

if __name__ == '__main__':
    sys.exit(main())
