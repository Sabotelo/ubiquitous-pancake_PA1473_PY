#!/usr/bin/env pybricks-micropython

from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor, TouchSensor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import sys
import __init__


def follow_line(speed: int, light_sensor: ColorSensor, color: Color, robot: DriveBase) -> None:

    if light_sensor.color() == color:
        robot.drive(speed, -45)
    else:
        robot.drive(speed, 45)
    return


def collision_dedector(robot: DriveBase, ultra_sensor: UltrasonicSensor, min_distance: int, ev3: EV3Brick) -> None:

    while ultra_sensor.distance() < min_distance:
        ev3.screen.clear()
        ev3.screen.print("Get out")
        robot.stop()
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


def get_color_object(ev3: EV3Brick, available_colors: List(Color)) -> Color:
    """ask the user for the color of object
    """
    choosen_color = None
    for color in available_colors:
        ev3.screen.clear()
        ev3.screen.print("Press down for: \n"+str(color)[
                         6:]+"\nPress up to\ncontinue")
        wait(500)

        while Button.UP not in ev3.buttons.pressed():
            if Button.DOWN in ev3.buttons.pressed():
                ev3.screen.clear()
                return color

    ev3.screen.clear()
    return get_color_object(ev3, available_colors)


def main():
    ev3 = EV3Brick()

    left_drive = Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE, gears=[12, 20])
    right_drive = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE, gears=[12, 20])
    crane_drive = Motor(
        Port.A, positive_direction=Direction.CLOCKWISE, gears=[14, 36])
    robot = DriveBase(left_drive, right_drive,
                      wheel_diameter=47, axle_track=128)
    front_button = TouchSensor(Port.S1)
    light_sensor = ColorSensor(Port.S3)
    ultrasonic_sensor = UltrasonicSensor(Port.S4)

    color = Color.YELLOW
    run = True
    speed = 70
    collision_distance = 200
    available_colors = [Color.BLACK, Color.BLUE,
                        Color.BROWN, Color.RED, Color.YELLOW]

    while run:

        follow_line(speed, light_sensor, color, robot)
        collision_dedector(robot, ultrasonic_sensor, collision_distance, ev3)

        if Button.CENTER in ev3.buttons.pressed():
            run = False

    return 0


if __name__ == '__main__':
    sys.exit(main())
