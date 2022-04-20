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


def follow_line(speed: int, color_sensor: ColorSensor, color: Color, robot: DriveBase, angle_tup: Tuple) -> None:

    if color_sensor.color() == color:
        robot.drive(speed, angle_tup[0])

    else:
        robot.drive(speed, angle_tup[1])
    return





def collision_detector(robot: DriveBase, ultra_sensor: UltrasonicSensor, min_distance: int, ev3: EV3Brick) -> None:

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


def pickup_pallet(crane_drive: Motor, touch_sensor: TouchSensor, robot: DriveBase):
    """Function for inserting and lifting forks.
    Returns True if pallet is loaded, otherwise False.

    Will currently just drive until it finds a pallet. Still need to figure out how to make sure it finds one though.

    1. Approach pallet
    2. Detect if item on forks
    3. Lift up forks
    """
    forks_inserted = False

    while not forks_inserted:
        if touch_sensor.pressed():
            robot.straight(80)
            forks_inserted = True

    lift_power = 0
    crane_drive.reset_angle()
    while crane_drive.angle() < 20:
        crane_drive.run(lift_power)
        lift_power += 5
    crane_drive.hold()

    if touch_sensor.pressed():
        return True
    else:
        return False


def leave_area():
    """
    moves back to the track safe
    """
    pass


def get_color_object(ev3: EV3Brick, available_colors: list(Color)) -> Color:
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


def main():
    ev3 = EV3Brick()

    left_drive = Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE, gears=[12, 20])
    right_drive = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE, gears=[12, 20])
    crane_drive = Motor(
        Port.A, positive_direction=Direction.COUNTERCLOCKWISE, gears=[14, 36])

    robot = DriveBase(left_drive, right_drive,
                      wheel_diameter=47, axle_track=128)
    touch_sensor = TouchSensor(Port.S1)
    color_sensor = ColorSensor(Port.S3)
    ultrasonic_sensor = UltrasonicSensor(Port.S4)
    available_colors = [Color.BLACK, Color.BLUE,
                        Color.BROWN, Color.RED, Color.YELLOW]
    color = get_color_object(ev3, available_colors)
    run = True
    speed = 70
    stright = False
    has_pallet = False
    collision_distance = 200

    while run:

        ev3.screen.print(color_sensor.color())

        follow_line(speed, color_sensor, color, robot, (-45, 50))

        collision_detector(robot, ultrasonic_sensor, collision_distance, ev3)

        if Button.CENTER in ev3.buttons.pressed():
            run = False

        if touch_sensor.pressed() and not has_pallet:
            wait(500)
            has_pallet = pickup_pallet(crane_drive, touch_sensor, robot)
    return 0



if __name__ == '__main__':
    sys.exit(main())
