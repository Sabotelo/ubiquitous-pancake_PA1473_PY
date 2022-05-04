#!/usr/bin/env pybricks-micropython
from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import sys


def follow_line_reflect(turn_rate_left: int, turn_rate_right: int, color_sensor: ColorSensor, robot: DriveBase, angle_tup: tuple) -> None:
    if color_sensor.reflection() < 75:
        robot.drive(turn_rate_left, angle_tup[1])

    else:
        robot.drive(turn_rate_right, angle_tup[0])

def follow_line_color(turn_rate_left: int, turn_rate_right: int, color_sensor: ColorSensor, robot: DriveBase, angle_tup: tuple,colors:list(Color)) -> None:
    if color_sensor.reflection() in colors:
        robot.drive(turn_rate_left, angle_tup[1])

    else:
        robot.drive(turn_rate_right, angle_tup[0])

def collision_detector(robot: DriveBase, ultra_sensor: UltrasonicSensor, min_distance: int, ev3: EV3Brick) -> None:
    while ultra_sensor.distance() < min_distance:
        ev3.screen.clear()
        ev3.screen.print("Get out")
        robot.stop()
        wait(10)
    return


def pickup_pallet(crane_drive: Motor, touch_sensor: TouchSensor, robot: DriveBase)-> bool:

    forks_inserted = False

    while not forks_inserted:
        if touch_sensor.pressed():
            robot.straight(80)
            forks_inserted = True
    print("Out of loop")

    crane_drive.run_time(45, 1000)

    if touch_sensor.pressed():
        return True
    else:
        return False


def get_color_object(ev3: EV3Brick, available_colors: list(Color)) -> list(Color):
    choosen_color = None

    for color in available_colors:
        ev3.screen.clear()
        ev3.screen.print("Down for: \n"+str(color)[6:])
        ev3.screen.print("\nPress up to\ncontinue")
        wait(500)

        while Button.UP not in ev3.buttons.pressed():
            if Button.DOWN in ev3.buttons.pressed():
                ev3.screen.clear()
                return [color,Color.WHITE,Color.BROWN]

    ev3.screen.clear()


def main():

    ev3 = EV3Brick()
    #classMotor(port, positive_direction=Direction.CLOCKWISE, gears=None)
    left_drive = Motor(Port.C, Direction.COUNTERCLOCKWISE, [12, 20])
    right_drive = Motor(Port.B, Direction.COUNTERCLOCKWISE, [12, 20])
    crane_drive = Motor(Port.A, Direction.COUNTERCLOCKWISE, [14, 36])

    #classDriveBase(left_motor, right_motor, wheel_diameter, axle_track)
    robot = DriveBase(left_drive, right_drive, 47, 128)
    touch_sensor = TouchSensor(Port.S1)
    light_sensor = ColorSensor(Port.S3)
    ultrasonic_sensor = UltrasonicSensor(Port.S4)

    # there is no Pink
    # 0 = No Colour
    # 1 = Black
    # 2 = Blue
    # 3 = Green
    # 4 = Yellow
    # 5 = Red
    # 6 = White
    # 7 = Brown

    available_colors = [Color.BROWN, Color.BLUE,
                        Color.RED, Color.GREEN, Color.PURPLE, Color.ORANGE]
    collision_distance = 200
    run = True
    turn_rate = 70


    path_colors = get_color_object(ev3, available_colors)

    while run:

        print(light_sensor.color())

        print(path_colors)

        """
        blå
        (8,29,39)
        (8,19,20)
        (8,20,23)

        brown
        (12,13,5)

        Green
        (7,25,5)
        (7,31,7)
        (7,28,8)

        Pruple
        (9,9,18)
        (7,6,10)
        (7,7,15)

        pink
        (48,16,16)
        (53,16,14)
        (43,15,15)

        Yellow
        (43.37,4)
        (39,34,4)

        """
        if light_sensor.color() is None or light_sensor.color() in path_colors  :
           follow_line_reflect(turn_rate, 0, light_sensor, robot, (30, -45))
        else:
            follow_line_reflect(turn_rate+30, turn_rate+30, light_sensor, robot, (-30, -30))

        collision_detector(robot, ultrasonic_sensor, collision_distance, ev3)

        if Button.LEFT in ev3.buttons.pressed():
            path_colors = get_color_object(ev3, available_colors)
        if Button.UP in ev3.buttons.pressed():
            wait(5)
            pickup_pallet(crane_drive, touch_sensor, robot)
            wait(1000)


if __name__ == '__main__':
    sys.exit(main())
