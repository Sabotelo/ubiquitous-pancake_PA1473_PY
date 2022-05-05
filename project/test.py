#!/usr/bin/env pybricks-micropython
from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


import sys


def follow_line(turn_rate_left: int, turn_rate_right: int, color_sensor: ColorSensor, robot: DriveBase, angle_tup: tuple) -> None:

    if sum(color_sensor.rgb()) < 105:
        robot.drive(turn_rate_left, angle_tup[1])

    else:
        robot.drive(turn_rate_right, angle_tup[0])


def collision_detector(robot: DriveBase, ultra_sensor: UltrasonicSensor, min_distance: int, ev3: EV3Brick) -> None:
    while ultra_sensor.distance() < min_distance:
        ev3.screen.clear()
        ev3.screen.print("Get out")
        wait(200)
        robot.stop()
        wait(10)
    return


def pickup_pallet(crane_drive: Motor, touch_sensor: TouchSensor, robot: DriveBase) -> bool:

    forks_inserted = False

    while not forks_inserted:
        if touch_sensor.pressed():
            robot.straight(80)
            forks_inserted = True
    print("Out of loop")

    crane_drive.track_target(200)

    if touch_sensor.pressed():
        return True
    else:
        return False


def get_color_object(ev3: EV3Brick, available_colors: list(Color)) -> list(Color):
    path_colors = [Color.WHITE, Color.BROWN]

    for color in available_colors:
        ev3.screen.clear()
        ev3.screen.print("Down for: \n"+str(color)[6:])
        ev3.screen.print("\nPress up to\ncontinue")
        wait(500)

        while Button.UP not in ev3.buttons.pressed():
            if Button.DOWN in ev3.buttons.pressed():
                ev3.screen.clear()
                if color == Color.BLACK:
                    path_colors.append(Color.YELLOW, Color.RED)
                    return path_colors
                path_colors.append(color)
                ev3.speaker.say(str(color)[6:])
                return path_colors
        ev3.screen.clear()
    return get_color_object(ev3, available_colors)

def main():

    ev3 = EV3Brick()
    #classMotor(port, positive_direction=Direction.CLOCKWISE, gears=None)
    left_drive = Motor(Port.C, Direction.COUNTERCLOCKWISE, [12, 20])
    right_drive = Motor(Port.B, Direction.COUNTERCLOCKWISE, [12, 20])
    crane_drive = Motor(Port.A, Direction.CLOCKWISE, [14, 36])

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
                        Color.RED, Color.GREEN, Color.PURPLE, Color.ORANGE, Color.BLACK]
    collision_distance = 400
    run = True

    path_colors = get_color_object(ev3, available_colors)
    crane_drive.reset_angle(0)

    rgb_color = sum(light_sensor.rgb())
    color = light_sensor.color()
    dark_angle = (-45, 30)
    light_angle = (30, 30)
    turn_rate = 40
    turn_rate2 = turn_rate+50

    while run:

        print(light_sensor.color())

        if color is None or light_sensor.color() in path_colors:
            if color is Color.BLUE and rgb_color < 40:
                follow_line(turn_rate2, turn_rate2,
                            light_sensor, robot, light_angle)
            else:
                follow_line(turn_rate, 0, light_sensor, robot, dark_angle)

        elif color is Color.BLUE and rgb_color < 40 and Color.PURPLE in path_colors:
            follow_line(turn_rate, 0, light_sensor, robot, dark_angle)

        else:
            follow_line(turn_rate2, turn_rate2,
                        light_sensor, robot, light_angle)

        collision_detector(robot, ultrasonic_sensor, collision_distance, ev3)

        if touch_sensor.pressed():
            robot.stop()
            crane_drive.track_target(200)
            robot.turn(200)

        if Button.LEFT in ev3.buttons.pressed():
            path_colors = get_color_object(ev3, available_colors)
        if Button.UP in ev3.buttons.pressed():
            pickup_pallet(crane_drive, touch_sensor, robot)


if __name__ == '__main__':
    sys.exit(main())
