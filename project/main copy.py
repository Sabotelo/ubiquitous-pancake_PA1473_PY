#!/usr/bin/env pybricks-micropython
from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import sys

import random
import __init__


"""
def follow_line(speed: int, color_sensor: ColorSensor, color: Color, robot: DriveBase) -> None:

    while color_sensor.color() != color:
        robot.turn(1)
    robot.straight(10)

follow_line(speed, color_sensor, color, robot, (-45, 50))
"""


def lets_try_some_shit(robot: DriveBase, light_sensor: ColorSensor, last_reflection_value: int):
        #robot, light_sensor, last_reflection_value

    # while True:
    #     counter = 0
    #     new_reflection_value = light_sensor.reflection()

    #     if counter == 0:
    #         robot.drive(10, 0)

    #     elif light_sensor.reflection() > last_reflection_value:
    #         robot.drive(10, 90)

    #     counter = counter+ 1

    pass


def follow_line(speed: int, color_sensor: ColorSensor, robot: DriveBase,) -> bool:
    """Om den kommer väldigt kort innan den möter färgen igen, så behöver den svänga skarpare.
    Då kan den ta en skarp högersväng. Default bör vara tillräckligt för att klara skarp sväng åt vänster.
    Öka svängningsskärpan om för skarp sväng... Ganska rimligt. Då är grundinställning fördelaktigast kanske
    bättre anpassad för en cirkulär bana."""

    right_angle = -45
    left_angle = 45

    if color_sensor.reflection() < 20:
        robot.drive(speed, left_angle)
    else:
        robot.drive(speed, right_angle)


def the_best_function(left_drive: Motor, right_drive: Motor, color_sensor: ColorSensor, color: Color, robot: DriveBase):
    if color_sensor.reflection() < 40:
        left_drive.run(100)
        right_drive.run(60)
    else:
        left_drive.run(100)
        right_drive.run(60)
    if color_sensor.reflection() > 70:
        left_drive.run(0)
        right_drive.run(100)
    elif color_sensor.reflection() < 20:
        left_drive.run(100)
        right_drive.run(0)
    return

def go_crazy(left_drive: Motor, right_drive: Motor, speed_list = [640, -640]):
    left_speed = speed_list[0]
    right_speed = speed_list[1]
    left_drive.run(left_speed)
    right_drive.run(right_speed)

def follow_line_motor(left_drive: Motor, right_drive: Motor, color_sensor: ColorSensor, color: Color, robot: DriveBase, speed_list = [60, -90]):
    """Speed in run() is deg/second, so about 147mm per full rotation of the wheels. Probably."""
    left_speed = speed_list[0]
    right_speed = speed_list[1]

    if color_sensor.Color() == color:
        speed_list = [60, -90]
        robot.reset()

    # ooooooooooooooooooooo

    left_drive.run(left_speed)
    right_drive.run(right_speed)

    robot.distance()

    # Run using:
    # follow_line_motor(left_drive, right_drive, color_sensor, color, robot)


def follow_multiple_lines(speed: int, light_sensor: ColorSensor, path_color: list, robot: DriveBase, current_color: Color, over_max_reflect: bool, ev3: EV3Brick) -> bool:
    color = light_sensor.color()
    #current_reflect_value = light_sensor.reflection()

    # if light_sensor.color() in path_color and not current_color:  # Svänger till en ny väg
    #     robot.stop()
    #     robot.drive(speed, -10)
    #     wait(200)
    #     while light_sensor.color() is not color.WHITE:
    #         robot.drive(speed, 10)
    #     current_color = light_sensor.color()
    #     robot.stop()

    if light_sensor.reflection() < 60:
        over_max_reflect = False
        robot.drive(speed, 0)
        # ev3.screen.print("FRAMÅT")
    else:
        if 50 <= light_sensor.reflection() < 70 and not over_max_reflect:
            robot.drive(speed-20, -45)
            ev3.screen.print("Vänster")

        elif light_sensor.reflection() > 50:
            over_max_reflect = True
            robot.drive(speed-20, 45)
            ev3.screen.print("HÖGER")
    return over_max_reflect


def collision_detector(robot: DriveBase, ultra_sensor: UltrasonicSensor, min_distance: int, ev3: EV3Brick) -> None:

    while ultra_sensor.distance() < min_distance:
        ev3.screen.clear()
        ev3.screen.print("Get out")
        robot.stop()
        wait(10)
    return


def move_to_pallet(crane_drive: Motor, touch_sensor: TouchSensor, robot: DriveBase, light_sensor: ColorSensor, speed):
    has_pallet = False
    robot.drive(speed, 0)
    wait(1500)
    if touch_sensor.pressed() and not has_pallet:
        wait(500)
        has_pallet = pickup_pallet(crane_drive, touch_sensor, robot)
    if light_sensor.color() is not Color.BLACK:
        parking_color = light_sensor.color()
    else:
        wait(500)
    # if light_sensor.color() = Color.BLACK and touch_sensor.pressed() == False:
        robot.stop()
        robot.drive(-speed, 0)  # backa upp för att testa en ny ruta
        wait(1000)

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

def attack_mode(crane_drive: Motor, robot: DriveBase, ev3: EV3Brick,):
    attacking = True
    robot.reset()
    crane_drive.reset_angle(0)

    while attacking:
        robot.drive(500, random.choice([-90, -45, -15, 15, 45, 90]))
        if crane_drive.angle() > 80:
            crane_drive.run_angle(900, 90, then=Stop.COAST, wait=False)
        if crane_drive.angle() < 20:
            crane_drive.run_angle(900, 0, then=Stop.COAST, wait=False)

        ev3.screen.print(random.choice["KILL", "DEATH", "DESTROY", "MURDER"])
        if robot.distance() > 500:
            attacking = False
    ev3.screen.print("MISSION SUCCESSFUL:\nENEMY ELIMINATED")



def get_color_object(ev3: EV3Brick, available_colors: list(Color), path_colors: list(Color) = []):
    """ask the user for the color of object
    """

    for color in available_colors:
        ev3.screen.clear()
        ev3.screen.print("Press down for: \n"+str(color)[
                         6:]+"\nPress up to\ncontinue\n Press center")
        wait(500)

        while Button.UP not in ev3.buttons.pressed():
            if Button.CENTER in ev3.buttons.pressed():
                return get_color_object(ev3, available_colors, path_colors)
            if Button.DOWN in ev3.buttons.pressed():
                path_colors.append(color)
                ev3.screen.clear()
            if Button.LEFT in ev3.buttons.pressed():
                return path_colors

    ev3.screen.clear()
    return path_colors


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
    light_sensor = ColorSensor(Port.S3)
    ultrasonic_sensor = UltrasonicSensor(Port.S4)
    available_colors = [Color.BLUE, Color.BROWN,
                        Color.RED, Color.YELLOW, Color.GREEN]

    #path_colors = get_color_object(ev3, available_colors)
    run = True
    speed = 40

    collision_distance = 200
    over_max_reflect = False
    current_color = light_sensor.color()
    color_seen = False

    # while True == True:
    #     go_crazy(left_drive,right_drive)

    while run:
        ev3.screen.print(light_sensor.reflection())
        #color_seen =  follow_line(speed, light_sensor, Color.GREEN, robot, color_seen)
        #the_best_function(left_drive,right_drive,light_sensor,Color.GREEN,robot)
        #collision_detector(robot, ultrasonic_sensor, collision_distance, ev3)
        follow_line(speed,light_sensor,robot)

        if Button.CENTER in ev3.buttons.pressed():  # Kolla om knappen är tryckt
            run = False


if __name__ == '__main__':
    sys.exit(main()
             )

    # if right_light.reflection() < 15:
    #     dont_crash()
    #     parking()
    #     right_motor.run(speed)
    #     left_motor.run(speed)
    # else:
    #     while 15 <= right_light.reflection() < 33:
    #         dont_crash()
    #         parking()
    #         right_motor.hold()
    #         left_motor.run(speed)

    #     while right_light.reflection() > 15:
    #         dont_crash()
    #         parking()
    #         right_motor.run(speed)
    #         left_motor.hold()
    # ev3.screen.print(light_sensor.reflection())

"""
        #dont mind this shit
        # last_reflection_value = light_sensor.reflection()
        # from_funktion = lets_try_some_shit(robot, light_sensor, last_reflection_value)

    # punk ass code below:
    while 1 > 99:

        ev3.screen.print(light_sensor.reflection())

        color_seen =  follow_line(speed, light_sensor, Color.GREEN, robot, color_seen)

        #follow_multiple_lines(speed, light_sensor, [Color.GREEN],robot, current_color, over_max_reflect, ev3)

        # check for black area thingy
        # måste nog vara en while i den här
        # if light_sensor.color() == Color.BLACK:
        #     robot.stop()
        #     move_to_pallet(crane_drive: Motor, touch_sensor: TouchSensor, robot: DriveBase, speed)
        # function for the area

        collision_detector(robot, ultrasonic_sensor, collision_distance, ev3)

        if Button.CENTER in ev3.buttons.pressed():  # Kolla om knappen är tryckt
            run = False
"""
