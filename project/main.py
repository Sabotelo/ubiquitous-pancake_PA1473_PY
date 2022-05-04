#!/usr/bin/env pybricks-micropython
from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import sys

def follow_line(turn_rate: int, color_sensor: ColorSensor, robot: DriveBase, angle_tup:tuple) -> None:
    if color_sensor.reflection() < 85:
        robot.drive(turn_rate, angle_tup[1])

    else:

        robot.drive(0, angle_tup[0])


def turn(left_drive: Motor, right_drive: Motor, robot: DriveBase, ev3: EV3Brick):
    robot.straight(100)
    print("dfdgdf")

    return


def collision_detector(robot: DriveBase, ultra_sensor: UltrasonicSensor, min_distance: int, ev3: EV3Brick) -> None:
    while ultra_sensor.distance() < min_distance:
        ev3.screen.clear()
        ev3.screen.print("Get out")
        robot.stop()
        wait(10)
    return

# def blueblack_loading_area(crane_drive: Motor, touch_sensor: TouchSensor, robot: DriveBase):
#     while touch_sensor.pressed() is False:
#         #do things
#     return pickup_pallet(crane_drive, touch_sensor, robot)



def pickup_pallet(crane_drive: Motor, touch_sensor: TouchSensor, robot: DriveBase):

    forks_inserted = False

    while not forks_inserted:
        if touch_sensor.pressed():
            robot.straight(80)
            forks_inserted = True
    print("Out of loop")
    #crane_drive.track_target(-300) funkar

    if touch_sensor.pressed():
        return True
    else:
        return False


def leave_area():
    pass


def get_color_object(ev3: EV3Brick, available_colors: list(Color)) -> Color:
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
    light_sensor = ColorSensor(Port.S3)
    ultrasonic_sensor = UltrasonicSensor(Port.S4)

    color = Color.RED
    run = True
    turn_rate = 70
    path_colors = [Color.GREEN,Color.WHITE]
    collision_distance = 200

    while run:

        #ev3.screen.print(light_sensor.color())

        follow_line(turn_rate, light_sensor, robot, (-30, 45))

        collision_detector(robot, ultrasonic_sensor, collision_distance, ev3)

        if ColorSensor.color() is color.BLACK:
            while touch_sensor.pressed() is False:

            crane_drive.track_target(300)

        if Button.CENTER in ev3.buttons.pressed():
            run = False

        if Button.UP in ev3.buttons.pressed():
            print("fdss")
            wait(5)
            print("fdssfsd")
            pickup_pallet(crane_drive, touch_sensor, robot)
            wait(1000)


if __name__ == '__main__':
    sys.exit(main())
