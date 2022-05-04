import random
from pybricks.parameters import Stop
from pybricks.hubs import EV3Brick
from pybricks.robotics import DriveBase
from pybricks.ev3devices import Motor

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
