#!/usr/bin/env python3
"""
Description: Connect a ps4 dualshock controller and use it to control the xArm robot.
"""

PIN_SUCKING=11
PIN_RELEASE=10
PIN_GRIPPER=8
MODE="SUCTION_CUP"
SENSITIVITY=3

import os
import sys
import time
import hid

sys.path.append(os.path.join(os.path.dirname(__file__), "../../.."))

from xarm.wrapper import XArmAPI


#######################################################
"""
Control the robot with a ps4 controller
"""

ip = "192.168.1.225"
arm = XArmAPI(ip)

controller_names = [
    "Wireless Controller",
    "DUALSHOCK 4 Wireless Controller",
]
hasController = False

vendor_id = 0
product_id = 0

for device in hid.enumerate():
    if device["product_string"] == "":
        continue

    print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")
    
    for controller_name in controller_names:
        if device["product_string"] == controller_name:
            if hasController:
                print("two controller with same name")
                sys.exit(1)
            
            hasController = True
            vendor_id = device['vendor_id']
            product_id = device['product_id']
            break

if hasController != True:
    print("no controller found")
    sys.exit(1)

gamepad = hid.device()
gamepad.open(vendor_id, product_id)
gamepad.set_nonblocking(True)

def reset():
    (code, point)= arm.get_initial_point()   # toma el aggulo del programa 
    if code != 0:
        print('get_initial_point error: {}'.format(code))
        exit(1)

    code = arm.set_servo_angle(angle=point, is_radian=False, wait=True)
    if code != 0:
        print('set_servo_angle error: {} {}'.format(code, point))
        arm.disconnect()
        exit(1)

# Set collision sensitivity
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
arm.set_collision_sensitivity(SENSITIVITY)
arm.save_conf()
time.sleep(0.5)

# switch to cartesian velocity mode
arm.motion_enable(enable=True)
arm.set_mode(1)
arm.set_state(state=0)
time.sleep(1)

stop = False
joystick_tolerance = 12
report = None
last_gripper_state = 0
has_reset = False
ereport = None

while stop == False:
    ereport = gamepad.read(64)
    if ereport:
        report = ereport
        if report[0] != 0x01:
            print("Invalid protocol code!")
            print("Please only use bluetooth PS4 controller.")
            print("Please refer to this: https://www.psdevwiki.com/ps4/DS4-BT#0x01")
            sys.exit(1)
    if report:
        print(report)
        should_reset = report[7] & 0b00000001
        if should_reset != 0:
            has_reset = True
            arm.set_mode(0)
            arm.set_state(state=0)
            time.sleep(1)
            reset()
            arm.set_mode(1)
            arm.set_state(state=0)
            time.sleep(1)
            continue
        else: has_reset = False

        joy1ver = int(report[2]-127)
        if joy1ver > -joystick_tolerance and joy1ver < joystick_tolerance:
            joy1ver = 0

        joy1hor = int(report[1]-127)
        if joy1hor > -joystick_tolerance and joy1hor < joystick_tolerance:
            joy1hor = 0

        joy2ver = int(report[3]-127)
        if joy2ver > -joystick_tolerance and joy2ver < joystick_tolerance:
            joy2ver = 0

        z_axis = report[8]
        if z_axis == 0:
            z_axis = -report[9]

        (code, value) = arm.get_cgpio_digital(8)
        if code == 0 and value == 1:
            if z_axis > 0:
                z_axis = 0

        gripper = report[6] & 0b00000010
        gripper_release = report[6] & 0b00000001

        if gripper != 0:
            if last_gripper_state != 1:
                last_gripper_state = 1
                arm.set_cgpio_digital(PIN_RELEASE, 0)
                if MODE == "GRIPPER":
                    arm.set_cgpio_digital(PIN_GRIPPER, 1)
                elif MODE == "SUCTION_CUP":
                    arm.set_cgpio_digital(PIN_SUCKING, 1)
        elif gripper_release != 0:
            if last_gripper_state != 2:
                last_gripper_state = 2
                if MODE == "GRIPPER":
                    arm.set_cgpio_digital(PIN_GRIPPER, 0)
                elif MODE == "SUCTION_CUP":
                    arm.set_cgpio_digital(PIN_SUCKING, 0)
                arm.set_cgpio_digital(PIN_RELEASE, 1)
        elif last_gripper_state == 2:
            last_gripper_state = 0
            arm.set_cgpio_digital(PIN_RELEASE, 0)
        elif last_gripper_state == 1:
            last_gripper_state = 0
            if MODE == "GRIPPER":
                arm.set_cgpio_digital(PIN_GRIPPER, 0)

        
        arm.set_servo_cartesian(
            [
                1 * joy1ver / 127,
                1 * -joy1hor / 127,
                1 * z_axis / 255,
                0,
                0,
                1 * -joy2ver / 127,
            ],
            is_tool_coord=True,
            is_radian=False
        )
        time.sleep(0.01)
     
arm.disconnect()
