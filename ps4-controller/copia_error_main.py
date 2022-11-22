#!/usr/bin/env python3
"""
Description: Connect a ps4 dualshock controller and use it to control the xArm robot.
"""

PIN_SUCKING=11
PIN_RELEASE=10
PIN_GRIPPER=8
MODE="SUCTION_CUP"

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
#if len(sys.argv) >= 2:
#    ip = sys.argv[1]
#else:
#    try:
#        from configparser import ConfigParser
#        parser = ConfigParser()
#        parser.read("../robot.conf")
#        ip = parser.get("xArm", "ip")
#    except Exception as e:
#        print(e)
#        ip = input("Please input the xArm ip address:")
#        if not ip:
#            print("input error, exit")
#            sys.exit(1)
########################################################

ip = "192.168.1.225"
arm = XArmAPI(ip)

controller_name = "Wireless Controller"
hasController = False

vendor_id = 0
product_id = 0

for device in hid.enumerate():
    if device["product_string"] == "":
        continue

    print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")
    
    if device["product_string"] == controller_name:
        if hasController:
            print("two controller with same name")
            sys.exit(1)
        
        hasController = True
        vendor_id = device['vendor_id']
        product_id = device['product_id']

if hasController != True:
    print("no controller found")
    sys.exit(1)

gamepad = hid.Device(vendor_id, product_id)
#gamepad.open(vendor_id, product_id)
#gamepad.set_nonblocking(True)

def reset():
    (code, point) = arm.get_initial_point()
    if code != 0:
        print('get_initial_point error: {}'.format(code))
        exit(1)

    code = arm.set_servo_angle(angle=point, is_radian=False, wait=True)
    if code != 0:
        print('set_servo_angle error: {} {}'.format(code, point))
        arm.disconnect()
        exit(1)

# switch to cartesian velocity mode
arm.motion_enable(enable=True)
arm.set_mode(1)
arm.set_state(state=0)
time.sleep(1)

#_, new_pose = arm.get_position()
stop = False
joystick_tolerance = 12
report = None
last_gripper_state = 0
has_reset = False
ancien_report = [0]*64

#_, new_pose = arm.get_position()
#_, angles = arm.get_inverse_kinematics(new_pose)

#sum_rot = angles[3]
#old_sum_rot = sum_rot
#old_pose = new_pose
while stop == False:
    ereport = gamepad.read(64)
    if ereport:
        report = ereport
    if report:
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

        #joy3rot = int(report[4]-127)
        #if joy3rot > -joystick_tolerance and joy3rot < joystick_tolerance:
        #    joy3rot = 0

        #print((report))

        #for i in range(len(report)):
        #    if abs(int(report[i])-int(ancien_report[i]))>1:
        #        print("CHANGED VALUE " + str(i) + "   " + str(report[i]))


        z_axis = report[8]
        if z_axis == 0:
            z_axis = -report[9]

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

        test=[0,10,0,0,0,0]
        #pose = [
        #        0,+1*  -joy1hor / 127,
        #        0,+1*  z_axis / 127,
        #        0,
        #        0,
        #        0,+1*  -joy2ver / 127]

        arm.set_servo_cartesian(test,
                                is_tool_coord=False,
                                is_radian=False)
        #arm.set_position(*[0, 0, 0, 0, 0, 0], wait=True) 
        
        
        time.sleep(0.01)
        
        #if joy3rot<-30:
        #    sum_rot -= 5

        #if joy3rot>30:
        #    sum_rot += 5
        #if sum_rot >50:
        #    sum_rot=50
        #if sum_rot <-50:
        #    sum_rot=-50

        #if sum_rot!=old_sum_rot:
        #    _, angles = arm.get_inverse_kinematics(pose)
        #    if len(angles)>3:
        #        arm.set_mode(0)
        #        arm.set_state(state=0)
        #        angles[3] = sum_rot
        #        time.sleep(0.01)
        #        print(angles)
        #        #arm.set_servo_angle(servo_id=4, angle=sum_rot, is_radian=False) 
        #        arm.set_servo_angle(angle=angles, speed=25, radius=20, wait=False)
        #        time.sleep(0.01)
        #        arm.set_mode(1)
        #        arm.set_state(state=0)
        
        time.sleep(0.01)
        #old_sum_rot = sum_rot
        #old_pose = pose
        # _, new_pose = arm.get_position()

arm.disconnect()
