from serial_comm import *
import time

# Axis used for Gantry control
GANTRY_X_AXIS = "X"
GANTRY_Y_AXIS = "Y"
BLOCK_HOLDER = "Z"
DEFAULT_GANTRY_SPEED = 200
DEFAULT_GANTRY_ACCELERATION = 300
DEFAULT_BLADE_SPEED = 12
DEFAULT_BLADE_ACCELERATION = 30
HOME = "G28"
MOVE = "G0"
SPEED = "M203"
ACCELERATION = "M201"
RELATIVE_MOTION = "G91"
ABSOLUTE_MOTION = "G90"
VACUUM_ON = 255
VACUUM_OFF = 0
PUMP = "M106 P0 S"

UART_1='I1'
UART_0='I0'
DXL_1= '1'
DXL_2= '2'
ANGLE='P'
ANGLE = "A="
DEFAULT_DYNAMIXEL_VELOCITY = 'V=200'
abs_pos="1000"
abs_pos1="0"
VELOCITY='50'

def home_block_tray_gantry():
    motorController.write((HOME + GANTRY_Y_AXIS + "\n").encode())
    time.sleep(0.1)
    print("Block Tray Homing")


def home_block_pickup_gantry():
    motorController.write((HOME + GANTRY_X_AXIS + "\n").encode())
    time.sleep(0.1)
    print("Block pickup gantry homing")


def home_block_holder():
    motorController.write((HOME + BLOCK_HOLDER + "\n").encode())
    time.sleep(0.1)
    motorController.write((HOME + BLOCK_HOLDER + "\n").encode())
    print("Block holder homing")


"""
    Relative motion functions for motion in individual axis direction
"""


# Gantry axis direction

def move_block_tray_forward_by(delta):
    _move_block_tray_by(delta)


def move_block_tray_backward_by(delta):
    _move_block_tray_by(-1 * delta)


def move_block_pickup_gantry_left_by(delta):
    _move_block_pickup_gantry_by(delta)


def move_block_pickup_gantry_right_by(delta):
    _move_block_pickup_gantry_by(-1 * delta)


def _move_block_pickup_gantry_by(delta):
    motorController.write((RELATIVE_MOTION + "\n").encode())
    motorController.write((MOVE + GANTRY_X_AXIS + str(delta) + "\n").encode())
    motorController.write((ABSOLUTE_MOTION + "\n").encode())


def _move_block_tray_by(delta):
    motorController.write((RELATIVE_MOTION + "\n").encode())
    motorController.write((MOVE + GANTRY_Y_AXIS + str(delta) + "\n").encode())
    motorController.write((ABSOLUTE_MOTION + "\n").encode())


def move_block_pickup_gantry_to(abs_pos):
    motorController.write((MOVE + GANTRY_X_AXIS + str(abs_pos) + "\n").encode())


def move_block_tray_to(abs_pos):
    motorController.write((MOVE + GANTRY_Y_AXIS + str(abs_pos) + "\n").encode())


def open_block_holder():
    motorController.write((ABSOLUTE_MOTION + "\n").encode())
    motorController.write((MOVE + BLOCK_HOLDER + "5" + "\n").encode())


def close_block_holder():
    motorController.write((ABSOLUTE_MOTION + "\n").encode())
    motorController.write((MOVE + BLOCK_HOLDER + "-5" + "\n").encode())


# speed & acc for all axes
def set_gantry_max_speed(mm_per_sec):
    motorController.write((SPEED + GANTRY_X_AXIS + str(mm_per_sec) + "\n").encode())
    motorController.write((SPEED + GANTRY_Y_AXIS + str(mm_per_sec) + "\n").encode())


def set_gantry_acc(acc):
    motorController.write((ACCELERATION + GANTRY_X_AXIS + str(acc) + "\n").encode())
    motorController.write((ACCELERATION + GANTRY_Y_AXIS + str(acc) + "\n").encode())


def set_block_holder_max_speed(mm_per_sec):
    motorController.write((SPEED + BLOCK_HOLDER + str(mm_per_sec) + "\n").encode())


def set_block_holder_acc(acc):
    motorController.write((ACCELERATION + BLOCK_HOLDER + str(acc) + "\n").encode())


def set_default_speed_and_acceleration():
    set_gantry_max_speed(DEFAULT_GANTRY_SPEED)
    set_gantry_acc(DEFAULT_GANTRY_ACCELERATION)
    set_block_holder_max_speed(DEFAULT_BLADE_SPEED)
    set_block_holder_acc(DEFAULT_BLADE_ACCELERATION)


def current_gantry_pos():
    motorController.flush()
    motorController.flushInput()
    motorController.write("M114\n".encode())
    resp = motorController.readline().decode("UTF-8")
    print(resp)

# Dynamixel


def block_clamping(clamping_pos=0):
    dynamixelController.write((UART_0 + DXL_1 + ',' + DEFAULT_DYNAMIXEL_VELOCITY + "\n").encode())
    dynamixelController.write((UART_0 + DXL_1 + ',' + ANGLE + str(clamping_pos) + "\n").encode())
    print((UART_0 + DXL_1 + ',' + ANGLE + str(clamping_pos) + "\n"))


def block_unclamping(unclamping_pos= 90):
    dynamixelController.write((UART_0 + DXL_1 + ',' + DEFAULT_DYNAMIXEL_VELOCITY + "\n").encode())
    dynamixelController.write((UART_0 + DXL_1 + ',' + ANGLE + str(unclamping_pos) + "\n").encode())
    print((UART_0 + DXL_1 + ',' + ANGLE + str(unclamping_pos) + "\n"))

def rotate_block_Horizontal(r=95):
    dynamixelController.write((UART_0 + DXL_2 + ',' + DEFAULT_DYNAMIXEL_VELOCITY + "\n").encode())
    dynamixelController.write((UART_0 + DXL_2 + ',' + ANGLE + str(r) + "\n").encode())
    dynamixelController.write((UART_0 + DXL_2 + ',' + ANGLE  + str(r) + "\n").encode())
    print((UART_0 + DXL_2 + ',' + ANGLE  + str(r)))

def rotate_block_Vertical(r=5):
    dynamixelController.write((UART_0 + DXL_2 + ',' + DEFAULT_DYNAMIXEL_VELOCITY + "\n").encode())
    dynamixelController.write((UART_0 + DXL_2 + ',' + ANGLE + str(r) + "\n").encode())
    dynamixelController.write((UART_0 + DXL_2 + ',' + ANGLE + str(r) + "\n").encode())
    print((UART_0 + DXL_2 + ',' + ANGLE + str(r)))

def close_gripper_vertical(angle=220):
    dynamixelController.flush()
    dynamixelController.flushInput()
    dynamixelController.write((UART_1 + DXL_1 + ',' + ANGLE + str(angle) + "\n").encode())
    dynamixelController.readline().decode()
    # time.sleep(1)
    dynamixelController.write((UART_1 + DXL_1 + ',' + 'Q' + '?' "\n").encode())
    q = float(dynamixelController.readline().decode())
    print(q)
    if q > 70.0:
        torque_off()
        # torque_on()
    else:
        torque_on()

def open_gripper_vertical(angle=110):
    dynamixelController.write((UART_1 + DXL_1 + ',' +'r'+ "\n").encode())
    dynamixelController.flush()
    dynamixelController.flushInput()
    torque_on()
    dynamixelController.write((UART_1 + DXL_1 + ',' + DEFAULT_DYNAMIXEL_VELOCITY + "\n").encode())
    dynamixelController.write((UART_1 + DXL_1 + ',' + ANGLE + str(angle) + "\n").encode())
    print((UART_1 + DXL_1 + ',' + ANGLE  + str(angle)))

def close_gripper_horizontal(angle=88):
    dynamixelController.flush()
    dynamixelController.flushInput()
    dynamixelController.write((UART_1 + DXL_1 + ',' + ANGLE + str(angle) + "\n").encode())
    dynamixelController.readline().decode()
    # time.sleep(1)
    dynamixelController.write((UART_1 + DXL_1 + ',' + 'Q' + '?' "\n").encode())
    q = float(dynamixelController.readline().decode())
    print(q)
    if q > 65.0:
        torque_off()
        # torque_on()
    else:
        torque_on()

def open_gripper_horizontal(angle=40):
    dynamixelController.write((UART_1 + DXL_1 + ',' + 'r' + "\n").encode())
    dynamixelController.flush()
    dynamixelController.flushInput()
    torque_on()
    dynamixelController.write((UART_1 + DXL_1 + ',' + DEFAULT_DYNAMIXEL_VELOCITY + "\n").encode())
    dynamixelController.write((UART_1 + DXL_1 + ',' + ANGLE + str(angle) + "\n").encode())
    print((UART_1 + DXL_1 + ',' + ANGLE  + str(angle)))


def gripper_to_loading_rotating_block(angle=85):
    dynamixelController.write((UART_1 + DXL_2 + ',' + DEFAULT_DYNAMIXEL_VELOCITY + "\n").encode())
    dynamixelController.write((UART_1 + DXL_2 + ',' + ANGLE + str(angle) + "\n").encode())
    dynamixelController.write((UART_1 + DXL_2 + ',' + ANGLE + str(angle) + "\n").encode())
    print((UART_1 + DXL_2 + ',' + ANGLE  + str(angle)))

def gripper_to_loading_block_holder(angle=87):
    dynamixelController.write((UART_1 + DXL_2 + ',' + DEFAULT_DYNAMIXEL_VELOCITY + "\n").encode())
    dynamixelController.write((UART_1 + DXL_2 + ',' + ANGLE + str(angle) + "\n").encode())
    dynamixelController.write((UART_1 + DXL_2 + ',' + ANGLE + str(angle) + "\n").encode())
    print((UART_1 + DXL_2 + ',' + ANGLE + str(angle)))

def gripper_to_loading_rack(angle=142):
    dynamixelController.write((UART_1 + DXL_2 + ',' + DEFAULT_DYNAMIXEL_VELOCITY + "\n").encode())
    dynamixelController.write((UART_1 + DXL_2 + ',' + ANGLE + str(angle) + "\n").encode())
    dynamixelController.write((UART_1 + DXL_2 + ',' + ANGLE  + str(angle) + "\n").encode())
    print((UART_1 + DXL_2 + ',' + ANGLE  + str(angle)))

def gripper_to_loading_above_rack(angle=180):
    dynamixelController.write((UART_1 + DXL_2 + ',' + DEFAULT_DYNAMIXEL_VELOCITY + "\n").encode())
    dynamixelController.write((UART_1 + DXL_2 + ',' + ANGLE + str(angle) + "\n").encode())
    dynamixelController.write((UART_1 + DXL_2 + ',' + ANGLE + str(angle) + "\n").encode())
    print((UART_0 + DXL_2 + ',' + ANGLE  + str(angle)))

def rotary_dynamixels_current_change(value=1600):
    dynamixelController.write((UART_1 + DXL_EX + ',' + "C" + "=" + str(value) + "\n").encode())
    dynamixelController.write((UART_1 + DXL_EX + ',' + "C" + "=" + str(value) + "\n").encode())
    print((UART_1 + DXL_EX + ',' + "C" + "=" + str(value)))


def gripper_dynamixels_current_change(value=1600):
    dynamixelController.write((UART_1 + DXL_1 + ',' + "C" + "=" + str(value) + "\n").encode())
    dynamixelController.write((UART_1 + DXL_1 + ',' + "C" + "=" + str(value) + "\n").encode())
    print((UART_1 + DXL_1 + ',' + "C" + "=" + str(value)))


def current_gripper_pos():
    dynamixelController.flush()
    dynamixelController.flushInput()
    dynamixelController.write((CURRENT_GRIPPER_POS+"\n").encode())
    resp = dynamixelController.readline().decode("UTF-8")
    resp2 = dynamixelController.readline().decode("UTF-8")
    print(resp2)


def reset_all_dynamixels():
    dynamixelController.write((UART_0 + DXL_1 + ",r\n").encode())
    time.sleep(0.1)
    dynamixelController.write((UART_0 + DXL_2 + ",r\n").encode())
    time.sleep(0.1)
    dynamixelController.write((UART_1 + DXL_1 + ",r\n").encode())
    time.sleep(0.1)
    dynamixelController.write((UART_1 + DXL_2 + ",r\n").encode())
    time.sleep(0.1)
    resp = dynamixelController.readlines()
    print(resp)
    dynamixelController.flush()
    dynamixelController.flushInput()

def max_speed():
    motorController.write((MOVE + "F12000" + "\n"))

def torque_on():
    dynamixelController.write((UART_1 + DXL_1 + ',' + 'T' + '1' "\n").encode())
    dynamixelController.readline().decode()

def torque_off():
    dynamixelController.write((UART_1 + DXL_1 + ',' + 'T' + '0' "\n").encode())
    dynamixelController.readline().decode()

def present_current():
    dynamixelController.write((UART_1 + DXL_1 + ',' + 'Q' + '?' "\n").encode())
    print(dynamixelController.readline().decode())



