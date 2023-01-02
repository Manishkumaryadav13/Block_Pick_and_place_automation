from positional_functions import *
import time
from serial_comm import *
DXL_EX='1'
VELOCITY='5'
def dxl_to_top(angle=0):
    torque_on()
    dynamixelController.write((UART_1 + DXL_EX + ',' +VELOCITY+ "\n").encode())
    dynamixelController.write((UART_1 + DXL_EX + ',' + ANGLE + str(angle) + "\n").encode())
    p=dynamixelController.readline().decode()
    print(p)
    dynamixelController.write((UART_1 + DXL_EX + ',' + 'Q' + '?' "\n").encode())
    q=(dynamixelController.readline().decode())
    print(q)
    # if q > '-200.000000':
    #     torque_off()
    #     torque_on()
    # else:
    #     torque_on()

def dxl_to_bottom(angle=55):
    torque_on()
    dynamixelController.write((UART_1 + DXL_EX + ',' +VELOCITY+ "\n").encode())
    dynamixelController.write((UART_1 + DXL_EX + ',' + ANGLE + str(angle) + "\n").encode())
    p=dynamixelController.readline().decode()
    print(p)
    dynamixelController.write((UART_1 + DXL_EX + ',' + 'Q' + '?' "\n").encode())
    q=(dynamixelController.readline().decode())
    print(q)



def present_current():
    dynamixelController.write((UART_1 + DXL_EX + ',' + 'Q' + '?' "\n").encode())
    print(dynamixelController.readline().decode())

def torque_on():
    dynamixelController.write((UART_1 + DXL_EX + ',' + 'T' + '1' "\n").encode())
    print(dynamixelController.readline().decode())

def torque_off():
    dynamixelController.write((UART_1 + DXL_EX + ',' + 'T' + '0' "\n").encode())
    print(dynamixelController.readline().decode())

def close_gripper(angle=210):
    dynamixelController.flush()
    dynamixelController.flushInput()
    dynamixelController.write((UART_1 + DXL_1 + ',' + ANGLE + str(angle) + "\n").encode())
    dynamixelController.readline().decode()
    # time.sleep(1)
    dynamixelController.write((UART_1 + DXL_1 + ',' + 'Q' + '?' "\n").encode())
    q = float(dynamixelController.readline().decode())
    print(q)
    if q > 40.0:
        torque_off()
        # torque_on()
    else:
        torque_on()

def open_gripper(angle=90):
    dynamixelController.flush()
    dynamixelController.flushInput()
    torque_on()
    dynamixelController.write((UART_1 + DXL_1 + ',' + DEFAULT_DYNAMIXEL_VELOCITY + "\n").encode())
    dynamixelController.write((UART_1 + DXL_1 + ',' + ANGLE + str(angle) + "\n").encode())
    print((UART_1 + DXL_1 + ',' + ANGLE  + str(angle)))
