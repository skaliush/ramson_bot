import ev3dev2.motor as motor
import time
from math import pi, cos, sin
import threading
import socket
import struct
import traceback


IP = "192.168.0.1"
RADIUS = 0.0275
BASE = 0.16
motor_l = motor.LargeMotor('outB')
motor_r = motor.LargeMotor('outA')
max_dps = 1000

inputSignals = [0, 0, 0, 0, 0]
outputSignals = [0, 0, 0, 0, 0]

deg_to_rad = pi / 180
rad_to_deg = 1 / deg_to_rad

lastpos_l = motor_l.position * deg_to_rad
lastpos_r =  motor_r.position * deg_to_rad

x_current, y_current, theta = 0, 0, 0

v_cmd = 0
omega_cmd = 0



def receive_data(inputData: list, s: socket.socket):
    T = 1/50

    while True:
        st = time.time()

        chunk = s.recv(8)
        if chunk == b'':
            raise RuntimeError("socket connection broken")
    
        # receive linear and angular speed
        v, w = struct.unpack(">ff", chunk)
        inputData[0] = v
        inputData[1] = w

        et = time.time()
        dt = T - (et - st)
        if dt > 0:
            time.sleep(dt)

def send_data(outputData: list, s: socket.socket):
    T = 1/50

    while True:
        st = time.time()

        sent = s.send(struct.pack(">fffff", outputData[0], outputData[1], outputData[2], outputData[3], outputData[4]))
        if sent == 0:
            raise RuntimeError("socket connection broken")

        et = time.time()
        dt = T - (et - st)
        if dt > 0:
            time.sleep(dt)



try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((IP, 8089))
    s.listen(5)

    print("Wait for connecting")
    while True:
        clientSock, _ = s.accept()
        print("Connected")
        pubThr = threading.Thread(target=send_data, args=(outputSignals, clientSock))
        subThr = threading.Thread(target=receive_data, args=(inputSignals, clientSock))
        pubThr.start()
        subThr.start()
        break

    while True:
        pos_l = motor_l.position * deg_to_rad
        pos_r = motor_r.position * deg_to_rad
        delta_pos_l = pos_l - lastpos_l
        delta_pos_r = pos_r - lastpos_r
        v = (delta_pos_r + delta_pos_l) * RADIUS / 2
        w = (delta_pos_r - delta_pos_l) * RADIUS / BASE

        x_current = x_current + v * cos(theta)
        y_current = y_current + v * sin(theta)
        theta = theta + w

        outputSignals[0] = x_current
        outputSignals[1] = y_current
        outputSignals[2] = theta
        outputSignals[3] = (motor_r.speed + motor_l.speed) * RADIUS / 2 * deg_to_rad
        outputSignals[4] = (motor_r.speed - motor_l.speed) * RADIUS / BASE * deg_to_rad


        v_cmd = inputSignals[0]
        omega_cmd = inputSignals[1]

        w_r = 0.5 * (2*v_cmd + BASE * omega_cmd) / RADIUS * rad_to_deg
        w_l = 0.5 * (2*v_cmd - BASE * omega_cmd) / RADIUS * rad_to_deg

        max_speed = max(abs(w_r), abs(w_l))
        if max_speed > max_dps:
            scale = max_dps / max_speed
            w_r *= scale
            w_l *= scale

        motor_r.on(motor.SpeedDPS(w_r))
        motor_l.on(motor.SpeedDPS(w_l))

        lastpos_l = pos_l
        lastpos_r = pos_r

except Exception as e:
    print('Error')
    print(traceback.format_exc())
finally:
    motor_l.stop()
    motor_r.stop()
