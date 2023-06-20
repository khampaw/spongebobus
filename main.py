#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from time import sleep
import socket
import math

BUFFER_SIZE=1024

class Client:
    def __init__(self, ip:str, port:int):
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ip = ip
        self.port = port
        self.buffer = ""
        self.conn.connect((self.ip, self.port))

    def recive(self)->str:
        rd = self.conn.recv(1024)
        return rd.decode('utf8')
    
    def send(self, msg:str):
        self.buffer += msg
        self.conn.send(msg.encode('utf8'))
        self.flush()

    def flush(self):
        size = BUFFER_SIZE - len(self.buffer) % BUFFER_SIZE
        self.conn.send(("" * size).encode('utf8'))
        self.buffer = ""

    def close(self):
        self.conn.close()

class Robot:
    def __init__(self, client:Client):
        self.m1 = Motor(Port.C)
        self.m2 = Motor(Port.A)
        self.m3 = Motor(Port.B)
        self.grab = Motor(Port.D)
        self.gyro = GyroSensor(Port.S2)
        self.motors = [self.m1, self.m2, self.m3]
        self.client = client
        self.touch = TouchSensor(Port.S1)

    def filter_speed_percent(self, power):
        return power if (abs(power) <= 100) else (100 if power > 100 else -100)

    def stop(self):
        for i in range(3):
            self.motors[i].stop()

    def moveD(self, cangle:float, angle:float, power:int, delta=0, kp=1):
        err = cangle - angle
        delta += 90 - angle
        delta %= 360
        rounded_angle = round(angle)
        z = (rounded_angle % 360, (120 + rounded_angle) % 360, (240 + rounded_angle) % 360)
        # cos(x + d) for 2' and 3' zones and cos(1.5 * x + d) for zone 1'
        
        for i in range(3):
            if z[i] - 60 > z[i] + 60:
                r = cangle in range(z[i] - 60, 0) or cangle in range(0, z[i] + 60)
            else:
                r = cangle in range(z[i] - 60, z[i] + 60)
            sp = power * (1.5 * round(math.cos(math.radians(delta - 180 + i * 120)), 6) if r else round(math.cos(math.radians(delta - 180 + i * 120)), 6)) + err * kp
            # GEARS!
            self.motors[i].dc(-self.filter_speed_percent(sp))
    
    def check_ships(self):
        self.client.send("check")
        coords = list(map(float, self.client.recive().split(" ")))
        return coords

    def get_coords(self):
        self.client.send("coords")
        coords = list(map(float, self.client.recive().split(" ")))
        return coords
    
    def grabber(self, dir:bool):
        self.grab.run_target(140, (-160 if dir else 0))

    def stabilize(self, angle:float, kp:float):
        coords = self.get_coords()
        ang = -(self.gyro.angle()) % 360
        if ang < 0:
            coords[2] -= 360
        while abs(angle - coords[2]) > 4 :
            ang = -(self.gyro.angle() * -1) % 360
            coords = self.get_coords()
            if ang < 0:
                coords[2] -= 360
            self.move(angle, 0, 0, kp)
        
    def move(self, angle, spd=0, delta=0, kp=4):
        ang = -(self.gyro.angle())
        if ang > 360:
            ang %= 360
        elif ang < -360:
            ang = -1 * (abs(ang) % 360)
        coords = self.get_coords()
        if ang < 0:
            coords[2] -= 360
        #print(ang, coords[2])
        diff = abs(ang - coords[2])
        if diff > 100:
            coords[2] = ang
        self.moveD(coords[2], angle, spd, delta, kp)
    
    def move_coord(self, coord_x, coord_y, angle, spd=0, kp=4):
        coords = self.get_coords()
        while(abs(coords[0] - coord_x) > 10) or (abs(coords[1] - coord_y) > 10):
            coords = self.get_coords()
            # y inversion
            y = coords[1] - coord_y
            x = coord_x - coords[0]
            delta = math.degrees(math.atan2(y, x))
            # -90 + 360 = 270
            delta += (360 if delta < 0 else 0)
            #print(delta, coords[0], coords[1])
            self.move(angle, spd, delta, kp)

    
def main():
    ev3 = EV3Brick()
    touch = TouchSensor(Port.S1)
    while not touch.pressed():
         sleep(1)
    ev3.speaker.beep()
    c = Client("192.168.88.200", 8080)
    r = Robot(c)

    
          
    while 1:
        while not touch.pressed():
            sleep(1)
        r.gyro.reset_angle(0)
        ev3.speaker.beep()
        sleep(3)
        # chekaem razmer korablikov
        ship_coords = r.check_ships()
        print(*ship_coords)
        # cx, cy, tx, ty, o
        if ship_coords[2] == -1:
            print("No place for parking")
            ev3.speaker.beep()
            ev3.speaker.beep()
            continue
        if ship_coords[0] == 0:
            print("No ship")
            ev3.speaker.beep()
            ev3.speaker.beep()
            ev3.speaker.beep()
            continue
        #### zaxBat korablya ####
        ang = 0
        pang = 0
        coords_start = (360, 270)
        if ship_coords[4] == 1:
            print(1)
            r.stabilize(90, 4)
            ev3.speaker.beep()
            coords = r.get_coords()
            if coords[1] < ship_coords[1]:
                print("vvv")
                while abs(coords[1] - ship_coords[1]) > 10:
                    coords = r.get_coords()
                    #print(coords[0], coords[1])
                    # vvv 
                    r.move(90, 70, 270, 4)
            elif coords[1] > ship_coords[1]:
                print("^^^")
                while abs(coords[1] - ship_coords[1]) > 10:
                    coords = r.get_coords()
                    #print(coords[0], coords[1])
                    # ^^^
                    r.move(90, 70, 90, 4)

            if ship_coords[0] < coords[0]:

                print("<<<")
                #r.move_coord(ship_coords[0] - 110, ship_coords[1], 90, 100)
                coords = r.get_coords()
                while abs(coords[0] - ship_coords[0]) > 110:
                    coords = r.get_coords()
                    # # <<<
                    r.move(90, 80, 180, 4) 
                pang = 0 
            elif ship_coords[0] > coords[0]:
                print(">>>")
                coords = r.get_coords()
                while abs(coords[0] - ship_coords[0]) > 110:
                    coords = r.get_coords()
                    # >>>
                    r.move(90, 80, 0, 4)
                pang = 180
            ang = 90

        elif ship_coords[4] == 0:
            print(0)
            coords = r.get_coords()
            # 540 // 2
            print("^^^")
           
            while coords[1]  >  260:
                    coords = r.get_coords()
                    # ^^^
                    r.move(0, 70, 90, 3)
            ev3.speaker.beep()
            if ship_coords[0] < coords[0]:
                coords = r.get_coords()
                print("<<<")
                diff = coords[0] - ship_coords[0]
                while abs(coords[0] - ship_coords[0]) > 10:
                    coords = r.get_coords()
                    # <<<
                    r.move(0, int(100 * ((coords[0] - ship_coords[0]) / diff)), 180, 4) 
            elif ship_coords[0] > coords[0]:
                coords = r.get_coords()
                print(">>>")
                diff = coords[0] - ship_coords[0]
                while abs(coords[0] - ship_coords[0]) > 10:
                    coords = r.get_coords()
                    # >>>
                    r.move(0, 70, 0, 3)
            ev3.speaker.beep()
            print("vvv")
            diff = coords[1] - ship_coords[1]
            while abs(coords[1] - ship_coords[1]) > 130:
                    coords = r.get_coords()
                    # vvv 
                    r.move(0, 70, 270, 3)
            ang = 0
            pang = 180
        
        ev3.speaker.beep()


        ######### PARK ##########
        r.grabber(True)

        if ship_coords[2] > coords[0]:
            print(">>>")
            diff = coords[0] - ship_coords[0]
            while coords[0] < ship_coords[2] - 30:
                    coords = r.get_coords()
                    # >>>
                    r.move(ang, 100, 0, 3)
        elif ship_coords[2] < coords[0]:
            print("<<<")
            diff = coords[0] - ship_coords[0]
            while coords[0] > ship_coords[2] + 30:
                    coords = r.get_coords()
                    # <<<
                    r.move(ang, 70, 180, 3)
        print(pang)
        r.stabilize(pang, 3)
        ev3.speaker.beep()

        # meow(parking)
        
        ev3.speaker.beep()
        ev3.speaker.beep()
        
        print("^^^")
        while coords[1]  >  170:
            coords = r.get_coords()
            # ^^^
            r.move(pang, 50, 90, 3)
        r.stop()
        ev3.speaker.beep()
        r.grabber(False)
        sleep(2)
        # vozrashenie v isxodku
        print("vvv")
        diff = coords[1] - ship_coords[1]
        while coords[1]  <  coords_start[1]:
            coords = r.get_coords()
            # vvv
            r.move(pang, int(100 * ((coords[1] - ship_coords[1]) / diff)), 270, 3)
        r.stop()
        ev3.speaker.beep()
        r.stabilize(0, 2)
        r.stop()
        ev3.speaker.beep()
        ev3.speaker.beep()
        ev3.speaker.beep()
        ev3.speaker.beep()
        


if __name__== "__main__":
    main()
