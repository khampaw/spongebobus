import cv2
import socket
import imutils
import numpy as np
from time import sleep
from threading import Thread
from imutils.video import VideoStream
import math
import threading

"""
Simple server to detect edges of marks from camera and send data to robot
"""

BUFFER_SIZE = 1024


class Serv:
    def __init__(self, port):
        self.buffer = ""
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        IP = socket.gethostbyname(socket.gethostname())
        PORT = port
        listener.bind((IP, PORT))
        listener.listen(1)
        self.connection, self.address = listener.accept()
        print("connected")

    def send(self, msg: str):
        self.buffer += msg
        self.connection.send(msg.encode('utf8'))
        self.flush()

    def flush(self):
        size = BUFFER_SIZE - len(self.buffer) % BUFFER_SIZE
        self.connection.send(("" * size).encode('utf8'))
        self.buffer = ""

    def recive(self):
        data = self.connection.recv(1024).decode("utf8")
        return data


class Rectangle:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.closest_left = 0
        self.closest_right = 0
        self.closest_up = 0
        self.closest_down = 0
        self.cx = x + w // 2
        self.cy = y + h // 2


class Camera:
    def __init__(self):
        self.red_rects = []
        self.red_lower = [0, 126, 0]
        self.red_higher = [18, 255, 255]
        self.red_threshold_w = 50
        self.red_threshold_h = 20

        self.yellow_lower = [7, 33, 0]
        self.yellow_higher = [26, 213, 255]
        #self.yellow_lower = [97, 20, 0]
        #self.yellow_higher = [140, 255, 255]
        self.yellow_rect = Rectangle(0, 0, 0, 0)
        self.yellow_threshold_w = 50
        self.yellow_threshold_h = 50

        self.green_lower = [26, 25, 0]
        self.green_higher = [97, 238, 255]
        self.green_rect = Rectangle(0, 0, 0, 0)
        self.green_threshold_w = 40
        self.green_threshold_h = 40

        self.direction = 0
        self.ang = 0
        self.vs = cv2.VideoCapture(1)
        self.vs.set(cv2.CAP_PROP_FOCUS, 0)
        self.vs.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
        self.vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
        _, self.frame = self.vs.read()

    def do(self):

        self.red_rects = []
        self.yellow = []
        self.green = []

        # Бортики
        self.red_rects.append(Rectangle(0, 0, 100, 0))
        self.red_rects.append(Rectangle(620, 0, 100, 0))
        #
        _, frame = self.vs.read()
        frame = cv2.flip(frame, -1)
        #frame = imutils.resize(frame, width=720)

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        self.hsv = hsv
        contours_red = self.get_contours(hsv, self.red_lower, self.red_higher)
        contours_yellow = self.get_contours(hsv, self.yellow_lower, self.yellow_higher)
        contours_green = self.get_contours(hsv, self.green_lower, self.green_higher)
        if len(contours_red) > 0:
            for cnt in contours_red:
                x, y, w, h = cv2.boundingRect(cnt)
                if w >= self.red_threshold_w and h >= self.red_threshold_h:
                    self.red_rects.append(Rectangle(x, y, w, h))

                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    M = cv2.moments(cnt)
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)

        if len(contours_yellow) > 0:
            mw = -1
            mh = -1
            for cnt in contours_yellow:
                rec = cv2.minAreaRect(cnt)
                if rec[1][0] >= self.yellow_threshold_w and rec[1][1] >= self.yellow_threshold_h:
                    if rec[1][0] > mw and rec[1][1] > mh:
                        self.yellow_rect = Rectangle(0, 0, rec[1][0], rec[1][1])
                        self.yellow_rect.cx = rec[0][0]
                        self.yellow_rect.cy = rec[0][1]

                        M = cv2.moments(cnt)
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        cv2.circle(frame, (cx, cy), 5, (255, 0, 255), -1)

        if len(contours_green) > 0:
            mw = -1
            mh = -1
            for cnt in contours_green:
                rec = cv2.minAreaRect(cnt)
                if rec[1][0] >= self.green_threshold_w and rec[1][1] >= self.green_threshold_h:
                    if rec[1][0] > mw and rec[1][1] > mh:
                        self.green_rect = Rectangle(0, 0, rec[1][0], rec[1][1])
                        self.green_rect.cx = rec[0][0]
                        self.green_rect.cy = rec[0][1]

                        M = cv2.moments(cnt)
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

        self.frame = frame

    def get_contours(self, hsv, lower, upper):
        mask = cv2.inRange(hsv, (lower[0], lower[1], lower[2]), (upper[0], upper[1], upper[2]))
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        return contours

    def partition(self, array, low, high):
        pivot = array[high]
        i = low - 1
        for j in range(low, high):
            if array[j].x <= pivot.x:
                i = i + 1
                (array[i], array[j]) = (array[j], array[i])
        (array[i + 1], array[high]) = (array[high], array[i + 1])
        return i + 1

    def quickSort(self, array, low, high):
        if low < high:
            pi = self.partition(array, low, high)
            self.quickSort(array, low, pi - 1)
            self.quickSort(array, pi + 1, high)

    def check_parking(self, ship: Rectangle):
        self.do()
        h, w, _ = self.hsv.shape
        x = -1
        y = -1
        rs = []
        for rect in self.red_rects:
            if rect.y < h // 2:
                rs.append(rect)
        self.quickSort(rs, 0, len(rs) - 1)
        for idx in range(1, len(rs)):
            r = rs[idx]
            prev = rs[idx - 1]
            diff = abs(r.x - (prev.x + prev.w))
            if diff > ship.w:
                return (r.x - diff // 2, r.y)
        else:
            return (-1, -1)

    def check_size(self) -> Rectangle:
        self.do()
        h, w, _ = self.hsv.shape
        r = Rectangle(0, 0, 0, 0)
        for rect in self.red_rects:
            if rect.y > h // 2 and rect.w >= r.w and rect.h > r.h:
                r = rect
        return r

    def get_robo_coords(self):
        self.do()
        ang = 0
        ax = self.green_rect.cx
        ay = self.green_rect.cy
        bx = self.yellow_rect.cx
        by = self.yellow_rect.cy

        l = math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)
        #print(math.degrees(math.asin((ay - by) / l)), math.degrees(math.acos((ax - bx) / l)))
        ang = math.degrees(math.atan2(by - ay, ax - bx))
        # cat = (by - ay) / 2
        # ang = math.degrees(math.asin(cat / (l / 2)))
        ang = (ang + 360) % 360

        diff_x = abs(self.green_rect.cx - self.yellow_rect.cx) // 2
        diff_y = abs(self.green_rect.cy - self.yellow_rect.cy) // 2
        self.ang = ang
        #return ((ax, bx), (ay, by), ang)
        return (max(self.green_rect.cx, self.yellow_rect.cx) - diff_x, max(self.green_rect.cy, self.yellow_rect.cy) - diff_y, ang)


def show(c: Camera):
    while True:
        c.do()
        cv2.imshow("frame", c.frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    cv2.destroyAllWindows()


def main():
    c = Camera()
    print("ready")
    t1 = threading.Thread(target=show, args=(c,))
    t1.start()
    s = Serv(8080)

    while 1:
        msg = s.recive()
        #msg = "coords"
        c.do()
        if msg == "check":
            # проверяем размер корабля
            ship = c.check_size()
            # находим место для парковки
            targ = c.check_parking(ship)
            # шлём на робота
            o = (0 if ship.w > ship.h else 1)
            s.send(str(ship.cx) + " " + str(ship.cy) + " " + str(targ[0]) + " " + str(targ[1]) + " " + str(o) + "\n")

        elif msg == "coords":
            # Берём координаты робота
            coords = c.get_robo_coords()
            # шлём роботу координаты в формате xc yc ang
            #print(*coords)
            s.send(str(coords[0]) + " " + str(coords[1]) + " " + str(coords[2]) + "\n")


if __name__ == "__main__":
    main()

