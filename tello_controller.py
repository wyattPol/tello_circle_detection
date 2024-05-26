# -*- coding: utf-8 -*-
"""
Created on Wed May  8 21:05:36 2024

@author: Yiting Wang
"""

from djitellopy import tello
from threading import Thread, Event
import keyboard
import cv2
import time
import csv
import numpy as np


class TelloController:
    class TelloKillSwitch(Thread):

        tc_handler = None

        def __init__(self, tc_handler):
            Thread.__init__(self)
            self.tc_handler = tc_handler

        def run(self):
            keyboard.wait('space')
            self.tc_handler.force_emergency_stop()

    class TelloTimer(Thread):
        interval = 1.0
        running = None
        func = None

        def __init__(self, interval, event, func):
            Thread.__init__(self)
            self.running = event
            self.interval = interval
            self.func = func

        def run(self):
            while not self.running.wait(self.interval):
                self.func()

    tello_drone = None
    stop_controller = None

    def force_emergency_stop(self):
        self.tello_drone.emergency()
        self.stop_controller.set()

    def __init__(self):


        self.kill_switch = self.TelloKillSwitch(self)
        self.kill_switch.start()

        self.stop_controller = Event()

        self.tello_drone = tello.Tello()
        self.tello_drone.connect()
        self.tello_drone.streamon()
        print('battery', self.tello_drone.get_battery())
        # circle detect and move function apply
        self.tello_drone.takeoff()
        time.sleep(5)
        fly_cir = self.TelloTimer(0.1, self.stop_controller, self.detect_circles)
        fly_cir.start()
        time.sleep(20.0)
        #self.ctr_dir()  # manually control in case of  danger,ctr_dir and detect_circles may have movement conflicts(not sure)
        # if have conflicts then comment ctr_dir
        time.sleep(1)
        self.tello_drone.land()
        time.sleep(1)

        '''
        ###get acc function apply
        acc = self.TelloTimer(0.1,self.stop_controller,self.get_acc)
        angles = self.TelloTimer(0.1, self.stop_controller, self.get_ang)

        #here is dreal time recording function
        recorder = self.TelloTimer(0.1,self.stop_controller,self.vid_cap)
        recorder.start()
        time.sleep(10.0)

        #this is key control  function
        self.tello_drone.takeoff()
        time.sleep(1)
        angles.start()
        acc.start()
        time.sleep(2)
        self.ctr_dir()#control move
        time.sleep(1)
        self.tello_drone.land()
        time.sleep(1)
        '''
        self.stop_controller.set()
        self.tello_drone.end()

    def get_acc(self):

        accx = self.tello_drone.get_acceleration_x()
        accy = self.tello_drone.get_acceleration_y()
        accz = self.tello_drone.get_acceleration_z()
        with open('recording.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['accx', accx])
            writer.writerow(['accy', accy])
            writer.writerow(['accz', accz])

    def get_ang(self):
        pitch = self.tello_drone.get_pitch()
        roll = self.tello_drone.get_roll()
        yaw = self.tello_drone.get_yaw()
        with open('recording.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['pitch', pitch])
            writer.writerow(['yaw', yaw])
            writer.writerow(['roll', roll])

    def get_speed(self):
        speedx = self.tello_drone.get_speed_x()
        speedy = self.tello_drone.get_speed_y()
        speedz = self.tello_drone.get_speed_z()
        with open('recording.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['speedx', speedx])
            writer.writerow(['speedy', speedy])
            writer.writerow(['speedz', speedz])

    def get_time(self):
        ft = self.tello_drone.get_flight_time()
        with open('recording.csv', 'a', newline='') as file:
            writer.writerow(['flight time', ft])

    def vid_cap(self):
        frame_read = self.tello_drone.get_frame_read()
        cv2.imshow("video", frame_read.frame)
        cv2.waitKey(1)

    def detect_circles(self):
        forward_count = 0  # only for obesrving forward command sending times

        frame_read = self.tello_drone.get_frame_read().frame
        gray = cv2.cvtColor(frame_read, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)  # gaussian blurry， 9 will be best
        edges = cv2.Canny(blurred, 100, 200)  # canny edge detect
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=200,
                                   param1=100, param2=30, minRadius=15, maxRadius=200)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(frame_read, (x, y), r, (0, 255, 0), 4)

                mask = np.zeros_like(gray)
                cv2.circle(mask, (x, y), r, (255, 255, 255), -1)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    M = cv2.moments(contours[0])
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    sy=0
                    # direction control to stabilize at center position
                    # time sleep may be recommended to use after each move command,uncomment to use

                    if center_x < frame_read.shape[1] // 2:
                        print("Fly left")
                        sx=-20
                        # self.tello_drone.send_rc_control(-20,10,0,0)
                        # time.sleep(1)
                    elif center_x > frame_read.shape[1] // 2:
                        print("Fly right")
                        sx=20
                        # self.tello_drone.send_rc_control(20,10,0,0)
                        # time.sleep(1)
                    if center_y < frame_read.shape[0] // 2:
                        print("Fly up")
                        sz=20
                        # self.tello_drone.send_rc_control(0,10,20,0)
                        # time.sleep(1)
                    elif center_y > frame_read.shape[0] // 2:
                        print("Fly down")
                        sz=-20
                        # self.tello_drone.send_rc_control(0,10,-20,0)
                        # time.sleep(1)
                self.tello_drone.send_rc_control(sx, sy, sz, 0)
                time.sleep(2)
                self.tello_drone.move_forward(20)


 # time.sleep(1)
        # display real time images from camera
        cv2.imshow('Original Frame', frame_read)
        cv2.imshow('Canny Edge Detection', edges)
        cv2.waitKey(1)

    cv2.destroyAllWindows()

    def ctr_dir(self):

        while True:

            if keyboard.is_pressed('w'):
                self.tello_drone.move_forward(30)
                time.sleep(1)
            elif keyboard.is_pressed('s'):
                self.tello_drone.move_back(30)
                time.sleep(1)
            elif keyboard.is_pressed('a'):

                self.tello_drone.move_left(30)
                time.sleep(1)
            elif keyboard.is_pressed('d'):
                self.tello_drone.move_right(30)
                time.sleep(1)
            if keyboard.is_pressed('q'):  # ESC
                break


if __name__ == '__main__':
    '''
    if os.geteuid() != 0:
        print('You need a root privileges!')
    else:

    '''
    tc = TelloController()
