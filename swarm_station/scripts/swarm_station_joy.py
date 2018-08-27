#!/usr/bin/env python
from __future__ import print_function
import rospy
import pygame
# import pymavlink
import sys, pygame

class RCState():
    def __init__(self):
        self.target_id = 0
        self.rcA = 0
        self.rcE = 0
        self.rcR = 0
        self.rcT = 0
        self.rcAUX1 = 0
        self.rcAUX2 = 0
        self.rcAUX3 = 0
        self.sw1 = 0
        self.sw2 = 0
        self.sw3 = 0

        pygame.init()
        pygame.joystick.init()

        self.joystick = pygame.joystick.Joystick(0)
    
    def pack_mavlink(self):
        return

    def update(self):
        for event in pygame.event.get(): # User did something
            if event.type == pygame.QUIT: # If user clicked close
                done=True # Flag that we are done so we exit this loop
            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")
            print(event)
        

        self.rcA = joystick.get_axis(2)
        self.rcE = - joystick.get_axis(3)
        self.rcR = joystick.get_axis(0)
        self.rcT = joystick.get_axis(1)
        self.rcAUX1 = joystick.get_axis(4)
        self.rcAUX2 = joystick.get_axis(5)

        #Right banji 5
        #Left banji 4

        # Left btn 8
        # Right btn 9

        self.sw1 = joystick.get_button(8)
        self.sw2 = joystick.get_button(9)

        # A 11
        # B 12
        # Y 14
        # X 13



     
if __name__ == "__main__":
    rospy.init_node("swarm_station_joy")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    axes = joystick.get_numaxes()

    print(axes)