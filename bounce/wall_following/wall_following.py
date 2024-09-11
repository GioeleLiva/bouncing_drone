# -*- coding: utf-8 -*-
#
#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  GNU general public license v3.0
#
#  Copyright (C) 2023 Bitcraze AB
#
#  Crazyflie Python Library
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
file: wall_following.py

Class for the wall following demo

This is a python port of c-based app layer example from the Crazyflie-firmware
found here https://github.com/bitcraze/crazyflie-firmware/tree/master/examples/
demos/app_wall_following_demo

Author:   Kimberly McGuire (Bitcraze AB)
"""
import math
from enum import Enum
from math import degrees


class WallFollowing():
    class StateWallFollowing(Enum):
        FORWARD = 1
        HOVER = 2
        BOUNCING= 3
        SCANNING = 4
        ROTATE__REFLECT = 5
        FIND_ESCAPE = 6


    class WallFollowingDirection(Enum):
        LEFT = 1
        RIGHT = -1

    def __init__(self, reference_distance_from_wall=0.0,
                 max_forward_speed=0.2,
                 max_turn_rate=0.5,
                 wall_following_direction=WallFollowingDirection.LEFT,
                 first_run=False,
                 prev_front=0.0,
                 prev_back=0.0,
                 scanned=False,
                 wall_angle=0.0,
                 arc_radius=0.0,
                 #arc_rate=0.0,
                 arc_time=0.0,
                 around_corner_back_track=False,
                 state_start_time=0.0,
                 ranger_value_buffer=0.2,
                 angle_value_buffer=0.1,
                 range_lost_threshold=0.3,
                 in_corner_angle=0.8,
                 wait_for_measurement_seconds=1.0,
                 init_state=StateWallFollowing.FORWARD):
        """
        __init__ function for the WallFollowing class

        reference_distance_from_wall is the distance from the wall that the Crazyflie
            should try to keep
        max_forward_speed is the maximum speed the Crazyflie should fly forward
        max_turn_rate is the maximum turn rate the Crazyflie should turn with
        wall_following_direction is the direction the Crazyflie should follow the wall
            (WallFollowingDirection Enum)
        first_run is a boolean that is True if the Crazyflie is in the first run of the
            wall following demo
        prev_heading is the heading of the Crazyflie in the previous state (in rad)
        wall_angle is the angle of the wall in the previous state (in rad)
        around_corner_back_track is a boolean that is True if the Crazyflie is in the
            around corner state and should back track
        state_start_time is the time when the Crazyflie entered the current state (in s)
        ranger_value_buffer is the buffer value for the ranger measurements (in m)
        angle_value_buffer is the buffer value for the angle measurements (in rad)
        range_lost_threshold is the threshold for when the Crazyflie should stop
            following the wall (in m)
        in_corner_angle is the angle the Crazyflie should turn when it is in the corner (in rad)
        wait_for_measurement_seconds is the time the Crazyflie should wait for a
            measurement before it starts the wall following demo (in s)
        init_state is the initial state of the Crazyflie (StateWallFollowing Enum)
        self.state is a shared state variable that is used to keep track of the current
            state of the Crazyflie's wall following
        self.time_now is a shared state variable that is used to keep track of the current (in s)
        """

        self.reference_distance_from_wall = reference_distance_from_wall
        self.max_forward_speed = max_forward_speed
        self.max_turn_rate = max_turn_rate
        self.direction_value = float(wall_following_direction.value)
        self.first_run = first_run
        self.prev_front = prev_front
        self.prev_back = prev_back
        self.scanned = scanned
        self.wall_angle = wall_angle
        self.arc_radius = arc_radius
        self.arc_time = arc_time
        #self.arc_rate = arc_rate
        self.around_corner_back_track = around_corner_back_track
        self.state_start_time = state_start_time
        self.ranger_value_buffer = ranger_value_buffer
        self.angle_value_buffer = angle_value_buffer
        self.range_threshold_lost = range_lost_threshold
        self.in_corner_angle = in_corner_angle
        self.wait_for_measurement_seconds = wait_for_measurement_seconds

        self.first_run = True
        self.state = init_state
        self.time_now = 0.0
        self.speed_redux_corner = 3.0
        self.speed_redux_straight = 2.0

    # Helper function
    def value_is_close_to(self, real_value, checked_value, margin):
        if real_value > checked_value - margin and real_value < checked_value + margin:
            return True
        else:
            return False

    def wrap_to_pi(self, number):
        if number > math.pi:
            return number - 2 * math.pi
        elif number < -math.pi:
            return number + 2 * math.pi
        else:
            return number

    def calculate_arc(self, radius=0):
        deg = degrees(self.wall_angle)
        self.arc_radius = radius if radius != 0 else ((90 - deg) / 270)     #raggio della circonferenza, in metri
        distance = (2 * self.arc_radius * math.pi * (2 * deg) / 360) - 2*self.angle_value_buffer          #lunghezza della corda da percorrere, in metri
        self.arc_time = distance / self.max_forward_speed                   #tempo di percorrenza della corda, in secondi
        self.arc_rate = self.max_forward_speed / self.arc_radius            #yaw da mantenere nella percorrenza, in radianti

        print('radius', self.arc_radius, 'distance', distance, 'angle', self.wall_angle)

         

    # Command functions
    def command_turn(self, reference_rate):
        """
        Command the Crazyflie to turn around its yaw axis

        reference_rate and rate_yaw is defined in rad/s
        velocity_x is defined in m/s
        """
        velocity_x = 0.0
        rate_yaw = self.direction_value * reference_rate
        return velocity_x, rate_yaw

    def command_circle(self, radius, velocity_rate):
        """
        Command the Crazyflie to follow an arc of a circle

        radius is defined in m
        velocity is defined in m/s
        """
        velocity_x = velocity_rate
        rate_yaw = self.direction_value * velocity_rate / radius
        return velocity_x, rate_yaw

    def command_hover(self):
        """
        Command the Crazyflie to hover in place
        """
        velocity_x = 0.0
        rate_yaw = 0.0
        return velocity_x, rate_yaw


    # state machine helper functions
    def state_transition(self, new_state):
        """
        Transition to a new state and reset the state timer

        new_state is defined in the StateWallFollowing enum
        """
        self.state_start_time = self.time_now
        return new_state


    # Wall following State machine
    def wall_follower(self, front_range, left_range, right_range, back_range, time_outer_loop):
        """
        wall_follower is the main function of the wall following state machine.
        It takes the current range measurements of the front range and side range
        sensors, the current heading of the Crazyflie, the wall following direction
        and the current time of the outer loop (the real time or the simulation time)
        as input, and handles the state transitions and commands the Crazyflie to
        to do the wall following.

        front_range and side_range is defined in m
        current_heading is defined in rad
        wall_following_direction is defined as WallFollowingDirection enum
        time_outer_loop is defined in seconds (double)
        command_velocity_x, command_velocity_ y is defined in m/s
        command_rate_yaw is defined in rad/s
        self.state is defined as StateWallFollowing enum
        """
    
        self.time_now = time_outer_loop

        if self.first_run:
            self.around_corner_back_track = False
            self.first_run = False

        # -------------- Handle state transitions ---------------- #
        if self.state == self.StateWallFollowing.FORWARD:
            #self.direction_value = 0
            if front_range < self.reference_distance_from_wall + self.ranger_value_buffer:
                if left_range < (2+math.sqrt(3))*self.reference_distance_from_wall + self.ranger_value_buffer or \
                    right_range < (2+math.sqrt(3))*self.reference_distance_from_wall + self.ranger_value_buffer:
                    if left_range<right_range:
                        side_range = left_range
                        oppo_range = right_range
                        self.direction_value = -1
                    else:
                        side_range = right_range
                        oppo_range = left_range
                        self.direction_value = 1
                    if oppo_range < (2+math.sqrt(3))*self.reference_distance_from_wall + self.ranger_value_buffer:
                        self.state = self.state_transition(self.StateWallFollowing.FIND_ESCAPE)
                    self.wall_angle = (math.pi/2 - math.atan(front_range / side_range) + self.angle_value_buffer)
                    self.calculate_arc() 
                    self.state = self.state_transition(self.StateWallFollowing.BOUNCING)                    

                
            #    if left_range < (2+math.sqrt(3))*self.reference_distance_from_wall + self.ranger_value_buffer:
            #        self.direction_value = -1
            #        self.wall_angle = (math.pi/2 - math.atan(front_range / left_range) + self.angle_value_buffer)
            #        self.calculate_arc() 
            #        self.state = self.state_transition(self.StateWallFollowing.BOUNCING)
            #    elif right_range < (2+math.sqrt(3))*self.reference_distance_from_wall + self.ranger_value_buffer:
            #        self.direction_value = 1
            #        self.wall_angle = (math.pi/2 - math.atan(front_range / right_range) + self.angle_value_buffer)
            #        self.calculate_arc() 
            #
            #        self.state = self.state_transition(self.StateWallFollowing.BOUNCING)
                
                else:
                    self.state = self.state_transition(self.StateWallFollowing.SCANNING)
                   
            elif left_range < self.reference_distance_from_wall + self.ranger_value_buffer:
                if front_range < (2+math.sqrt(3))*self.reference_distance_from_wall + self.ranger_value_buffer:
                    self.wall_angle = (math.pi/2 - math.atan(front_range / left_range) + self.angle_value_buffer)
                elif right_range < 2*self.reference_distance_from_wall + self.ranger_value_buffer:
                    self.state = self.state_transition(self.StateWallFollowing.HOVER)
                else:
                    self.wall_angle = math.pi/12
                self.direction_value = -1
                self.calculate_arc()
                self.state = self.state_transition(self.StateWallFollowing.BOUNCING)
            elif right_range < self.reference_distance_from_wall + self.ranger_value_buffer:
                if front_range < (2+math.sqrt(3))*self.reference_distance_from_wall + self.ranger_value_buffer:
                    self.wall_angle = (math.pi/2 - math.atan(front_range / right_range) + self.angle_value_buffer)
                elif left_range < 2*self.reference_distance_from_wall + self.ranger_value_buffer:
                    self.state = self.state_transition(self.StateWallFollowing.HOVER)
                else:
                    self.wall_angle = math.pi/12
                self.direction_value = 1
                self.calculate_arc()
                self.state = self.state_transition(self.StateWallFollowing.BOUNCING)

                
        elif self.state == self.StateWallFollowing.HOVER:
            print('hover')
        elif self.state == self.StateWallFollowing.BOUNCING:
            if front_range < 0.5*(self.reference_distance_from_wall + self.ranger_value_buffer) or \
                (self.time_now - self.state_start_time > 0.6*self.arc_time and \
                front_range < self.reference_distance_from_wall + self.ranger_value_buffer and \
                back_range < self.reference_distance_from_wall + self.ranger_value_buffer):
                    self.state = self.state_transition(self.StateWallFollowing.FIND_ESCAPE)
            if self.time_now - self.state_start_time > self.arc_time and \
                front_range > self.reference_distance_from_wall + self.ranger_value_buffer:
                self.state = self.state_transition(self.StateWallFollowing.FORWARD)

        elif self.state == self.StateWallFollowing.FIND_ESCAPE:
            if abs(left_range - right_range) < self.angle_value_buffer:
                self.state = self.state_transition(self.StateWallFollowing.FORWARD)

        elif self.state == self.StateWallFollowing.SCANNING:
            print('prev_front', self.prev_front, 'front', front_range, 'back', back_range)
            if self.scanned:
                if front_range > self.prev_front:
                    self.direction_value = -1 * self.direction_value
                self.scanned = False
                self.state = self.state_transition(self.StateWallFollowing.ROTATE__REFLECT)
            else:
                # self.direction_value = -1 * self.direction_value
                self.prev_front = front_range
                self.prev_back = 999
                self.scanned = True

        elif self.state == self.StateWallFollowing.ROTATE__REFLECT:
            if back_range > self.prev_back:
                self.state = self.state_transition(self.StateWallFollowing.FORWARD)
            else:
                if back_range < self.reference_distance_from_wall + self.ranger_value_buffer:
                    self.prev_back = back_range
            

        # -------------- Handle state actions ---------------- #
        command_velocity_x_temp = 0.0
        command_angle_rate_temp = 0.0

        if self.state == self.StateWallFollowing.FORWARD:
            command_velocity_x_temp = self.max_forward_speed
            command_angle_rate_temp = 0.0

        elif self.state == self.StateWallFollowing.HOVER:
            command_velocity_x_temp, command_angle_rate_temp = self.command_hover()

        elif self.state == self.StateWallFollowing.BOUNCING:
            command_velocity_x_temp, command_angle_rate_temp = self.command_circle(self.arc_radius, self.max_forward_speed)
            #command_velocity_x_temp = self.max_forward_speed
            #command_angle_rate_temp = self.direction_value * self.arc_rate

        elif self.state == self.StateWallFollowing.FIND_ESCAPE:
            command_velocity_x_temp, command_angle_rate_temp = self.command_turn(self.max_turn_rate)

        elif self.state == self.StateWallFollowing.SCANNING:
            command_velocity_x_temp, command_angle_rate_temp = self.command_turn(self.max_turn_rate)

        elif self.state == self.StateWallFollowing.ROTATE__REFLECT:
            command_velocity_x_temp, command_angle_rate_temp = self.command_turn(self.max_turn_rate)

        else:
            # state does not exist, so hover!
            command_velocity_x_temp, command_angle_rate_temp = self.command_hover()

        command_velocity_x = command_velocity_x_temp
        command_yaw_rate = command_angle_rate_temp

        return command_velocity_x, command_yaw_rate, self.state
