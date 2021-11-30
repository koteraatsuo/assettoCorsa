#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down

    TAB          : change sensor position
    `            : next sensor
    [1-9]        : change to sensor [1-9]
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc
from acs import *

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import threading
import types
from ctypes import *
import time


try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_o
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- observation ---------------------------------------------------------------
# ==============================================================================
class Observation2(object):
    def __init__(self, args):
        self.obs = {}
        self.obs["foward_wps"] = []

class Observation(object):
    def __init__(self, args):
        self.obs = {}
        self.obs["player"] = None
        self.obs["other_vehicles"] = {}
        self.obs["walkers"] = {}
        self.obs["velocity"] = None
        self.obs["speed"] = None
        self.obs["rpm"] = 0
        self.obs["hero_transform"] = None
        self.obs["target_dist"] = 0
        self.obs["throttle"] = 0
        self.obs["brake"] = 0
        self.obs["speed_limit"] = 0
        self.obs["acceleration"] = None
        self.obs["road_id"] = 0
        self.obs["lane_id"] = 0
        self.obs["last_w_road_id"] = 0
        self.obs["last_w_lane_id"] = 0
        self.obs["target_wp"] = None
        self.obs["target_speed"] = 0
        self.obs["corrent_wp"] = None
        self.obs["corrent_wp_type"] = None
        self.obs["waypoint_rs"] = None
        self.obs["waypoints"] = None
        self.obs["intersection_flag"]  = None
        self.obs["walker_danger_flag"]  = None
        self.obs["vehicle_danger_flag"] = None
        self.obs["avoid_vehicle_danger_flag"] = False
        self.obs["acs_avoid_vehicle_danger_flag"] = {}
        self.obs["acs_avoid_vehicle_danger_flag"]["foward"] = False
        self.obs["acs_avoid_vehicle_danger_flag"]["back"] = False
        self.obs["acs_avoid_vehicle_danger_flag"]["right"] = False
        self.obs["acs_avoid_vehicle_danger_flag"]["left"] = False
        self.obs["vehicle_danger_constant_velocityflag"] = None
        self.obs["traffic_light_danger_flag"] = None
        self.obs["oth_obs"] = None
        self.obs["corrent_waypoint"] = None
        self.obs["before_waypoints"] = None

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================
class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.actor_role_name = args.rolename
        self.hud = hud
        self.map = self.world.get_map()
        self.obs = Observation(None)
        self.obs2 = Observation2(None)
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        
    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')


        # Spawn the player.
        if self.obs.obs["player"] is not None:
            spawn_point = self.obs.obs["player"].get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            # self.destroy()
            self.obs.obs["player"] = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.obs.obs["player"] is None:
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            # spawn_point.location.x = 5
            # spawn_point.location.y = 91
            # spawn_point.location.z = 5
            # spawn_point.rotation.roll = 0.0
            # spawn_point.rotation.yaw = 179.0
            # spawn_point.rotation.pitch = 0.0
            self.obs.obs["player"] = self.world.try_spawn_actor(blueprint, spawn_point)
        
        # #対向車
        # for i in range (1):
        #     spawn_points = self.map.get_spawn_points()
        #     spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
        #     spawn_point.location.x = 56.2 
        #     spawn_point.location.y = 130
        #     spawn_point.location.z = 2
        #     spawn_point.rotation.roll = 0.0
        #     spawn_point.rotation.yaw = 180.0
        #     spawn_point.rotation.pitch = 0.0
        #     # blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        #     other_vehicle = self.world.try_spawn_actor(blueprint, spawn_point)
        #     if other_vehicle is not None:
        #         other_vehicle.set_autopilot(True)
           
        # # 停止車両
        # spawn_points = self.map.get_spawn_points()
        # spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
        # spawn_point.location.x = 1
        # spawn_point.location.y = 91
        # spawn_point.location.z = 5
        # spawn_point.rotation.roll = 0.0
        # spawn_point.rotation.yaw = 0.0
        # spawn_point.rotation.pitch = 0.0
        # # blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        # other_vehicle = self.world.try_spawn_actor(blueprint, spawn_point)
        # self.obs.obs["other_vehicles"].append(other_vehicle)

        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.obs.obs["player"], self.hud)
        self.imu_sensor = IMUSensor(self.obs.obs["player"])
        self.lane_invasion_sensor = LaneInvasionSensor(self.obs.obs["player"], self.hud)
        self.gnss_sensor = GnssSensor(self.obs.obs["player"])
        self.camera_manager = CameraManager(self.obs.obs["player"], self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        self.auto_cruise = AutoCruise(self.obs, self.obs2)
        self.control_vehicle = ControlVehicle(self.obs.obs["player"], self.obs)
        self.lidar = lidar(self.obs.obs["player"])

        actor_type = get_actor_display_name(self.obs.obs["player"])
        self.hud.obs = self.obs
        self.hud.notification(actor_type)

    def add_other_vehicle(self):
        # 他車をlist追加
        for vehicle in self.world.get_actors().filter('vehicle.*'):
            if self.obs.obs["player"] is not vehicle is not self.obs.obs["other_vehicles"]["other_vehicle"]:
                self.obs.obs["other_vehicles"]["other_vehicle"].append(vehicle)
        # walkerをlist追加
        for walker in self.world.get_actors().filter('walker.*'):
            if walker is not self.obs.obs["walkers"]:
                self.obs.obs["walkers"].append(walker)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.obs.obs["player"].get_world().set_weather(preset[0])

    def update_hero_status(self):
        self.obs.obs["velocity"] = self.obs.obs["player"].get_velocity()
        self.obs.obs["speed"] = math.sqrt(self.obs.obs["velocity"].x**2 + self.obs.obs["velocity"].y**2 + self.obs.obs["velocity"].z**2)
        self.obs.obs["compass"] = self.imu_sensor.compass
        self.obs.obs["hero_transform"]= self.obs.obs["player"].get_transform()
        self.obs.obs["corrent_wp"] = self.map.get_waypoint(self.obs.obs["hero_transform"].location)
        self.obs.obs["corrent_wp_type"] = self.obs.obs["corrent_wp"].lane_type
        self.obs.obs["acceleration"] = self.obs.obs["player"].get_acceleration()
        self.obs.obs["speed_limit"] = self.obs.obs["player"].get_speed_limit()/3.6
        self.obs.obs["other_vehicles"]["other_vehicle"] = []
        self.obs.obs["other_vehicles"]["transform"] = []
        self.obs.obs["other_vehicles"]["dist"] = []
        self.obs.obs["other_vehicles"]["before_dist"] = []
        self.obs.obs["other_vehicles"]["velocity"] = []
        self.obs.obs["other_vehicles"]["corrent_wp"] = []
        self.obs.obs["other_vehicles"]["speed"] = []
        self.obs.obs["walkers"]["walker"] = []
        self.obs.obs["walkers"]["transform"] = []
        self.obs.obs["walkers"]["velocity"] = []
        self.obs.obs["walkers"]["corrent_wp"] = []
        self.obs.obs["walkers"]["speed"] = []
        vehicles = self.world.get_actors().filter('vehicle.*')
        distance = lambda l: math.sqrt((l.x - self.obs.obs["hero_transform"].location.x)**2 + (l.y - self.obs.obs["hero_transform"].location.y)**2 + (l.z - self.obs.obs["hero_transform"].location.z)**2)
        vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != self.obs.obs["player"].id]
        for d, vehicle in sorted(vehicles):
            if d > 50.0:
                break
            self.obs.obs["other_vehicles"]["other_vehicle"].append(vehicle)
            self.obs.obs["other_vehicles"]["transform"].append(vehicle.get_transform())
            self.obs.obs["other_vehicles"]["velocity"].append(vehicle.get_velocity())
            dist = self.obs.obs["other_vehicles"]["transform"][len(self.obs.obs["other_vehicles"]["other_vehicle"])-1].location\
                 - self.obs.obs["hero_transform"].location
            self.obs.obs["other_vehicles"]["dist"].append(math.sqrt(dist.x**2 + dist.y**2))
            self.obs.obs["other_vehicles"]["before_dist"].append(self.obs.obs["other_vehicles"]["dist"][len(self.obs.obs["other_vehicles"]["dist"])-1])
            self.obs.obs["other_vehicles"]["corrent_wp"].append(self.map.get_waypoint(self.obs.obs["other_vehicles"]["transform"][len(self.obs.obs["other_vehicles"]["transform"])-1].location))
            self.obs.obs["other_vehicles"]["speed"].append(math.sqrt(self.obs.obs["other_vehicles"]["velocity"][len(self.obs.obs["other_vehicles"]["velocity"])-1].x**2 + self.obs.obs["other_vehicles"]["velocity"][len(self.obs.obs["other_vehicles"]["velocity"])-1].y**2 + self.obs.obs["other_vehicles"]["velocity"][len(self.obs.obs["other_vehicles"]["velocity"])-1].z**2))
        walkers = self.world.get_actors().filter('walker.*')
        distance = lambda l: math.sqrt((l.x - self.obs.obs["hero_transform"].location.x)**2 + (l.y - self.obs.obs["hero_transform"].location.y)**2 + (l.z - self.obs.obs["hero_transform"].location.z)**2)
        walkers = [(distance(x.get_location()), x) for x in walkers if x.id != self.obs.obs["player"].id]
        for d, walker in sorted(walkers):
            if d > 30.0:
                break
            self.obs.obs["walkers"]["walker"].append(walker)
            self.obs.obs["walkers"]["transform"].append(walker.get_transform())
            self.obs.obs["walkers"]["velocity"].append(walker.get_velocity())
            self.obs.obs["walkers"]["corrent_wp"].append(self.map.get_waypoint(self.obs.obs["walkers"]["transform"][len(self.obs.obs["walkers"]["transform"])-1].location))
            self.obs.obs["walkers"]["speed"].append(math.sqrt(self.obs.obs["walkers"]["velocity"][len(self.obs.obs["walkers"]["velocity"])-1].x**2 + self.obs.obs["walkers"]["velocity"][len(self.obs.obs["walkers"]["velocity"])-1].y**2 + self.obs.obs["walkers"]["velocity"][len(self.obs.obs["walkers"]["velocity"])-1].z**2))

    def tick(self, clock):
        self.update_hero_status()
        # self.frame = self.world.tick()
        self.hud.tick(self, clock)
        #self.lidar.tick()
        self.auto_cruise.tick()
        self.auto_cruise.render
        self.control_vehicle.tick()
        self.lidar.tick()
        
        # for vehicle in self.world.get_actors().filter('vehicle.*'):
        #     # draw Box
        #     transform = vehicle.get_transform()
        #     bounding_box = vehicle.bounding_box
        #     bounding_box.location += transform.location
        #     bounding_box.location += vehicle.get_velocity()/10
        #     self.world.debug.draw_box(bounding_box, transform.rotation, thickness=0.05,life_time=0.001)

    def render(self, display):
        self.camera_manager.render(display)
        # self.CV_CAMERA.render(display)
        self.hud.render(display)
        self.auto_cruise.render()
        self.lidar.render()

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.obs.obs["player"],
            self.lidar.sensor
            # self.CV_CAMERA.sensor
            ]
        for actor in actors:
            if actor is not None:
                actor.destroy()
        if self.obs.obs["other_vehicles"]["other_vehicle"] is not None:
            for vehicle in self.obs.obs["other_vehicles"]["other_vehicle"]:
                if vehicle is not None:
                    vehicle.destroy()

# ==============================================================================
# -- AutoCruise -----------------------------------------------------------------
# ==============================================================================
class AutoCruise(object):
    WAYPOINT_RATE = 3
    POINT_RANGE =10
    def __init__(self, obs ,obs2):
        self.obs = obs
        self.obs2 = obs2
        self.world = self.obs.obs["player"].get_world()
        self.ACS = ACS(self.world, self.obs, self.obs2)
        self.map = self.world.get_map()
        # self.waypoints = self.map.generate_waypoints(self.POINT_RANGE)
        # self.image_map = np.zeros((600, 600,3),dtype=np.uint8)
        # self.image_line = np.zeros((600, 600,3),dtype=np.uint8)
        self.corrent_waypoint = []
        self.next_waypoint = None
        self.before_waypoints = []
        self.hori_diff = None
        self.left_transform = carla.Transform ()
        self.right_transform = carla.Transform ()
        self.nlw = None
        self.nrw = None
        self.way_point_num = 1
        self.last_wp = None
        self.furing_cornering_flag = False
        self.count = 0
        self.lane_change_dir_both = False
        self.lane_change_dir_left = False
        self.lanechange_count = 0
        self.lanechange_random_count = 0
        self.right_foward_wps = []
        self.left_foward_wps = []
        self.num1 = 0
        self.target_mention_flag = True
        self.dir_flag = True
        self.oth_obs = [] 
        self.nw = self.map.get_waypoint(self.obs.obs["player"].get_transform().location)
        self.last_w_lane_id = self.nw.lane_id
        self.last_w_road_id = self.nw.road_id

        self.lane_change_flage = False
        self.first_nw_flage = True
    
    def tick(self):
        # if self.dir_flag == True:
        #     num = 0
        # else:
        #     num = -1
        
        # dx, dy = self.kaiten(self.hero_transform.rotation.yaw, num , 0)
        # 前方の経路判定
       
        self.obs.obs["intersection_flag"] = False
        self.obs.obs["waypoint_rs"] = []
        self.obs2.obs["foward_wps"] = []
        judge_wps = []
        
        # wpの間隔計算
        # interbal_dist = (self.obs.obs["speed"]/2)
        interbal_dist = (3)
        wp_cnt = int(self.obs.obs["speed"] * 3.6 /interbal_dist)

        waypoint_flage = False 
        if wp_cnt < 7:
            wp_cnt = 7
        for i in range(wp_cnt):
            temp_wps = self.obs.obs["corrent_wp"].next(i*interbal_dist+2)
            if len(temp_wps) > 1:
                waypoint_flage = True
                if self.way_point_num > len(temp_wps)-1:   
                    self.way_point_num = len(temp_wps)-1
                judge_wps2 = []
                for k in range(wp_cnt-i):
                    temp_wps2 = temp_wps[self.way_point_num].next(k*interbal_dist+2)
                    if k == 0:
                        temp_wp2 = temp_wps2[0]
                        judge_wps2.append(temp_wp2)
                    else:
                        temp_wp2 = temp_wps2[0]
                        judge_wps2.append(temp_wp2)
                        dya = judge_wps2[k-1].transform.rotation.yaw
                        x = judge_wps2[k].transform.location.x - judge_wps2[k-1].transform.location.x
                        y = judge_wps2[k].transform.location.y - judge_wps2[k-1].transform.location.y
                        L = math.sqrt(x**2 + y**2)
                        a = dya + math.degrees(math.atan2(-y, x))
                        num = 2*math.sin(math.radians(a))
                        if num == 0:
                            R = 9999999
                        else:
                            R = abs(L /num)
                        self.obs.obs["waypoint_rs"].append(R)
                        self.obs2.obs["foward_wps"].append(temp_wp2)
                        temp_wp2.transform.location.z += 0.5
                        self.world.debug.draw_point(temp_wp2.transform.location,color=carla.Color(255, 0, 255),life_time=0.01)
            
            if i == 0:
                temp_wp = temp_wps[0]
                judge_wps.append(temp_wp)
            else:
                if waypoint_flage is False:
                    temp_wp = temp_wps[0]
                    judge_wps.append(temp_wp)
                    dya = judge_wps[i-1].transform.rotation.yaw
                    x = judge_wps[i].transform.location.x - judge_wps[i-1].transform.location.x
                    y = judge_wps[i].transform.location.y - judge_wps[i-1].transform.location.y
                    L = math.sqrt(x**2 + y**2)
                    a = dya + math.degrees(math.atan2(-y, x))
                    num = 2*math.sin(math.radians(a))
                    if num == 0:
                        R = 9999999
                    else:
                        R = abs(L /num)
                    self.obs.obs["waypoint_rs"].append(R)
                    self.obs2.obs["foward_wps"].append(temp_wp)
                    temp_wp.transform.location.z += 0.5
                    self.world.debug.draw_point(temp_wp.transform.location,color=carla.Color(255, 0, 255),life_time=0.01)

        if self.obs.obs["speed"] < 15/3.6:
            self.obs.obs["target_dist"] = 5.5
        elif self.obs.obs["speed"] < 20/3.6:
            self.obs.obs["target_dist"] = 7.2
        elif self.obs.obs["speed"] < 25/3.6:
            self.obs.obs["target_dist"] = 0.9 * self.obs.obs["speed"] + 2.5
        elif self.obs.obs["speed"] < 40/3.6:
            self.obs.obs["target_dist"] = 0.9 * self.obs.obs["speed"]
        elif self.obs.obs["speed"] < 60/3.6: 
            self.obs.obs["target_dist"] = self.obs.obs["speed"]
        elif self.obs.obs["speed"] < 90/3.6: 
            self.obs.obs["target_dist"] = self.obs.obs["speed"]
        elif self.obs.obs["speed"] > 90/3.6: 
            self.obs.obs["target_dist"] = self.obs.obs["speed"]


        nws = self.obs.obs["corrent_wp"].next(self.obs.obs["target_dist"])
        if len(nws) > 1:
            if self.furing_cornering_flag == False:
                self.way_point_num = random.randrange(len(nws))
                # FIXME-CD 一時的に0を選ぶ
                if self.nw.is_intersection is True or self.nw.is_junction is True:
                    if self.last_w_lane_id is nws[self.way_point_num].lane_id and self.last_w_road_id  is  nws[self.way_point_num].road_id:
                        self.nw = nws[self.way_point_num]
                        self.last_wp = self.nw
                        self.last_w_lane_id = self.nw.lane_id 
                        self.last_w_road_id = self.nw.road_id
                    elif self.last_wp.next(self.obs.obs["target_dist"])[0].lane_id == nws[0].lane_id and self.last_wp.next(self.obs.obs["target_dist"])[0].road_id == nws[0].road_id:
                        self.nw = nws[self.way_point_num]
                        self.last_wp = self.nw
                        self.last_w_lane_id = self.nw.lane_id 
                        self.last_w_road_id = self.nw.road_id
                    else:
                        self.nw = self.last_wp.next(self.obs.obs["target_dist"])[self.way_point_num]
                else: 
                    self.nw = nws[self.way_point_num]
                    self.last_wp = self.nw
                    self.last_w_lane_id = self.nw.lane_id 
                    self.last_w_road_id = self.nw.road_id
            else:
                # if len(nws) >= self.way_point_num:
                #     self.way_point_num = 0
                self.nw = nws[self.way_point_num]
                self.last_wp = self.nw
                self.last_w_lane_id = self.nw.lane_id 
                self.last_w_road_id = self.nw.road_id
            self.dir_flag = False
            self.count = 0
        else:
            if self.obs.obs["corrent_wp"].is_intersection is True or self.obs.obs["corrent_wp"].is_junction is True:
                
                if self.last_w_lane_id is nws[0].lane_id and self.last_w_road_id is nws[0].road_id:                    
                    self.nw = nws[0]
                    self.last_wp = self.nw
                    self.last_w_lane_id = self.nw.lane_id 
                    self.last_w_road_id = self.nw.road_id
                elif self.last_wp.next(self.obs.obs["target_dist"])[0].lane_id == nws[0].lane_id and self.last_wp.next(self.obs.obs["target_dist"])[0].road_id == nws[0].road_id :
                    self.nw = nws[0]
                    self.last_wp = self.nw
                    self.last_w_lane_id = self.nw.lane_id 
                    self.last_w_road_id = self.nw.road_id
                else:
                    self.nw = self.last_wp.next(self.obs.obs["target_dist"])[0]
            else:
                self.nw = nws[0]
                self.last_wp = self.nw
                self.last_w_lane_id = self.nw.lane_id 
                self.last_w_road_id = self.nw.road_id
            self.dir_flag = True
        self.count += 1

        if self.first_nw_flage is True:
            self.obs.obs["target_wp"] = self.nw
            self.first_nw_flage = False

        for foward_wp in self.obs2.obs["foward_wps"]:
            if foward_wp.is_intersection is True or foward_wp.is_junction is True:
                self.obs.obs["intersection_flag"] = True

        # ロード情報更新
        self.obs.obs["road_id"] = self.nw.road_id 
        self.obs.obs["lane_id"] = self.nw.lane_id
        self.obs.obs["last_w_road_id"] = self.last_w_road_id
        self.obs.obs["last_w_lane_id"] = self.last_w_lane_id

        # ターゲットWPの点表示
        self.world.debug.draw_point(self.nw.transform.location,color=carla.Color(255, 0, 0),life_time=0.01)
        
        # 次のジャンクションでランダムで変更するか決める
        if self.count > 40:
            self.furing_cornering_flag = False
        else:
            self.furing_cornering_flag = True      
        
        # 信号判定
        self.obs.obs["traffic_light_danger_flag"] = False
        if self.obs.obs["player"].is_at_traffic_light():
            tl = self.obs.obs["player"].get_traffic_light()
            for x in tl.get_group_traffic_lights():
                if x.get_pole_index() == tl.id:
                    state = carla.TrafficLightState.Green
                else:
                    state = carla.TrafficLightState.Red
            if state == carla.TrafficLightState.Red:
                self.obs.obs["traffic_light_danger_flag"] = True
        
        # FIXME-CD 前方に他社がある時、車のデンジャーフラグを立てる
        self.obs.obs["vehicle_danger_constant_velocityflag"] = False
        self.obs.obs["avoid_vehicle_danger_flag"] = False
        self.obs.obs["acs_avoid_vehicle_danger_flag"]["foward"] = False
        self.obs.obs["acs_avoid_vehicle_danger_flag"]["back"] = False
        self.obs.obs["acs_avoid_vehicle_danger_flag"]["right"] = False
        self.obs.obs["acs_avoid_vehicle_danger_flag"]["left"] = False       
        self.obs.obs["vehicle_danger_flag"] = False
        self.obs.obs["walker_danger_flag"] = False
        self.obs.obs["oth_obs"] = []


        for num in range (len(self.obs.obs["other_vehicles"]["other_vehicle"])):
            oth_v = self.obs.obs["other_vehicles"]["velocity"][num]
            oth_v = math.sqrt(oth_v.x**2 + oth_v.y**2 + oth_v.z**2)
            
            count = 0
            for i in range (len(self.obs2.obs["foward_wps"]) - 1):               
                if oth_v > 1:
                    dists = self.obs.obs["other_vehicles"]["transform"][num].location - self.obs2.obs["foward_wps"][i].transform.location
                    if math.sqrt(dists.x**2 + dists.y**2) < self.obs2.obs["foward_wps"][i].lane_width/2 :
                        self.obs.obs["vehicle_danger_constant_velocityflag"]= True
                        self.lanechange_count += 1
                        #　レーンチェンジを開始するかどうか？
                        self.obs.obs["oth_obs"].append(oth_v)
                else:
                    dists = self.obs.obs["other_vehicles"]["transform"][num].location - self.obs2.obs["foward_wps"][i].transform.location
                    player_dist = self.obs.obs["other_vehicles"]["transform"][num].location - self.obs.obs["hero_transform"].location
                    px, py = self.rotation(math.radians(self.obs.obs["compass"]), player_dist.x, -player_dist.y)
                    if math.sqrt(dists.x**2 + dists.y**2) < self.obs2.obs["foward_wps"][i].lane_width + 0.5:
                        count += 1
                        if count < 6:
                            self.obs.obs["avoid_vehicle_danger_flag"] = True
                        if math.sqrt(player_dist.x**2 + player_dist.y**2) < 10 and py > 0 and -1.75 < px and px < 1.75 :
                            self.obs.obs["vehicle_danger_flag"] = True
    

        for num in range (len(self.obs.obs["walkers"]["walker"])):             
            player_dist = self.obs.obs["walkers"]["transform"][num].location - self.obs.obs["hero_transform"].location
            px, py = self.rotation(math.radians(self.obs.obs["compass"]), player_dist.x, -player_dist.y)
            if math.sqrt(player_dist.x**2 + player_dist.y**2) < 10 and py > 0 and -1.75 < px and px < 1.75 :
                self.obs.obs["waker_danger_flag"] = True
                
        self.ACS.tick()

        #　どのくらい前に車がいるか、どのくらい同じ車線にいるか
        self.lanechange_random_count += 1
        if 20 < self.lanechange_count or self.lanechange_random_count > 500:
            self.lanechange_random_count = 0
            self.obs.obs["avoid_vehicle_danger_flag"] = True
            self.lane_change_dir_both = True
        
        # 車がレーンチェンジ終わりか？
        if  20 < self.lanechange_count and self.obs.obs["target_wp"].lane_id == self.obs.obs["lane_id"] and self.lane_change_flage is True:
            self.lane_change_flage = False
            self.vehicle_danger_lane_change_flag = True
            self.obs.obs["avoid_vehicle_danger_flag"] = False
            self.lanechange_count = 0

        if self.obs.obs["acs_avoid_vehicle_danger_flag"]["right"] == True:
            self.lane_change_dir_left = True
        if self.obs.obs["acs_avoid_vehicle_danger_flag"]["left"] == True:
            self.lane_change_dir_left = False
        # # 周りの車検知
        # for other_vehicle in self.obs.obs["other_vehicles"]: 
        #     dist = other_vehicle.get_transform().location - self.obs.obs["player"].get_transform().location

        #     if self.obs.obs["speed"] * 3.6 < 40:
        #         check_dist = 11
        #     elif self.obs.obs["speed"] * 3.6 < 72:
        #         check_dist = self.obs.obs["speed"]
        #     else:
        #         check_dist = 20
        
        #     if math.sqrt(dist.x**2 + dist.y**2) < check_dist and math.sqrt(dist.x**2 + dist.y**2) != 0:                      
        #         self.obs.obs["avoid_vehicle_danger_flag"]  = False
            
        #　レーンチェンジ（前方に他車がいた場合）
        # 車がレーンチェンジ開始かか？
        self.obs.obs["target_wp"] = self.nw
        if (self.obs.obs["acs_avoid_vehicle_danger_flag"]["back"] == True or self.obs.obs["acs_avoid_vehicle_danger_flag"]["foward"] == True or self.obs.obs["avoid_vehicle_danger_flag"] is True )and self.obs.obs["intersection_flag"] == False:
            # TODOなんでかけてるんの？忘れた。
            if self.obs.obs["corrent_wp"].get_right_lane() is not None:
                right_wp = self.obs.obs["corrent_wp"].get_right_lane()
                if (right_wp.lane_id*self.obs.obs["corrent_wp"].lane_id >= 0 and\
                    right_wp.lane_type == carla.LaneType.Driving and \
                    self.lane_change_dir_left is False) or \
                    (right_wp.lane_id*self.obs.obs["corrent_wp"].lane_id >= 0 and\
                    right_wp.lane_type == carla.LaneType.Driving and self.lane_change_dir_both is True) :
                    self.right_foward_wps = []
                    for i in range(wp_cnt):
                        self.right_foward_wps.append(right_wp.next(i*3.5+0.1))
                    for right in self.right_foward_wps:
                        self.world.debug.draw_point(right[0].transform.location,color=carla.Color(47, 210, 231),life_time=0.01)
                    
                    temp_flag = False
                    for num in range(len(self.obs.obs["other_vehicles"]["other_vehicle"])):
                        for right_foward_wp in self.right_foward_wps:
                            dists = self.obs.obs["other_vehicles"]["transform"][num].location - right_foward_wp[0].transform.location
                            if math.sqrt(dists.x**2 + dists.y**2) < right_foward_wp[0].lane_width/2 :
                                temp_flag = True
                    if temp_flag is False:
                        nrws = right_wp.next(self.obs.obs["target_dist"]*1.5)
                        if self.obs.obs["acs_avoid_vehicle_danger_flag"]["back"] == True or self.obs.obs["acs_avoid_vehicle_danger_flag"]["foward"] == True:
                            # self.obs.obs["target_dist"]*0.75
                            nrws = right_wp.next(self.obs.obs["target_dist"]*0.6 + 0.1 if self.obs.obs["target_dist"]>11.11 else 6.6)
                        self.nrw = nrws[0]
                        self.obs.obs["target_wp"] = self.nrw
                        self.vehicle_danger_lane_change_flag = True
                        self.world.debug.draw_point(self.nrw.transform.location,color=carla.Color(255, 0, 0),life_time=0.01)

            if self.obs.obs["corrent_wp"].get_left_lane() is not None:
                left_wp = self.obs.obs["corrent_wp"].get_left_lane()
                if (left_wp.lane_id*self.obs.obs["corrent_wp"].lane_id >= 0 and\
                    left_wp.lane_type == carla.LaneType.Driving and\
                    self.lane_change_dir_left is True) or\
                    (left_wp.lane_id*self.obs.obs["corrent_wp"].lane_id >= 0 and\
                    left_wp.lane_type == carla.LaneType.Driving and self.lane_change_dir_both is True):
                    self.left_foward_wps = []
                    for i in range(wp_cnt):
                        self.left_foward_wps.append(left_wp.next(i*3.5+0.1))
                    for left in self.left_foward_wps:
                        self.world.debug.draw_point(left[0].transform.location,color=carla.Color(47, 210, 231),life_time=0.01)

                    temp_flag = False
                    for num in range(len(self.obs.obs["other_vehicles"]["other_vehicle"])):
                        for left_foward_wp in self.left_foward_wps:
                            dists = self.obs.obs["other_vehicles"]["transform"][num].location - left_foward_wp[0].transform.location
                            if math.sqrt(dists.x**2 + dists.y**2) < left_foward_wp[0].lane_width/2 :
                                temp_flag = True
                    if temp_flag is False:
                        nlws = left_wp.next(self.obs.obs["target_dist"]*1.5)
                        if self.obs.obs["acs_avoid_vehicle_danger_flag"]["back"] == True or self.obs.obs["acs_avoid_vehicle_danger_flag"]["foward"] == True:
                            nlws = left_wp.next(self.obs.obs["target_dist"]*0.6 + 0.1 if self.obs.obs["target_dist"]>11.11 else 6.6)
                        self.nlw = nlws[0]
                        self.obs.obs["target_wp"] = self.nlw
                        self.vehicle_danger_lane_change_flag = True
                        self.world.debug.draw_point(self.nlw.transform.location,color=carla.Color(255, 0, 0),life_time=0.01)


        # 車がレーンチェンジ中か？
        if  self.obs.obs["target_wp"].lane_id != self.obs.obs["lane_id"] and self.vehicle_danger_lane_change_flag is True:
            self.lane_change_flage = True

        # OpenCV表示

        # self.image_map = np.zeros((600, 600,3),dtype=np.uint8)
        # for waypoint in self.waypoints:
        #     cv2.line(self.image_map, (int(waypoint.transform.location.x+300), int(waypoint.transform.location.y+300)), (int(waypoint.transform.location.x + 300), int(waypoint.transform.location.y + 300)), (255, 255, 255), 5)
        # cv2.circle(self.image_map, (int(self.obs.obs["hero_transform"].location.x+300), int(self.obs.obs["hero_transform"].location.y + 300)), 2, color=(0, 0, 255), thickness=2)


        # self.image_line = np.zeros((600, 600,3),dtype=np.uint8)
        # self.world.debug.draw_point(self.nw.transform.location,color=carla.Color(47, 210, 231),life_time=0.01)
        # dp = self.obs.obs["hero_transform"].rotation.pitch
        # dya = self.obs.obs["hero_transform"].rotation.yaw
        # dr = self.obs.obs["hero_transform"].rotation.roll
        # nx = self.nw.transform.location.x - self.obs.obs["hero_transform"].location.x 
        # ny = self.nw.transform.location.y - self.obs.obs["hero_transform"].location.y 
        # nz = self.nw.transform.location.z - self.obs.obs["hero_transform"].location.z
        # nx, ny = self.kaiten(math.radians(dya), nx, ny)
        # ny = self.nw.transform.location.y - self.obs.obs["hero_transform"].location.y 
        # nz = self.nw.transform.location.z - self.obs.obs["hero_transform"].location.z
        # dx = -ny 
        # dy = -nx
        # self.hori_diff = dx
        # dx *= -20 
        # dy *= 20
        # bounding_box = self.obs.obs["player"].bounding_box
        # bounding_box.extent = bounding_box.extent * 20
        # cv2.rectangle(self.image_line,(int(bounding_box.extent.y) +300 ,int(bounding_box.extent.x) + 300),(int(-bounding_box.extent.y) +300, int(-bounding_box.extent.x) +300),(0,255,0),1)
        # cv2.line(self.image_line, (int(300), int(300)), (int(300), int(300)), (255, 255, 255), 5)
        # cv2.line(self.image_line, (int(dx+300), int(dy+300)), (int(dx + 300), int(dy + 300)), (0, 0, 255), 5)

        

    def render(self):
        a = 1
        # cv2.imshow('MAP',self.image_map)
        # cv2.waitKey(1)
        # cv2.imshow('LINE',self.image_line)
        # cv2.waitKey(1)
    
    def kaiten(self, r, l1, l2):
        n1 = l1 * math.cos(r) + l2 *math.sin(r)
        n2 = l1 * math.sin(r) - l2 *math.cos(r)
        return n1, n2

    def rotation(self, r, l1, l2):
        n1 = l1 * math.cos(r) - l2 *math.sin(r)
        n2 = l1 * math.sin(r) + l2 *math.cos(r)
        return n1, n2
        
    def kaiten2(self, r, l1, l2):
        n1 = l1 * math.cos(r) - l2 *math.sin(r)
        n2 = l1 * math.sin(r) - l2 *math.cos(r)
        return n1, n2
        
    def kaiten3(self, x, y, z, p, w, r):
        z, x = self.kaiten2(math.radians(p), z, x)
        x, y = self.kaiten2(math.radians(w), x, y)
        y, z = self.kaiten2(math.radians(r), y, z)
        return x, y, z

# ==============================================================================
# -- ControlVehicle ----------------------------------------------------------------
# ==============================================================================

class ControlVehicle(object):
    STEER_ANGLE_RATO = 1.6
    def __init__(self,actor, obs):
        self.obs = obs
        self._control = carla.VehicleControl()
        self.right_count = 0
        self.left_count = 0
        self.hori_diff = None
        self.velocity = None
        self.acceleration = None
        self.steer_angle = 0
        self.pidaccelcontrol = PidAccelControl()
        self.brake_control = BrakeControl()
        self._lights = carla.VehicleLightState.NONE
        self.oth_obs = {}


    def control(self):
        self._control.throttle = 1.0
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0
        self._control.hand_brake = keys[K_SPACE]
    
    def kaiten(self, r, l1, l2):
        n1 = l1 * math.cos(r) + l2 *math.sin(r)
        n2 = l1 * math.sin(r) - l2 *math.cos(r)
        return n1, n2

    def cal_gear_rpm(self):
        if self.obs.obs["speed"] < 15/3.6:
            gear = 1
        elif self.obs.obs["speed"] < 35/3.6:
            gear = 2
        elif self.obs.obs["speed"] < 55/3.6:
            gear = 3
        elif self.obs.obs["speed"] < 75/3.6:
            gear = 4
        else:
            gear = 5

        rpm = 0
        if gear == 1:
            num1 = self.obs.obs["speed"]
            num2 = 4 * 0.00218188165
            rpm = num1 / num2
        if gear == 2:
            num1 = self.obs.obs["speed"]
            num2 = 4*2 * 0.00218188165
            rpm = num1 / num2
        if gear == 3:
            num1 = self.obs.obs["speed"]
            num2 = 4*2.667 * 0.00218188165
            rpm = num1 / num2
        if gear == 4:
            num1 = self.obs.obs["speed"]
            num2 = 4*3.636 * 0.00218188165
            rpm = num1 / num2
        if gear == 5:
            num1 = self.obs.obs["speed"]
            num2 = 4 * 4 * 0.00218188165
            rpm = num1 / num2
        self.obs.obs["rpm"] =rpm
        return gear, rpm

    def tick(self):

        dp = self.obs.obs["hero_transform"].rotation.pitch
        dya = self.obs.obs["hero_transform"].rotation.yaw
        dr = self.obs.obs["hero_transform"].rotation.roll
        nx = self.obs.obs["target_wp"].transform.location.x - self.obs.obs["hero_transform"].location.x 
        ny = self.obs.obs["target_wp"].transform.location.y - self.obs.obs["hero_transform"].location.y 
        nz = self.obs.obs["target_wp"].transform.location.z - self.obs.obs["hero_transform"].location.z
        
        
        L = math.sqrt(nx**2 + ny**2)
        a = dya + math.degrees(math.atan2(-ny, nx))
        num = 2*math.sin(math.radians(a))
        R = abs(L /num)
        w = 3
        target_steer_angle = (math.degrees(math.atan2(-w,R)) / 69.999992)* self.STEER_ANGLE_RATO
        if num < 0:
            target_steer_angle = -target_steer_angle
        if  self.obs.obs["acs_avoid_vehicle_danger_flag"]["foward"] is True or self.obs.obs["acs_avoid_vehicle_danger_flag"]["back"] is True:
            self.steer_angle = target_steer_angle
        elif target_steer_angle - self.steer_angle  < -1.5/ 69.999992 and self.obs.obs["avoid_vehicle_danger_flag"] is True:
            self.steer_angle -= 1.5/ 69.999992
        elif target_steer_angle - self.steer_angle  > 1.5/ 69.999992 and self.obs.obs["avoid_vehicle_danger_flag"] is True:
            self.steer_angle += 1.5/ 69.999992
        elif target_steer_angle - self.steer_angle  < -1.5/ 69.999992:
            self.steer_angle -= 4/ 69.999992
        elif target_steer_angle - self.steer_angle  > 1.5/ 69.999992:
            self.steer_angle += 4/ 69.999992
        else:
            self.steer_angle = target_steer_angle
        self._control.steer = self.steer_angle
        self._control.manual_gear_shift = True

        # gear_rpm計算    
        self._control.gear , _ = self.cal_gear_rpm()

        # R_danger_flag = False
        self.obs.obs["throttle"] = 0
        self.obs.obs["brake"] = 0
        if self.obs.obs["walker_danger_flag"]  == True or self.obs.obs["vehicle_danger_flag"]== True \
            or self.obs.obs["acs_avoid_vehicle_danger_flag"]["foward"] is True or\
            self.obs.obs["acs_avoid_vehicle_danger_flag"]["foward"] is True:    
            self.obs.obs["target_speed"] = 0
        elif self.obs.obs["acs_avoid_vehicle_danger_flag"]["back"] is True:
            self.obs.obs["target_speed"] = 100/3
        elif self.obs.obs["traffic_light_danger_flag"] == True:    
            self.obs.obs["target_speed"] = 0
        elif self.obs.obs["vehicle_danger_constant_velocityflag"] == True:
            min_speed = 1000
            for oth_v in self.obs.obs["oth_obs"]:
                if min_speed > oth_v:
                    min_speed = oth_v
            self.obs.obs["target_speed"] = min_speed-2/3.6
        elif self.obs.obs["intersection_flag"] == True and self.obs.obs["speed_limit"] < 35/3.6:
            self.obs.obs["target_speed"] = 5.5556
        else:
            R = 1000000.0
            for waypoint_r in self.obs.obs["waypoint_rs"]:
                if waypoint_r < R:
                    R = waypoint_r  
            self.obs.obs["target_speed"] = math.sqrt(R*2.)
  
            if self.obs.obs["target_speed"] > self.obs.obs["speed_limit"]:
                self.obs.obs["target_speed"] = self.obs.obs["speed_limit"]


        self._control.throttle = 0
        self._control.brake = 0
        self._control.brake = self.brake_control.cal(self.obs) 
        self.obs.obs["throttle"] = self.pidaccelcontrol.pid_throttle_control(self.obs.obs["target_speed"], self.obs.obs["velocity"], self.obs.obs["acceleration"],self.obs)       
        self._control.throttle = self.obs.obs["throttle"]
        # if self._control.brake:
        #     self._lights |= carla.VehicleLightState.Brake
        # else: # Remove the Brake flag
        #     self._lights &= carla.VehicleLightState.All ^ carla.VehicleLightState.Brake
        # if self._control.reverse:
        #     self._lights |= carla.VehicleLightState.Reverse
        # else: # Remove the Reverse flag
        #     self._lights &= carla.VehicleLightState.All ^ carla.VehicleLightState.Reverse
        # self.obs.obs["player"].set_light_state(carla.VehicleLightState(self._lights))

        self.obs.obs["player"].apply_control(self._control)

# ==============================================================================
# -- BrakeControl --------------------------------------------------------------
# ==============================================================================
class BrakeControl(object):
    def __init__(self):
        self.target_brake = 0.4
        self.brake = 0.0
        self.target_speed = 0

    def cal(self, obs):
        if obs.obs["walker_danger_flag"]  == True:
            self.target_brake = 0.0    
            self.brake = 1
        elif obs.obs["acs_avoid_vehicle_danger_flag"]["foward"] == True:
            self.target_brake = 0.0
            self.brake = 0.6
        elif obs.obs["vehicle_danger_flag"] == True:
            self.target_brake = 0.0    
            self.brake = 0.3
        elif obs.obs["traffic_light_danger_flag"] == True:
            self.target_brake = 0.0
            self.brake = 0.45
        elif obs.obs["acs_avoid_vehicle_danger_flag"]["back"] == True:
            self.target_brake = 0.0
            self.brake = 0
        elif obs.obs["vehicle_danger_constant_velocityflag"] == True and (obs.obs["speed"] *3.6- obs.obs["target_speed"] *3.6) > 20:
            self.target_brake = 0.0    
            self.brake = 1
        elif obs.obs["vehicle_danger_constant_velocityflag"] == True and (obs.obs["speed"] *3.6- obs.obs["target_speed"] *3.6) > 10:
            self.target_brake = 0.0    
            self.brake = 0.6
        elif obs.obs["vehicle_danger_constant_velocityflag"] == True and (obs.obs["speed"] *3.6- obs.obs["target_speed"] *3.6) > 5:
            self.target_brake = 0.0    
            self.brake = 0.3
        
        if obs.obs["speed"] *3.6 >= obs.obs["target_speed"] *3.6:
            speed_diff = abs(obs.obs["speed"] *3.6 - obs.obs["target_speed"] *3.6)
            if speed_diff > 30:
                self.target_brake = 0.8
            elif speed_diff > 20:
                self.target_brake = 0.65
            elif speed_diff > 15:
                self.target_brake = 0.5
            elif speed_diff > 10:
                self.target_brake = 0.3
            elif speed_diff >= 5:
                self.target_brake = 0.1
            elif obs.obs["target_speed"] *3.6 <= 5:
                self.target_brake = 0.2
            else:
                self.target_brake = 0.

            if self.target_brake > self.brake:
                self.brake += 0.02 
                if self.target_brake < self.brake:
                    self.brake = self.target_brake
        else:
            self.brake = 0
            self.target_brake = 0.0


        obs.obs["brake"] = self.brake
        return obs.obs["brake"]
 


# ==============================================================================
# -- PidAccelControl------------------------------------------------------------
# ==============================================================================
class PidAccelControl(object):
    def __init__(self):
        self.Kp = 0.4
        self.Ki = 0.0016
        self.Kd = 0.1
        self.accumulation = []
        self.target_speed_diff_count = 0
        self.target_throttle = 0
        self.throttle = 0

    def kaiten(self, r, l1, l2):
        n1 = l1 * math.cos(r) + l2 *math.sin(r)
        n2 = l1 * math.sin(r) - l2 *math.cos(r)
        return [n1, n2]

    def pid_throttle_control(self, target_speed ,velocity, acceleration, obs):
        corrent_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        corrent_acceleration = math.sqrt(acceleration.x**2 + acceleration.y**2 + acceleration.z**2)
        acceleration_xy = self.kaiten(corrent_acceleration, acceleration.x, acceleration.x)
        rad = math.atan2(acceleration_xy[1], acceleration_xy[0])
        x_acceleration = corrent_acceleration * math.cos(rad)
        gap_speed = target_speed - corrent_speed

        if (gap_speed * 3.6) > 5:
            self.target_speed_diff_count += 1
        else:
            self.target_speed_diff_count = 0

        
        self.accumulation.append(gap_speed)
        if len(self.accumulation) > 5:
            self.accumulation.pop(0)

        self.target_throttle = self.Kp * gap_speed + self.Ki * sum(self.accumulation) - self.Kd * x_acceleration + 0.23 * math.log(corrent_speed + 1)


        if self.target_throttle > self.throttle:
            self.throttle += 0.01
        elif self.target_throttle < self.throttle:
            self.throttle -= 0.01      
        # TODO関数化

        if corrent_speed < 1/3.6:
            throttle_limit = 0.2
        elif corrent_speed < 10/3.6:
            throttle_limit = 0.45
        elif corrent_speed < 30/3.6:
            throttle_limit = 0.5
        elif corrent_speed < 60/3.6:
            throttle_limit = 0.8
        else:
            throttle_limit = 1.0            

        if self.throttle > throttle_limit:
            self.throttle = throttle_limit
        elif self.throttle <= 0:
            self.throttle = 0.0

        # スロットルリミットでスピードで上がらない場合、2秒
        if self.target_speed_diff_count > 10:
            self.throttle += 0.002*self.target_speed_diff_count
            if self.throttle >= 1:
                self.throttle = 1
        if obs.obs["walker_danger_flag"]  == True:
            self.throttle = 0.0    
        elif obs.obs["acs_avoid_vehicle_danger_flag"]["foward"] == True:
            self.throttle = 0.0
        elif obs.obs["vehicle_danger_flag"] == True:
            self.throttle = 0.0    
        elif obs.obs["traffic_light_danger_flag"] == True:
            self.throttle = 0.0
        if obs.obs["vehicle_danger_constant_velocityflag"] == True and (obs.obs["speed"] *3.6- obs.obs["target_speed"] *3.6) > 20:
            self.throttle = 0.0    
        elif obs.obs["vehicle_danger_constant_velocityflag"] == True and (obs.obs["speed"] *3.6- obs.obs["target_speed"] *3.6) > 10:
            self.throttle = 0.0    
        elif obs.obs["vehicle_danger_constant_velocityflag"] == True and (obs.obs["speed"] *3.6- obs.obs["target_speed"] *3.6) > 5:
            self.throttle = 0.0
        return self.throttle
 
# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.obs.obs["player"], carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            world.obs.obs["player"].set_autopilot(self._autopilot_enabled)
        elif isinstance(world.obs.obs["player"], carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.obs.obs["player"].get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock):
        current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_o:
                    world.add_other_vehicle()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    currentIndex = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.obs.obs["player"].set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(currentIndex)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                    current_lights ^= carla.VehicleLightState.Special1
                elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                    current_lights ^= carla.VehicleLightState.HighBeam
                elif event.key == K_l:
                    # Use 'L' key to switch between lights:
                    # closed -> position -> low beam -> fog
                    if not self._lights & carla.VehicleLightState.Position:
                        world.hud.notification("Position lights")
                        current_lights |= carla.VehicleLightState.Position
                    else:
                        world.hud.notification("Low beam lights")
                        current_lights |= carla.VehicleLightState.LowBeam
                    if self._lights & carla.VehicleLightState.LowBeam:
                        world.hud.notification("Fog lights")
                        current_lights |= carla.VehicleLightState.Fog
                    if self._lights & carla.VehicleLightState.Fog:
                        world.hud.notification("Lights off")
                        current_lights ^= carla.VehicleLightState.Position
                        current_lights ^= carla.VehicleLightState.LowBeam
                        current_lights ^= carla.VehicleLightState.Fog
                elif event.key == K_i:
                    current_lights ^= carla.VehicleLightState.Interior
                elif event.key == K_z:
                    current_lights ^= carla.VehicleLightState.LeftBlinker
                elif event.key == K_x:
                    current_lights ^= carla.VehicleLightState.RightBlinker

                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.obs.obs["player"].get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not (pygame.key.get_mods() & KMOD_CTRL):
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.obs.obs["player"].set_autopilot(self._autopilot_enabled)
                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
        if not self._autopilot_enabled:
            if self._control.brake:
                current_lights |= carla.VehicleLightState.Brake
            else: # Remove the Brake flag
                current_lights &= carla.VehicleLightState.All ^ carla.VehicleLightState.Brake
            if self._control.reverse:
                current_lights |= carla.VehicleLightState.Reverse
            else: # Remove the Reverse flag
                current_lights &= carla.VehicleLightState.All ^ carla.VehicleLightState.Reverse
            if current_lights != self._lights: # Change the light state only if necessary
                self._lights = current_lights
                world.obs.obs["player"].set_light_state(carla.VehicleLightState(self._lights))
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            # world.obs.obs["player"].apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 3.333 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self.obs = None
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)

        if not self._show_info:
            return
        t = world.obs.obs["player"].get_transform()
        v = world.obs.obs["player"].get_velocity()
        c = world.obs.obs["player"].get_control()
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.obs.obs["player"], truncate=20),
            'Map:     % 20s' % world.map.name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'rpm:     % 16.1f rpm' % self.obs.obs["rpm"],
            'speed_limit: % 12.3f km/h' % (self.obs.obs["speed_limit"]*3.6),
            'target_speed:% 12.3f km/h' % (self.obs.obs["target_speed"]*3.6),
            'Speed:   % 15.3f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            'G    :   % 15.3f m/s2' % (math.sqrt(self.obs.obs["acceleration"].x**2 + self.obs.obs["acceleration"].y**2 + self.obs.obs["acceleration"].z**2)),
            'throttle:   % 12.3f  ' % (self.obs.obs["throttle"]*100),
            'brake:   % 15.3f     ' % (self.obs.obs["brake"]*100),
            'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            'intersection_flag:        % 4s     ' % (self.obs.obs["intersection_flag"]),
            'walker_danger_flag:       % 4s     ' % (self.obs.obs["walker_danger_flag"]),
            'vehicle_danger_flag:      % 4s     ' % (self.obs.obs["vehicle_danger_flag"]),
            'acs_av_veh_danger_flag:', 
            ' % 20s' % (self.obs.obs["acs_avoid_vehicle_danger_flag"]),
            'traffic_light_danger_flg: % 4s     ' % (self.obs.obs["traffic_light_danger_flag"]),
            'v_danger_constant_veflag: % 4s     ' % (self.obs.obs["vehicle_danger_constant_velocityflag"]),
            'road_id:       % 14.0f     ' % (self.obs.obs["road_id"]),
            'lane_id:       % 14.0f     ' % (self.obs.obs["lane_id"]),
            'last_w_road_id:   % 11.0f' % (self.obs.obs["last_w_road_id"]),
            'last_w_lane_id:   % 11.0f' % (self.obs.obs["last_w_lane_id"]),
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                # ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                # ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.obs.obs["player"].id]
            for d, vehicle in sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))


    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)

# ==============================================================================
# -- Lidar ---------------------------------------------------------------------
# ==============================================================================
class lidar(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        self.world = self._parent.get_world()
        bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
        bp.set_attribute('range', str(6000))
        bp.set_attribute('points_per_second', str(36000)) # 1440*20
        bp.set_attribute('rotation_frequency', str(30))
        bp.set_attribute('upper_fov', str(90))
        bp.set_attribute('lower_fov', str(-1))
        bp.set_attribute('sensor_tick', str(0))
        self.sensor = self.world.spawn_actor(bp, carla.Transform(carla.Location(x = 0, y = 0, z = 1.5)), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: lidar.on_invasion(self, weak_self, event))
        self.points = None
        self.image_lidar = None
        self.points_location = carla.Location()

    def tick(self):
        self.image_lidar = np.zeros((512, 512,3),dtype=np.uint8)
        # for pt in self.points:
        #     cv2.circle(self.image_lidar, (int(pt[0]*5+ 256), int(pt[1]*5 + 256)), 1, color=(255, 255, 255), thickness=1)
        # bounding_box = self._parent.bounding_box
        # bounding_box.extent = bounding_box.extent * 5
        # cv2.rectangle(self.image_lidar,(int(bounding_box.extent.y) +256 ,int(bounding_box.extent.x) + 256),(int(-bounding_box.extent.y) +256, int(-bounding_box.extent.x) +256),(0,255,0),1)
        # cv2.circle(self.image_lidar, (256, 256), 2, color=(0, 0, 255), thickness=2)

    def render(self):
        # cv2.imshow('LIDAR',self.image_lidar)
        cv2.waitKey(1)

    def on_invasion(self, weak_self, event):
        points = np.frombuffer(event.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))
        self.points = points
 
 
# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        Attachment = carla.AttachmentType
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=10, y = 10, z = 1 )), attach_to=self._parent, attachment_type= Attachment.SpringArm)

        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))

# ==============================================================================
# -- GnssSensor --------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))
    
    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude
        #print("探索中’", self.lat," ",self.lon)

# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        Attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
            # (carla.Transform(carla.Location(x=-30.5, z=8.5), carla.Rotation(pitch=0.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
            (carla.Transform(carla.Location(x=5.5, y=1.5, z=1.5)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-8.0, z=6.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-1, y=-bound_y, z=0.5)), Attachment.Rigid)]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
                'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '5000')
            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][0] != self.sensors[self.index][0]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0),)

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype = int)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
            
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    try:
        # client = carla.Client("192.168.1.7", args.port)
        client = carla.Client("127.0.0.1", args.port)
        client.set_timeout(2.0)
        world = client.load_world('Town03')
        client.set_timeout(4.0)
        world = client.reload_world()
        

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args)
        controller = KeyboardControl(world, args.autopilot)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock):
                return
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()
        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1700x956',
        # default='640x956',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
