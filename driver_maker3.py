#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
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

from configparser import ConfigParser
import glob
import os
import sys
import pickle
import csv

try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
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

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

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
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
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

START_POSITION1 = carla.Transform(carla.Location(x=0.0, y=45.5, z=2.0))
START_POSITION2 = carla.Transform(carla.Location(x=225.0, y=150.0, z=39.0), carla.Rotation(yaw=90))
START_POSITION = carla.Transform(carla.Location(x=180.0, y=199.0, z=40.0))
#320x180
MINI_WINDOW_WIDTH =  320
MINI_WINDOW_HEIGHT = 180
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate-1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, actor_filter,START_POSITIONk):
        self.world = carla_world
        self.map = self.world.get_map()
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        self.startpose = START_POSITIONk
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.vehicle = self.player

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager._index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager._transform_index if self.camera_manager is not None else 0
        #blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, self.startpose)
        while self.player is None:
            spawn_points = self.map.get_spawn_points()
            #spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, self.startpose)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager._transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)
        self.vehicle = self.player
        print(dir(self.player))

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        #self.hud.render(display)

    def destroySensors(self):
            self.camera_manager.sensor.destroy()
            self.camera_manager.sensor = None
            self.camera_manager._index = None

    def destroy(self):
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.player.set_autopilot(self._autopilot_enabled)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock):
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
                    currentIndex = world.camera_manager._index
                    world.destroySensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
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
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' % ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not (pygame.key.get_mods() & KMOD_CTRL):
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            world.player.apply_control(self._control)

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
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
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
    def __init__(self, width, height,FileSavedest):
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
        self.frame_number = 0
        self.simulation_time = 0
        self._show_info = False
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        self.write_count = 0
        self.print_list = []
        self.save_once = 0
        self.dest = FileSavedest
        self.currCamStateLR = 0
        self.currCamStateBACK = 0
        self.write_start = 0

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame_number = timestamp.frame_count
        self.simulation_time = timestamp.elapsed_seconds
        self.write_count+=1

    def tick(self, world, clock):
        #self._notifications.tick(world, clock)
        # if not self._show_info:
        #     return
        # if not self.save_once:
        #     vehicles = world.world.get_actors().filter('vehicle.*')
        #     vided = {}
        #     for idi, x in enumerate(sorted(vehicles)):
        #         vided[idi] = np.asarray([x.id, get_actor_display_name(x, truncate=22)])
        #     zd = zip(*vided.values())
        #     with open('vehicle_ids.csv',"wb") as f:
        #         writer = csv.writer(f, delimiter=',')
        #         print(vided.keys())
        #         writer.writerows(vided.keys())
        #         writer.writerows(zd)
        #     self.save_once = 1
        # else:
        #     pass

        t = world.vehicle.get_transform()
        v = world.vehicle.get_velocity()
        c = world.vehicle.get_control()
        a = world.vehicle.get_acceleration()
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        #collision = [colhist[x + self.frame_number - 200] for x in range(0, 200)]
        #max_col = max(1.0, max(collision))
        #collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

        #if not self._show_info:
        

        vehicles = world.world.get_actors().filter('vehicle.*')
        vehicles = sorted(vehicles) 
        vel_mag = (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
        acc_mag = (math.sqrt(a.x**2 + a.y**2 + a.z**2))
        angle = t.rotation.yaw
        heading = heading
        x_ = t.location.x
        y_ = t.location.y
        z_ = t.location.z
        lati = world.gnss_sensor.lat
        longi = world.gnss_sensor.lon
        throttle = c.throttle
        brake = c.brake
        steer = c.steer
        frame_number = self.frame_number
        impulse = colhist[self.frame_number -1]
        time = datetime.timedelta(seconds=int(self.simulation_time))
        player_id = world.player.id
        distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
        vehicle_list = sorted([x.id for x in vehicles])
        vehicles_distance = [distance(x.get_location()) for x in vehicles if x.id != world.player.id] 
        



        listing = [player_id, frame_number, time, self.simulation_time,x_,y_,z_,vel_mag,v.x,v.y, acc_mag, a.x,a.y,angle, heading, throttle, brake, steer, self.currCamStateLR, self.currCamStateBACK, impulse]
        if len(self.print_list) == 0:
            self.print_list = [listing+vehicle_list+vehicles_distance]
        else:
            self.print_list.append(listing+vehicle_list+vehicles_distance)
            print("appended")


        if self.write_count>400 or self.write_start == 0:
            with open('../saved_data/saved_egocar_'+self.dest+'.csv',"wb") as f:
                writer = csv.writer(f)
                print("inside")
                if self.write_start == 0:
                    write_info = ["EGO_ID", "FRAME_NUM", "TIME_FORMATTED", "SIM_TIM_MILLISEC","X","Y","Z","VEL_MAG","VEL_x","VEL_Y", "ACC_MAG", "ACC_x","ACC_Y", "ANGLE", "HEADING", "THROTTLE", "BRAKE", "STEER", "LR_CAMERA_STATE", "FB_CAMERA_STATE", "IMPULSE"]
                    write_info  += ["V_ID_"+str(i) for i in range(0,len(vehicle_list))]
                    write_info  += ["V_DISTANCE_FROM_"+str(i) for i in range(0,len(vehicle_list))]
                    writer.writerows(write_info)
                    print("saved_start")
                    self.write_start = 100
                else:
                    #writer.writerows([write_info])
                    writer.writerows(self.print_list)
                    self.write_count = 0
                    print("saved")

        if self.write_count>200:
            with open('../saved_data/saved_egocar_'+self.dest+'.csv',"wb") as f:
                writer = csv.writer(f)
                writer.writerows(self.print_list)
                print("saved")
        # self.f.write(''.join(str(e)+'\t' for e in ))
        # self.f.write("\n")
        return


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
            bar_h_offset = 80
            bar_width = 86
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
                if item: # At this point has to be a str.
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
        self._history = []
        self._parent = parent_actor
        self._hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self._history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        #self._hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self._history.append((event.frame_number, intensity))
        if len(self._history) > 4000:
            self._history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self._hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_detector')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        text = ['%r' % str(x).split()[-1] for x in set(event.crossed_lane_markings)]
        self._hud.notification('Crossed line %s' % ' and '.join(text))

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


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.mini_sensor1 = None
        self.mini_sensor2 = None
        self._mini_surface1 = None
        self._mini_surface2 = None
        self._surface = None
        self._parent = parent_actor
        self._hud = hud
        self._recording = False
        self._mini_transform_index1 = 0
        self._mini_transform_index2 = 0
        self.mini_index1 = 2
        self.mini_index2 = 5
        
        self._camera_transforms = [
        #1.6,1.7
            #carla.Transform(carla.Location(x=1.0, z=1.7)),
            carla.Transform(carla.Location(x=0.1, y = -0.4, z=1.1)),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))]
        self._camera_transforms_change = [
        #1.6,1.7
            #carla.Transform(carla.Location(x=1.0, z=1.7)),
            carla.Transform(carla.Location(x=0.2, y = -0.4, z=1.2)),
            carla.Transform(carla.Location(x=0.2, y = -0.4, z=1.2), carla.Rotation(yaw=90)),
            carla.Transform(carla.Location(x=0.2, y = -0.4, z=1.2), carla.Rotation(yaw=-90)),
            carla.Transform(carla.Location(x=-0.3, y = 0.1, z=1.25), carla.Rotation(yaw=150)),
            carla.Transform(carla.Location(x=0.2, y = -0.4, z=1.2),carla.Rotation(yaw=0)),]
        self._transform_index = 1
        self._sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        
        self._mini_view_image_sensor1 = self._sensors[self.mini_index1][0]
        self._mini_view_image_sensor2 = self._sensors[self.mini_index2][0]
##

        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self._sensors:
            bp = bp_library.find(item[0])
            if item[0]== self._mini_view_image_sensor1:
                bp.set_attribute('image_size_x', str(MINI_WINDOW_WIDTH))
                bp.set_attribute('image_size_y', str(MINI_WINDOW_HEIGHT))
            elif item[0] == self._mini_view_image_sensor2:
                bp.set_attribute('image_size_x', str(MINI_WINDOW_WIDTH))
                bp.set_attribute('image_size_y', str(MINI_WINDOW_HEIGHT))
            elif item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            else:
                pass
            # if item[0].startswith('sensor.camera'):
            #     bp.set_attribute('image_size_x', str(hud.dim[0]))
            #     bp.set_attribute('image_size_y', str(hud.dim[1]))
            # elif item[0].startswith('sensor.lidar'):
            #     bp.set_attribute('range', '5000')
            item.append(bp)
        self._index = None

    def toggle_camera(self):
        self._transform_index = (self._transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self._transform_index])

    def set_sensor(self, index, change_cam = 0, notify=True):
        # change_cam: Basically change the parent sensor camera to 0:default-front, 1:left, 2:right, 3:back, 4:front

        index = index % len(self._sensors)
        needs_respawn = True if self._index is None \
            else self._sensors[index][0] != self._sensors[self._index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self._surface = None
                self.mini_sensor1.destroy()
                self._mini_surface1 = None
                self.mini_sensor2.destroy()
                self._mini_surface2 = None

            self.sensor = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index],
                attach_to=self._parent)
            self.mini_sensor1 = self._parent.get_world().spawn_actor(
                self._sensors[self.mini_index1][-1],
                self._camera_transforms[self._mini_transform_index1],
                attach_to=self._parent)
            self.mini_sensor2 = self._parent.get_world().spawn_actor(
                self._sensors[self.mini_index2][-1],
                self._camera_transforms[self._mini_transform_index2],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image,0))
            self.mini_sensor1.listen(lambda image: CameraManager._parse_image(weak_self, image,self.mini_index1))
            self.mini_sensor2.listen(lambda image: CameraManager._parse_image(weak_self, image,self.mini_index2))

        if notify:
            self._hud.notification(self._sensors[index][2])
        self._index = index

    def next_sensor(self):
        self.set_sensor(self._index + 1)

    def toggle_recording(self):
        self._recording = not self._recording
        self._hud.notification('Recording %s' % ('On' if self._recording else 'Off'))

    def render(self, display):
        gap_x = (WINDOW_WIDTH - 2 * MINI_WINDOW_WIDTH) / 3
        mini_image_y = WINDOW_HEIGHT - MINI_WINDOW_HEIGHT - gap_x

        if self._surface is not None:
            display.blit(self._surface, (0, 0))

        # if self._mini_surface1 is not None:
        #     display.blit(self._mini_surface1 , (gap_x, mini_image_y))

        # if self._mini_surface2 is not None:
        #     display.blit(self._mini_surface2 , ((2 * gap_x + MINI_WINDOW_WIDTH), mini_image_y))

    @staticmethod
    def _parse_image(weak_self, image,mini_index):
        self = weak_self()
        if not self:
            return
        if self._sensors[self._index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0]/3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self._hud.dim) / 100.0
            lidar_data += (0.5 * self._hud.dim[0], 0.5 * self._hud.dim[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self._hud.dim[0], self._hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self._surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self._sensors[self._index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            if mini_index == self.mini_index1:
                self._mini_surface1 = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            elif mini_index == self.mini_index2:
                self._mini_surface2 = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            else:
                self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
                #self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out/%08d' % image.frame_number)



# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.player.set_autopilot(self._autopilot_enabled)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # initialize steering wheel
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

        self._parser = ConfigParser()
        self._parser.read('wheel_config.ini')
        self._steer_idx = int(
            self._parser.get('G29 Racing Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('G29 Racing Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('G29 Racing Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('G29 Racing Wheel', 'reverse'))
        self._handbrake_idx = int(
            self._parser.get('G29 Racing Wheel', 'handbrake'))
        self.cam_manager = world.camera_manager
        self.camera = world.camera_manager.sensor
        self.currCamStateLR = 0
        self.currCamStateBACK = 0

    def parse_events(self, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                # if event.button == 0:
                #     world.restart()
                # elif event.button == 1:
                #     world.hud.toggle_info()
                if event.button == 8:
                    world.camera_manager.toggle_camera()
                elif event.button == 6:
                    world.next_weather()
                elif event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.button == 7:
                    world.camera_manager.next_sensor()

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
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' % ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p:
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
 
        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._parse_vehicle_wheel()
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            world.player.apply_control(self._control)

    def toggle_lr(self, hat):
        print(self.currCamStateLR)
        if hat == 0 and self.currCamStateLR == 0:
            self.currCamStateLR = 0
            #self.camera.set_transform(self.cam_manager._camera_transforms_change[1])
            
            return
        if hat == 1 and self.currCamStateLR != 1:
            self.currCamStateLR = 1
            #self.camera.set_sensor(self.index, 1)
            self.camera.set_transform(self.cam_manager._camera_transforms_change[1])
            return
        if hat == -1 and self.currCamStateLR != 2:
            self.currCamStateLR = 2
            #self.camera.set_sensor(self.index, 2)
            self.camera.set_transform(self.cam_manager._camera_transforms_change[2])
            return
        if hat == 0 and self.currCamStateLR != 0:
            self.currCamStateLR = 0
            #self.camera.set_sensor(self.index, 4)
            self.camera.set_transform(self.cam_manager._camera_transforms_change[4])
            
            return
        
    def toggle_back(self,hat):
        print(self.currCamStateBACK)
        if hat == 0 and self.currCamStateBACK != 0:
            self.currCamStateBACK = 0
            #self.camera.set_sensor(self.index, 4)
            self.camera.set_transform(self.cam_manager._camera_transforms_change[4])
            print("back actual")
        if hat == 0 and self.currCamStateLR == 0:
            self.currCamStateBACK = 0
            print("transforming")
            return
        if hat == 1 and self.currCamStateBACK != 1:
            self.currCamStateBACK = 1
            #self.camera.set_sensor(self.index, 4)
            self.camera.set_transform(self.cam_manager._camera_transforms_change[4])
            return
        if hat == -1 and self.currCamStateBACK != 2:
            self.currCamStateBACK = 2
            #self.camera.set_sensor(self.index, 3)
            self.camera.set_transform(self.cam_manager._camera_transforms_change[3])
            return

    # camera list: 0:default-front, 1:left, 2:right, 3:back, 4:front
    def toggle_lr_button(self, X,B):
        print(self.currCamStateLR)
        if X == 0 and B == 0 and self.currCamStateLR == 0:
            self.currCamStateLR = 0
            #self.camera.set_transform(self.cam_manager._camera_transforms_change[1])
            return
        if X == 1 and B == 0 and self.currCamStateLR != 1:
            self.currCamStateLR = 1
            #self.camera.set_sensor(self.index, 1)
            self.camera.set_transform(self.cam_manager._camera_transforms_change[1])
            return
        if X == 0 and B == 1 and self.currCamStateLR != 2:
            self.currCamStateLR = 2
            #self.camera.set_sensor(self.index, 2)
            self.camera.set_transform(self.cam_manager._camera_transforms_change[2])
            return
        if X == 0 and B == 0 and self.currCamStateLR != 0:
            self.currCamStateLR = 0
            #self.camera.set_sensor(self.index, 4)
            self.camera.set_transform(self.cam_manager._camera_transforms_change[4])
            
            return
        
    def toggle_back_button(self,Y,A):
        print(self.currCamStateBACK)
        if Y == 0 and A == 0 and self.currCamStateBACK != 0:
            self.currCamStateBACK = 0
            #self.camera.set_sensor(self.index, 4)
            self.camera.set_transform(self.cam_manager._camera_transforms_change[4])
        if Y == 0 and A == 0 and self.currCamStateLR == 0:
            self.currCamStateBACK = 0
            return
        if Y == 1 and A == 0 and self.currCamStateBACK != 1:
            self.currCamStateBACK = 1
            #self.camera.set_sensor(self.index, 4)
            self.camera.set_transform(self.cam_manager._camera_transforms_change[4])
            return
        if Y == 0 and A == 1 and self.currCamStateBACK != 2:
            self.currCamStateBACK = 2
            #self.camera.set_sensor(self.index, 3)
            self.camera.set_transform(self.cam_manager._camera_transforms_change[3])
            return

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
        
    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        # hats = self._joystick.get_numhats()
        # hat = self._joystick.get_hat( hats-1 )

        buttonA = self._joystick.get_button(0)
        buttonB = self._joystick.get_button(1)
        buttonX = self._joystick.get_button(2)
        buttonY = self._joystick.get_button(3)

        self.toggle_lr_button(buttonB,buttonX)
        self.toggle_back_button(buttonY,buttonA)
        
        # self.toggle_lr(hat[0])
        # self.toggle_back(hat[1])


        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd

        #toggle = jsButtons[self._reverse_idx]


        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])


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
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


class Data_extract(object):
    
    def __init__(self, hud, world, startpos):
        self.x = 0
        self.y = 0
        self.z = 0
        self.world = world
        self.car = world.vehicle
        self.hud = hud
        self.locx_array = np.array([])
        self.locy_array = np.array([])
        self.vel_array = np.array([])
        self.acc_array = np.array([])
        self.time_stamp_array = np.array([])
        self.server_clock = pygame.time.Clock()
        self.server_fps = 0
        self.frame_number = 0
        self.simulation_time = 0
        self.f = open('saved'+str(startpos)+'.txt', 'w', buffering=0)

    def extract(self):
        self.simulation_time = self.hud.simulation_time
        self.vel_mag = math.sqrt(self.car.get_velocity().x**2 + self.car.get_velocity().y**2 + self.car.get_velocity().z**2)
        self.x = self.car.get_location().x
        self.y = self.car.get_location().y
        self.z = self.car.get_location().z
        self.f.write(''.join(str(e)+'\t' for e in [self.simulation_time,self.x,self.y, self.z,self.vel_mag]))
        self.f.write("\n")




# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    if args.startpos == 1:
        startposition = START_POSITION1
    if args.startpos == 2:
        startposition = START_POSITION2

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height, args.dest)
        world = World(client.get_world(), hud, args.filter,startposition)
        #controller = KeyboardControl(world, args.autopilot)
        
        if args.keyjs == 'key':
            controller = KeyboardControl(world, args.autopilot)
        else:
            controller = DualControl(world, args.autopilot)

        world2 = client.get_world()
        spectator = world2.get_spectator()

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)

            if args.keyjs == 'key':
                if controller.parse_events(client, world, clock):
                    return
            else:
                if controller.parse_events(world, clock):
                    return
            # if args.keyjs == 'key':
            #     hud.currCamStateLR = controller.currCamStateLR
            #     hud.currCamStateBACK = controller.currCamStateBACK
            
            world.tick(clock)
            world.render(display)

            transform = world.player.get_transform()
            spectator_transform = carla.Transform(carla.Location(transform.location.x+10,transform.location.y,transform.location.z+100))
            spectator_transform.rotation.pitch = -90.0
            #spectator_transform.rotation.yaw = transform.rotation.yaw 
            spectator.set_transform(spectator_transform)

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
        #default='1280x720',
        default='3440x1300',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '-c','--keyjs',
        metavar='KEYJS',
        default='key',
        help='Keyboard or Joystick')
    argparser.add_argument(
        '-d','--dest',
        metavar='DEST',
        default='0001',
        help='00 -> player number, 01 -> trial number')
    argparser.add_argument(
        '-s','--startpos',
        metavar='STRTPOS',
        default='1',
        type=int,
        #default='853x480',
        #1280x720
        help='Start position of vehicle {1,2}')
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
