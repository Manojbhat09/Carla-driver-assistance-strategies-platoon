#!/usr/bin/env python

"""
Welcome to Autopilot Multi-agent control file for CARLA

    TAB          : change camera position
    `            : next camera sensor
    [1-9]        : change to camera sensor [1-9]
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    R            : toggle recording images to disk

    H/?          : toggle help
    ESC          : quit
"""

#******************* Imports ********************#
from __future__ import print_function
import glob
import os
import sys
import math
import time
try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import ColorConverter as cc
from configparser import SafeConfigParser
import argparse
import logging
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
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')



#************************************************#
#******************* Initilize position 1 ********************#

START_POSITION1 = carla.Transform(carla.Location(x=180.0, y=199.0, z=40.0))
START_POSITION2 = carla.Transform(carla.Location(x=225.0, y=151.0, z=40.0), carla.Rotation(yaw=90))
START_POSITION = carla.Transform(carla.Location(x=180.0, y=199.0, z=40.0))
#320x180
# MINI_WINDOW_WIDTH = 320
# MINI_WINDOW_HEIGHT = 180
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
#************************************************#
#******************* Weather Presets ********************#

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


#************************************************#
#******************* Classes ********************#

#******************* World ********************#
class World(object):
    def __init__(self, carla_world, hud,START_POSITIONk):
        self.world = carla_world
        self.hud = hud
        
        blueprint = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        #blueprint = self._get_random_blueprint()
        spawn_points = self.map.get_spawn_points()
        spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
        #self.vehicle = self.world.spawn_actor(blueprint, START_POSITIONk)
        self.vehicle = self.world.spawn_actor(blueprint, spawn_point)
        self.collision_sensor = CollisionSensor(self.vehicle, self.hud)
        self.camera_manager = CameraManager(self.vehicle, self.hud)
        self.camera_manager.set_sensor(0,notify=False)
#        self.camera_manager.set_sensor(2, notify=False)
#Not controlling mini sensors from here right now
        self.controller = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0

    def restart(self):
        cam_index = self.camera_manager._index
        cam_pos_index = self.camera_manager._transform_index
        start_pose = self.vehicle.get_transform()
        start_pose.location.z += 2.0
        start_pose.rotation.roll = 0.0
        start_pose.rotation.pitch = 0.0
        blueprint = self._get_random_blueprint()
        self.destroy()
        self.vehicle = self.world.spawn_actor(blueprint, start_pose)
        self.collision_sensor = CollisionSensor(self.vehicle, self.hud)
        self.camera_manager = CameraManager(self.vehicle, self.hud)
        self.camera_manager._transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = ' '.join(self.vehicle.type_id.replace('_', '.').title().split('.')[1:])
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.vehicle.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy(self):
        for actor in [self.camera_manager.sensor, self.collision_sensor.sensor, self.vehicle]:
            if actor is not None:
                actor.destroy()

    def _get_random_blueprint(self):
        bp = random.choice(self.world.get_blueprint_library().filter('vehicle'))
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)
        return bp

#******************* Camera Manager ********************#
class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        # self.mini_sensor1 = None
        # self.mini_sensor2 = None
        self._surface = None
        # self._mini_surface1 = None
        # self._mini_surface2 = None
        self._parent = parent_actor
        self._hud = hud
        self._recording = False
        self._camera_transforms = [
        #1.6,1.7
            carla.Transform(carla.Location(x=1.6, z=1.7)),
            carla.Transform(carla.Location(x=-0, z=1.1)),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))]
        self._transform_index = 1
        # self._mini_transform_index1 = 0
        # self._mini_transform_index2 = 0
        self._sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)']]

##		
        # self.mini_index1 = 2
        # self.mini_index2 = 5
        # self._mini_view_image_sensor1 = self._sensors[self.mini_index1][0]
        # self._mini_view_image_sensor2 = self._sensors[self.mini_index2][0]
##

        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self._sensors:
            bp = bp_library.find(item[0])
            # if item[0]== self._mini_view_image_sensor1:
            # 	bp.set_attribute('image_size_x', str(MINI_WINDOW_WIDTH))
            # 	bp.set_attribute('image_size_y', str(MINI_WINDOW_HEIGHT))
            # elif item[0] == self._mini_view_image_sensor2:
            # 	bp.set_attribute('image_size_x', str(MINI_WINDOW_WIDTH))
            # 	bp.set_attribute('image_size_y', str(MINI_WINDOW_HEIGHT))
            # else:
            bp.set_attribute('image_size_x', str(hud.dim[0]))
            bp.set_attribute('image_size_y', str(hud.dim[1]))
            item.append(bp)
        self._index = None
        self._server_clock = pygame.time.Clock()

    def toggle_camera(self):
        self._transform_index = (self._transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self._transform_index])

    def set_sensor(self, index ,notify=True):
        index = index % len(self._sensors)
        needs_respawn = True if self._index is None \
            else self._sensors[index][0] != self._sensors[self._index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self._surface = None
                # self.mini_sensor1.destroy()
                # self._mini_surface1 = None
                # self.mini_sensor2.destroy()
                # self._mini_surface2 = None

            self.sensor = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index],
                attach_to=self._parent)
            # self.mini_sensor1 = self._parent.get_world().spawn_actor(
            #     self._sensors[self.mini_index1][-1],
            #     self._camera_transforms[self._mini_transform_index1],
            #     attach_to=self._parent)
            # self.mini_sensor2 = self._parent.get_world().spawn_actor(
            #     self._sensors[self.mini_index2][-1],
            #     self._camera_transforms[self._mini_transform_index2],
            #     attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image,0))
            # self.mini_sensor1.listen(lambda image: CameraManager._parse_image(weak_self, image,self.mini_index1))
            # self.mini_sensor2.listen(lambda image: CameraManager._parse_image(weak_self, image,self.mini_index2))
        if notify:
            self._hud.notification(self._sensors[index][2])
        self._index = index

    def next_sensor(self):
        self.set_sensor(self._index + 1)

    def toggle_recording(self):
        self._recording = not self._recording
        self._hud.notification('Recording %s' % ('On' if self._recording else 'Off'))

    def render(self, display):
        if self._surface is not None:
            display.blit(self._surface, (0,0))

###
        # gap_x = (WINDOW_WIDTH - 2 * MINI_WINDOW_WIDTH) / 3
        # mini_image_y = WINDOW_HEIGHT - MINI_WINDOW_HEIGHT - gap_x

   #      if self._mini_surface1 is not None:
			# display.blit(self._mini_surface1 , (gap_x, mini_image_y))

   #      if self._mini_surface2 is not None:
   #          display.blit(self._mini_surface2 , ((2 * gap_x + MINI_WINDOW_WIDTH), mini_image_y))




    @staticmethod
    def _parse_image(weak_self, image,mini_index):
        self = weak_self()
        if not self:
            return
        self._server_clock.tick()
        self._hud.server_fps = self._server_clock.get_fps()

        # if mini_index:
        # 	image.convert(self._sensors[mini_index][1])
        # 	image.save_to_disk('_out' + str(mini_index) +'/%08d' % image.frame_number)
        # else:
        image.convert(self._sensors[self._index][1])
        image.save_to_disk('_out' + str(self._index) +'/%08d' % image.frame_number)

        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        # if mini_index == self.mini_index1:
        # 	self._mini_surface1 = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        # elif mini_index == self.mini_index2:
        # 	self._mini_surface2 = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        # else:
        self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        # if self._recording:
        #     image.save_to_disk('_out' + str(mini_index) +'/%08d' % image.frame_number)


#******************* Manual Control ********************#

class ManualControl(object):
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self._steer_cache = 0.0
        world.vehicle.set_autopilot(self._autopilot_enabled)
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)
        pygame.init()
        pygame.joystick.init()
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self.control = carla.VehicleControl()
        self.parser = SafeConfigParser()
        self.parser.read('wheel_config.ini')
        self.steer_idx = int(self.parser.get('G29 Racing Wheel', 'steering_wheel'))
        self.throttle_idx = int(self.parser.get('G29 Racing Wheel', 'throttle'))
        self.brake_idx = int(self.parser.get('G29 Racing Wheel', 'brake'))
        self.clutch_idx = int(self.parser.get('G29 Racing Wheel', 'clutch'))
        self.reverse_idx = int(self.parser.get('G29 Racing Wheel', 'buttonLSB'))
        self.handbrake_idx = int(self.parser.get('G29 Racing Wheel', 'buttonRSB'))
        self.Lgear_idx = int(self.parser.get('G29 Racing Wheel', 'buttonLG'))
        self.Rgear_idx = int(self.parser.get('G29 Racing Wheel', 'buttonRG'))
       	self.reverse_gear = 0
       	self.tim = 0

    def reset_joystick(self):
        pygame.event.clear()
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        jsInit = self.js.get_init()
        jsId = self.js.get_id()
        axis = self.js.get_axis(1)
        print('Joystick ID: %d Init status: %s Axis(1): %d' % (jsId, jsInit, axis))



    def parse_events(self, world, clock):
        pygame.event.pump()
        numAxes = self.js.get_numaxes()
        numButtons = self.js.get_numbuttons()
        jsInputs = [ float(self.js.get_axis(i)) for i in range(numAxes)]
        jsButtons = [ float(self.js.get_button(i)) for i in range(numButtons)]
        
        # Value Mapping 
        Throttlecmd = ((1-jsInputs[self.throttle_idx])/2)
        Brakecmd = ((1-jsInputs[self.brake_idx]))
        Clutchcmd = (1-jsInputs[self.clutch_idx])/2

        if Clutchcmd == 0.5:
        	self.tim = 0

        # Clutch Activation
        if  not self.tim:
        	if Brakecmd == 1.0 or Brakecmd == 0.0:
        		if Throttlecmd > 0.0:
        			Throttlecmd = 0.0

        # Initialization fix
        if not self.tim: 
        	#print("Start")
        	if Brakecmd == 1.0:
	        	Brakecmd = 0
	        	self.tim = 1
	        if Clutchcmd == 0.5:
	        	#print('fixed')
	        	Clutchcmd = 0
	        	self.tim = 1
			if Throttlecmd == 0.5:
				Throttlecmd = 0
	        	self.tim = 1
	        

	    # Value adjustments
        if Clutchcmd <0.3:
        	Clutchcmd = 0
        if Brakecmd <0.1:
        	Brakecmd = 0
        if Throttlecmd <0.05:
            Throttlecmd = 0		
        
        
        
        #print("SteerCmd", jsInputs[self.steer_idx], "BrakeCmd", Brakecmd, "ThrottleCmd", Throttlecmd, "clutch", Clutchcmd)
        
        self.control.brake =  Brakecmd
        self.control.throttle = Throttlecmd
        self.control.clutch = Clutchcmd
        self.control.steer = jsInputs[self.steer_idx]
        #print("{0},  {1},   {2}".format(Throttlecmd,Brakecmd,Clutchcmd))

        if jsButtons[self.reverse_idx]==1 and Clutchcmd >0.3:
        	self.reverse_gear += 1
        	print('deploy')

        if self.reverse_gear > 1:
        	self.reverse_gear = 0
        	print('return')

        if self.reverse_gear:
        	self.control.reverse = True
        else :
        	self.control.reverse = False
        vel = math.sqrt(math.pow(world.vehicle.get_velocity().x,2) + math.pow(world.vehicle.get_velocity().y,2))
        if(vel < 0.05):
        	vel = 0
        t = clock.tick(20)
        acc = vel/t*100

        print("Velocity is :{0}, Acceleration is:{1}".format(vel,acc))

        if not self._autopilot_enabled:
            world.vehicle.apply_control(self.control)


    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)



#******************* Display ********************#
class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        mono = next(x for x in pygame.font.get_fonts() if 'mono' in x) # hope for the best...
        mono = pygame.font.match_font(mono, bold=True)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.client_fps = 15
        self.server_fps = 15

    def tick(self, world, clock):
        self.client_fps = clock.get_fps()
        self._notifications.tick(world, clock)

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        self._notifications.render(display)
        self.help.render(display)
        fps_text = 'client: %02d FPS; server: %02d FPS' % (self.client_fps, self.server_fps)
        fps = self._font_mono.render(fps_text, True, (60, 60, 60))
        display.blit(fps, (6, 4))


#******************* Fading Text ********************#
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


#******************* Help Text ********************#
class HelpText(object):
    def __init__(self, font, width, height):
    	print(str(width)+" "+str(height)+" "+str(__doc__))
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

#******************* Collision Sensor ********************#
class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self._hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = ' '.join(event.other_actor.type_id.replace('_', '.').title().split('.')[1:])
        self._hud.notification('Collision with %r' % actor_type)


#******************* Game loop ********************#
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

        display = pygame.display.set_mode((args.width, args.height),pygame.HWSURFACE | pygame.DOUBLEBUF)
 #       display2 = pygame.display.set_mode((args.width, args.height),pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
 #       hud2 = HUD(args.width, args.height)
        world = World(client.get_world(), hud,startposition)
 #       world2 = World(client.get_world(), hud2, START_POSITION1)
        #controller = KeyboardControl(world, args.autopilot)
 #       controller2 = KeyboardControl(world2, args.autopilot)
        clock = pygame.time.Clock()

        driver = ManualControl(world, args.autopilot)
        
        # if control is None:
        #     self._on_new_episode()

        while True:
            clock.tick_busy_loop(20)
            # if controller.parse_events(world, clock):
            #     return
                
            if driver.parse_events(world,clock):
                return
 #           if controller2.parse_events(world2, clock):
 #               return
            world.tick(clock)
            world.render(display)
 #           world2.tick(clock)
 #           world2.render(display2)
            pygame.display.flip()

    finally:

        if world is not None:
            world.destroy()

        pygame.quit()


#************************************************#
#******************* Main Content ********************#



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
        default='1280x720',
        #default='853x480',
        #1280x720
        help='window resolution (default: 853x480)')
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
    except Exception as error:
        logging.exception(error)


if __name__ == '__main__':

    main()