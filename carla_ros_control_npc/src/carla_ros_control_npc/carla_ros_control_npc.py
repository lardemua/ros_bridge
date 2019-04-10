#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Class used to spawn NPCs into the simulation
"""

# ==============================================================================
# -- IMPORTS -------------------------------------------------------------------
# ==============================================================================
import glob
import os
import sys
import time
import rospy
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import argparse
import logging
import random
import roslib
roslib.load_manifest('carla_ros_control_npc')


# ==============================================================================
# -- GLOBAL FUNCTIONS ----------------------------------------------------------
# ==============================================================================
def try_spawn_random_vehicle_at(world, actor_list, blueprints, transform):
    blueprint = random.choice(blueprints)
    if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)
    blueprint.set_attribute('role_name', 'autopilot')
    vehicle = world.try_spawn_actor(blueprint, transform)
    if vehicle is not None:
        actor_list.append(vehicle)
        vehicle.set_autopilot()
        print('spawned %r at %s' % (vehicle.type_id, transform.location))
        return True
    return False

# ==============================================================================
# -- MAIN FUNCTION -------------------------------------------------------------
# ==============================================================================


def main():
    """
    Main Function
    :param args:
    :return:
    """
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', metavar='H', default='127.0.0.1',
                           help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('-n', '--number-of-vehicles', metavar='N', default=10, type=int,
                           help='number of vehicles (default: 10)')
    argparser.add_argument('-d', '--delay', metavar='D', default=2.0, type=float,
                           help='delay in seconds between spawns (default: 2.0)')
    argparser.add_argument('--safe', action='store_true', help='avoid spawning vehicles prone to accidents')
    args = argparser.parse_args()

    actor_list = []
    # setup ROS node
    rospy.init_node('carla_ros_control_npc')
    world = None
    client = None
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        world = client.get_world()
        blueprints = world.get_blueprint_library().filter('vehicle.*')
        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        #  @todo needs to be converted to a list and be shuffled
        spawn_points = list(world.get_map().get_spawn_points())
        random.shuffle(spawn_points)
        print('Found %d spawn points.' % len(spawn_points))
        count = args.number_of_vehicles

        while not rospy.core.is_shutdown():
            for spawn_point in spawn_points:
                if try_spawn_random_vehicle_at(world, actor_list, blueprints, spawn_point):
                    count -= 1
                if count <= 0:
                    break
            while count > 0:
                time.sleep(args.delay)
                if try_spawn_random_vehicle_at(world, actor_list, blueprints, random.choice(spawn_points)):
                    count -= 1
            time.sleep(10)
            print('Spawned %d vehicles, press Ctrl+C to exit.' % args.number_of_vehicles)
    finally:
        if world is not None:
            print('\nDestroying %d actors' % len(actor_list))
            client.apply_batch([carla.command.DestroyActor(x.id) for x in actor_list])


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone!')