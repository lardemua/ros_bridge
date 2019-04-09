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
# -- NPC CONTROLLER ------------------------------------------------------------
# ==============================================================================
class NPC_Controller:
    """
    Class responsible for the behaviour of the NPCs in the carla WORLD
    """
    def __init__(self):
        """
        Constructor for NPC_Controller.
        """
        self.callback()

    def callback(self):
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

        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

        actor_list = []
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        try:
            world = client.get_world()
            blueprints = world.get_blueprint_library().filter('vehicle.*')

            if args.safe:
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
                blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
                blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]

            def try_spawn_random_vehicle_at(transform):
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

            # @todo Needs to be converted to list to be shuffled.
            spawn_points = list(world.get_map().get_spawn_points())
            random.shuffle(spawn_points)

            print('found %d spawn points.' % len(spawn_points))

            count = args.number_of_vehicles

            for spawn_point in spawn_points:
                if try_spawn_random_vehicle_at(spawn_point):
                    count -= 1
                if count <= 0:
                    break

            while count > 0:
                time.sleep(args.delay)
                if try_spawn_random_vehicle_at(random.choice(spawn_points)):
                    count -= 1

            print('spawned %d vehicles, press Ctrl+C to exit.' % args.number_of_vehicles)

            while True:
                time.sleep(10)


            # spawn_points = world.get_map().get_spawn_points()
            # number_of_spawn_points = len(spawn_points)
            # if args.number_of_vehicles < number_of_spawn_points:
            #     random.shuffle(spawn_points)
            # elif args.number_of_vehicles > number_of_spawn_points:
            #     msg = 'requested %d vehicles, but could only find %d spawn points'
            #     logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            #     args.number_of_vehicles = number_of_spawn_points
            # # @todo cannot import these directly.
            # SpawnActor = carla.command.SpawnActor
            # SetAutopilot = carla.command.SetAutopilot
            # FutureActor = carla.command.FutureActor
            #
            # batch = []
            # for n, transform in enumerate(spawn_points):
            #     if n >= args.number_of_vehicles:
            #         break
            #     blueprint = random.choice(blueprints)
            #     if blueprint.has_attribute('color'):
            #         color = random.choice(blueprint.get_attribute('color').recommended_values)
            #         blueprint.set_attribute('color', color)
            #     blueprint.set_attribute('role_name', 'autopilot')
            #     batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))
            #
            # for response in client.apply_batch_sync(batch):
            #     if response.error:
            #         logging.error(response.error)
            #     else:
            #         actor_list.append(response.actor_id)
            #
            # print('spawned %d vehicles, press Ctrl+C to exit.' % len(actor_list))
            #
            # while True:
            #     world.wait_for_tick()
        finally:
            print('\ndestroying %d actors' % len(actor_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])

# ==============================================================================
# -- MAIN FUNCTION -------------------------------------------------------------
# ==============================================================================


def main(args):
    npc_controller = NPC_Controller()
    rospy.init_node('npc_controller', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")


if __name__ == '__main__':
    main(sys.argv)