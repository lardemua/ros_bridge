#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Class used to spawn NPCs and Pedestrians into the simulation
"""

# ------------------------
#   IMPORTS
# ------------------------
from __future__ import print_function
import rospy
import carla
import logging
import random
import roslib
roslib.load_manifest('carla_ros_spawn_npc')


# -------------------------
#   Global Variables
# -------------------------
NUM_OF_VEHICLE_SPAWN_POINTS = 83
NUM_OF_WALKER_SPAWN_POINTS = 42

# ---------------------------------
#   Spawn NPC and Pedestrians Class
# ---------------------------------
class Spawn_NPCS:
    """
    Class used to spawn NPCs and pedestrians into the simulation
    """
    def __init__(self):
        rospy.init_node('spawn_npc')
        self.host = rospy.get_param('/carla/host', '127.0.0.1')
        self.port = rospy.get_param('/carla/port', '2000')
        self.number_of_vehicles = rospy.get_param('/carla/vehicles', '10')
        self.number_of_pedestrians = rospy.get_param('/carla/pedestrians', '10')
        self.vehicle_role_name = rospy.get_param('~vehicle_role_name', 'ego_vehicle')
        self.pedestrian_role_name = rospy.get_param('~pedestrian_role_name', 'ego_pedestrian')
        self.world = None
        self.client = None
        self.vehicle_actor_list = []
        self.pedestrian_actor_list = []

    """
    Run Function
    """
    def run(self):
        """
        Main Loop Function
        :return:
        """
        client = carla.Client(self.host, self.port)
        client.set_timeout(2.0)
        self.client = client
        self.world = client.get_world()
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
        spawn_points = self.world.get_map().get_spawn_points()
        vehicle_spawn_points = spawn_points[:NUM_OF_VEHICLE_SPAWN_POINTS]
        walker_spawn_points = spawn_points[-NUM_OF_WALKER_SPAWN_POINTS:]
        self.spawn_npcs(vehicle_spawn_points)
        self.spawn_pedestrians(walker_spawn_points)
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    """
    Spawn NPC Function
    """
    def spawn_npcs(self, spawn_points):
        number_of_spawn_points = len(spawn_points)

        if self.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif self.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, self.number_of_vehicles, number_of_spawn_points)
            self.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these actors directly
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= self.number_of_vehicles:
                break
            blueprint = random.choice(self.world.get_blueprint_library().filter('vehicle.*'))
            blueprint.set_attribute('role_name', "{}".format(self.vehicle_role_name))
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))

        for response in self.client.apply_batch_sync(batch):
            if response.error:
                logging.error(response.error)
            else:
                self.vehicle_actor_list.append(response.actor_id)

        print('spawned %d vehicles, press Ctrl+C to exit.' % len(self.vehicle_actor_list))

        # while True:
        #     self.world.wait_for_tick()

    """
    Spawn NPC Function
    """
    def spawn_pedestrians(self, spawn_points):
        number_of_spawn_points = len(spawn_points)

        if self.number_of_pedestrians < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif self.number_of_pedestrians > number_of_spawn_points:
            msg = 'requested %d pedestrians, but could only find %d spawn points for pedestrians'
            logging.warning(msg, self.number_of_pedestrians, number_of_spawn_points)
            self.number_of_pedestrians = number_of_spawn_points

        for n, transform in enumerate(spawn_points):
            if n >= self.number_of_pedestrians:
                break
            blueprint = random.choice(self.world.get_blueprint_library().filter('walker.*'))
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            player = self.world.try_spawn_actor(blueprint, transform)
            player_control = carla.WalkerControl()
            player_control.speed = 3
            pedestrian_heading = 90
            player_rotation = carla.Rotation(0, pedestrian_heading, 0)
            player_control.direction = player_rotation.get_forward_vector()
            player.apply_control(player_control)
            self.pedestrian_actor_list.append(player)

        print('spawned %d pedestrians, press Ctrl+C to exit.' % len(self.pedestrian_actor_list))

    """
    Destroy NPC Function
    """
    def destroy_npcs(self):
        print('\nDestroying %d vehicles' % len(self.vehicle_actor_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicle_actor_list])

    """
    Destroy Pedestrians Function
    """
    def destroy_pedestrians(self):
        print('\nDestroying %d pedestrians' % len(self.pedestrian_actor_list))
        for actor in self.pedestrian_actor_list:
            # print('\nDestroying actor ' + actor.get_global_ID())
            actor.destroy()


# ==============================================================================
# -- Main Function ---------------------------------------------------------
# ==============================================================================

def main():
    spawn_npcs = Spawn_NPCS()
    try:
        spawn_npcs.run()
    finally:
        if spawn_npcs is not None:
            spawn_npcs.destroy_npcs()
            spawn_npcs.destroy_pedestrians()


if __name__ == '__main__':
    main()


