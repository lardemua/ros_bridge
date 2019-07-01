#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Class used to spawn NPCs into the simulation
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


# ------------------------
#   Spawn NPC Class
# ------------------------
class Spawn_NPC:
    """
    Class used to spawn NPCs into the simulation
    """
    def __init__(self):
        rospy.init_node('spawn_npc')
        self.host = rospy.get_param('/carla/host', '127.0.0.1')
        self.port = rospy.get_param('/carla/port', '2000')
        self.number_of_vehicles = rospy.get_param('/carla/npcs', '10')
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        self.world = None
        self.client = None
        self.actor_list = []

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
        self.spawn_npcs()
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    """
    Spawn NPC Function
    """
    def spawn_npcs(self):
        # blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        spawn_points = self.world.get_map().get_spawn_points()
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
            blueprint.set_attribute('role_name', "{}".format(self.role_name))
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))

        for response in self.client.apply_batch_sync(batch):
            if response.error:
                logging.error(response.error)
            else:
                self.actor_list.append(response.actor_id)

        print('spawned %d vehicles, press Ctrl+C to exit.' % len(self.actor_list))

        # while True:
        #     self.world.wait_for_tick()

    """
    Destroy NPC Function
    """
    def destroy_npcs(self):
        print('\nDestroying %d actors' % len(self.actor_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])


# ==============================================================================
# -- Main Function ---------------------------------------------------------
# ==============================================================================

def main():
    spawn_npc = Spawn_NPC()
    try:
        spawn_npc.run()
    finally:
        if spawn_npc is not None:
            spawn_npc.destroy_npcs()


if __name__ == '__main__':
    main()


