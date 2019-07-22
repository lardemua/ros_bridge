#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Class used to spawn pedestrians into the simulation
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


# -------------------------
#   Spawn Pedestrians Class
# -------------------------
class Spawn_Pedestrians:
    """
    Class used to spawn pedestrians into the simulation
    """
    def __init__(self):
        rospy.init_node('spawn_npc')
        self.host = rospy.get_param('/carla/host', '127.0.0.1')
        self.port = rospy.get_param('/carla/port', '2000')
        self.number_of_pedestrians = rospy.get_param('/carla/pedestrians', '10')
        self.role_name = rospy.get_param('~role_name', 'ego_pedestrians')
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
        vehicle_spawn_points = spawn_points[:NUM_OF_VEHICLE_SPAWN_POINTS]
        walker_spawn_points = spawn_points[-NUM_OF_WALKER_SPAWN_POINTS:]
        number_of_spawn_points = len(walker_spawn_points)

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
            if blueprint.has_attribute('is_invincible'):
                blueprint.set_attribute('is_invincible', 'false')

            player = self.world.try_spawn_actor(blueprint, transform)
            player_control = carla.WalkerControl()
            player_control.speed = 3
            player_control.jump = False
            pedestrian_heading = 90
            player_rotation = carla.Rotation(0, pedestrian_heading, 0)
            direction = player_rotation.get_forward_vector()
            player_control.direction.x = direction.x
            player_control.direction.y = -direction.y
            player_control.direction.z = direction.z
            player.apply_control(player_control)
            self.actor_list.append(player)

        print('spawned %d pedestrians, press Ctrl+C to exit.' % len(self.actor_list))

    """
    Destroy NPC Function
    """
    def destroy_npcs(self):
        print('\nDestroying %d actors' % len(self.actor_list))
        for actor in self.actor_list:
            # print('\nDestroying actor ' + actor.get_global_ID())
            actor.destroy()


# ==============================================================================
# -- Main Function ---------------------------------------------------------
# ==============================================================================

def main():
    spawn_pedestrians = Spawn_Pedestrians()
    try:
        spawn_pedestrians.run()
    finally:
        if spawn_pedestrians is not None:
            spawn_pedestrians.destroy_npcs()


if __name__ == '__main__':
    main()


