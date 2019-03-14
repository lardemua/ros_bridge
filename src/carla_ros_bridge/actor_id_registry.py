#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""
Actor registry class for carla-id mapping
"""


class ActorIDRegistry(object):

    """
    Registry class to map carla-ids (potentially 64 bit)
    to increasing numbers (usually not exceeding 32 bits)
    """

    def __init__(self):
        """
        Constructor of ActorIDRegistry
        """
        self.id_lookup_table = {}

    def get_ID(self, actor_ID):
        """
        Return a unique counting ID for the given actor_ID
        :param actor_ID: the ID of carla.Actor object
        :type actor_ID: int64
        :return: mapped ID of the actor (unique increasing counter value)
        :rtype: uint32
        """
        if actor_ID not in self.id_lookup_table:
            self.id_lookup_table[actor_ID] = len(self.id_lookup_table) + 1
        return self.id_lookup_table[actor_ID]

    def get_ID_ToString(self, actor_ID):
        """
        Return a string of the unique counting ID for the given actor_ID
        :param actor_ID: the ID of carla.Actor object
        :type actor_ID: int64
        :return: string with leading zeros of mapped id of the actor (unique increasing counter values)
        :rtype: string
        """
        mapped_ID = self.get_ID(actor_ID)
        return "{:03d}".format(mapped_ID)
