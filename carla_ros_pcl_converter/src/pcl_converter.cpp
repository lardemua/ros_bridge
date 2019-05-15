//
// Created by pedro on 15-05-2019.
//

/*
 * Copyright (c) 2019 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */

/* System Includes */
#include <ros/ros.h>
#include "PclConverter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_converter");
    PclConverter pclConverter;
    ros::spin();
};
