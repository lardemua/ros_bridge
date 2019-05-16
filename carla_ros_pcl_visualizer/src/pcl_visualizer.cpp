//
// Created by pedro on 16-05-2019.
//

/*
 * Copyright (c) 2019 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */

/* System Includes */
#include <ros/ros.h>
#include "PclVisualizer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_visualizer");
    PclVisualizer pclVisualizer;
    ros::spin();
};
