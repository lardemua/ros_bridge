//
// Created by pedro on 15-04-2019.
//

#ifndef ROS_BRIDGE_SIMPLE_PLANE_H
#define ROS_BRIDGE_SIMPLE_PLANE_H
#include <carla_ros_kfilter/ekfilter.hpp>


class cPlaneEKF_sp : public Kalman::EKFilter<double,1>
{
    public:
        cPlaneEKF_sp();

    protected:
        void makeA();
        void makeH();
        void makeV();
        void makeR();
        void makeW();
        void makeQ();
        void makeProcess();
        void makeMeasure();

        double Period, Mass, Bfriction, Portance, Gravity;
};

typedef cPlaneEKF_sp::Vector Vector;
typedef cPlaneEKF_sp::Matrix Matrix;
#endif //ROS_BRIDGE_SIMPLE_PLANE_H
