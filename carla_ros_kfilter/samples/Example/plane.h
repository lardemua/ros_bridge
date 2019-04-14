//
// Created by pedro on 15-04-2019.
//

#ifndef ROS_BRIDGE_PLANE_H
#define ROS_BRIDGE_PLANE_H

#include <carla_ros_kfilter/ekfilter.hpp>

class cPlaneEKF : public Kalman::EKFilter<double,1,false,true,false> {
    public:
        cPlaneEKF();

    protected:
        void makeBaseA();
        void makeBaseH();
        void makeBaseV();
        void makeBaseR();
        void makeBaseW();
        void makeBaseQ();

        void makeA();
        void makeH();
        void makeProcess();
        void makeMeasure();

        double Period, Mass, Bfriction, Portance, Gravity;
};

typedef cPlaneEKF::Vector Vector;
typedef cPlaneEKF::Matrix Matrix;

#endif //ROS_BRIDGE_PLANE_H
