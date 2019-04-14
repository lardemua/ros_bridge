//
// Created by pedro on 15-04-2019.
//

#include <carla_ros_kfilter/ekfilter.hpp>

using namespace std;
using namespace Kalman;

typedef EKFilter<double, 1, false, false, true> MyFilter;

/**
 *
@brief Sample EKFilter implementation class, very brief
*/

class sample_A : public MyFilter {
protected:
    void makeMeasure() {}
    void makeProcess() {}
};

//int main() {}

int main() {
    sample_A filter;
    return 0;
}