/**
Description goes here and also licesnse?
Authors: Bogdan Vlahov and Jason Gibson
**/

#ifndef STATEESTIMATOR_ESTIMATOR_H
#define STATEESTIMATOR_ESTIMATOR_H

#include <alphapilot/Utils.h>

class Estimator {
public:
    Estimator() = default;
    ~Estimator() = default;

    virtual alphapilot::drone_state latest_state() {
        return current_pose_estimate;
    }

private:
    alphapilot::drone_state current_pose_estimate;

};

#endif //STATEESTIMATOR_ESTIMATOR_H
