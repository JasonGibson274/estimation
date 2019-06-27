/**
Description goes here and also licesnse?
Authors: Bogdan Vlahov and Jason Gibson
**/

#ifndef STATEESTIMATOR_ESTIMATOR_H
#define STATEESTIMATOR_ESTIMATOR_H

#include <alphapilot_common/Utils.h>

class Estimator {
public:
    Estimator() = default;
    ~Estimator() = default;
    /** Takes the latest state estimation from gtsam and returns it as a
   * drone state for other code to use
   * @return
   */
    virtual alphapilot::drone_state latest_state() {
        return current_pose_estimate_;
    }

protected:
    alphapilot::drone_state current_pose_estimate_;

};

#endif //STATEESTIMATOR_ESTIMATOR_H
