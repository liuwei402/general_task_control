
#ifndef _MOTION_GENERAL_H_
#define _MOTION_GENERAL_H_

#include <iostream>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "general_task_control/motion.h"
#include "general_task_control/motion_controlAction.h"

#include "general_task_control/CurrentPosition.h"
#include "general_task_control/MotionBase.h"

namespace GeneralTaskControl
{

using MotionControlActionClient = actionlib::SimpleActionClient<general_task_control::motion_controlAction>;
using MotionControlActionGoal = general_task_control::motion_controlGoal;
using MotionControlResultConstPtr = general_task_control::motion_controlResultConstPtr;
using MotionControlFeedbackConstPtr = general_task_control::motion_controlFeedbackConstPtr;

class MotionGeneral: public MotionBase
{
public:
    MotionGeneral(ros::NodeHandle *_nh);
    virtual ~MotionGeneral();

    /**
     * @return true motion success
     * @return false motion failed
     */
    virtual bool motion(const Task &task);

    /* send motion control action goal */
    void sendMotionControlActionGoal(const MotionControlActionGoal &goal);

    /* cancel motion control action goal */
    void cancelMotionControlActionGoal();

    /* motion control action done callback */
    void done(const actionlib::SimpleClientGoalState &state,
              const MotionControlResultConstPtr &result);

    /* motion control action active callback */
    void active();

    /* motion control action feedback callback */
    void feedback(const MotionControlFeedbackConstPtr &feedback);

private:
    ros::NodeHandle *nh;

    MotionControlActionClient motionControlActionClient;

    CurrentPosition currentPosition;

    bool motionArrived;

    std::mutex motionArrivedMutex;

};

} /* namespace GeneralTaskControl */

#endif