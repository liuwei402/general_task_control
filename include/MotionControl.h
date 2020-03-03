
#ifndef _MOTION_CONTROL_H_
#define _MOTION_CONTROL_H_

#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "general_task_control/motion.h"
#include "general_task_control/motion_controlAction.h"
#include "general_task_control/IotCommunicate.h"
#include "general_task_control/MotionBase.h"

namespace GeneralTaskControl
{

class MotionControl
{
public:
    enum MotionState
    {
        STOP,
        MOVING,
        FAILED,
        ARRIVED
    };

public:
    MotionControl(ros::NodeHandle *_nh);
    ~MotionControl();

    /**
     * @brief   motion control execute
     * @return  motion control status
     *          success 1
     *          failed 0
     */
    int exec(const Task &task);

    /* currentState reset */
    void currentStateReset();

private:
    void init();
    void release();

    /* stop handler */
    void stopHandler();

    /* moving handler */
    void movingHandler(const Task &task);

    /* failed handler */
    void failedHandler();

    /* arrvied handler */
    void arrivedHandler();

private:
    ros::NodeHandle *nh;

    MotionState currentState;

    MotionBase *actualMotion;

    int failedRetryCount;
};

} /* namespace GeneralTaskControl */

#endif