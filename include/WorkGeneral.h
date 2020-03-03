
#ifndef _WORK_GENERAL_H_
#define _WORK_GENERAL_H_

#include <iostream>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "general_task_control/WorkBase.h"

namespace GeneralTaskControl
{

class WorkGeneral: public WorkBase
{
public:
    WorkGeneral(ros::NodeHandle *_nh);
    virtual ~WorkGeneral();

    /**
     * @return true motion success
     * @return false motion failed
     */
    virtual bool work(const std::string &workData);


private:
    ros::NodeHandle *nh;
};

} /* namespace GeneralTaskControl */

#endif