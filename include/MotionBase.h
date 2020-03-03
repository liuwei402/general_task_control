
#ifndef _MOTION_BASE_H_
#define _MOTION_BASE_H_

#include <iostream>
#include "general_task_control/IotCommunicate.h"

namespace GeneralTaskControl
{

class MotionBase
{
public:
    virtual ~MotionBase(){};

    /**
     * @return true motion success
     * @return false motion failed
     */
    virtual bool motion(const Task &task) = 0;
};

} /* namespace GeneralTaskControl */

#endif