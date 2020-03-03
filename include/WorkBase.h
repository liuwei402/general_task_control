
#ifndef _WORK_BASE_H_
#define _WORK_BASE_H_

#include <iostream>

namespace GeneralTaskControl
{

class WorkBase
{
public:
    virtual ~WorkBase(){};

    /**
     * @return true work success
     * @return false work failed
     */
    virtual bool work(const std::string &workData) = 0;
};

} /* namespace GeneralTaskControl */

#endif