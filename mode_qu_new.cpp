#include "Copter.h"

#if MODE_QU_NEW_ENABLED == ENABLED

/*
 * Init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool Copter::ModeQUNew::init(bool ignore_checks)
{
    return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void Copter::ModeQUNew::run()
{
    return;
}

 
#endif