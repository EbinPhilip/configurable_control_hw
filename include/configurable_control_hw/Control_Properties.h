#ifndef __CONTROL_PROPERTIES_H__
#define __CONTROL_PROPERTIES_H__

struct Control_Properties
{
    Control_Properties()
        : position(0.0),
          velocity(0.0),
          effort(0.0)
    {
    }

    double position;
    double velocity;
    double effort;
};

// base structure for actuators, joints
struct Controllable_Entity
{
    Control_Properties command;
    Control_Properties state;
};

#endif