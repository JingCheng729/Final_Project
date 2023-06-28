/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef PRONKING_H
#define PRONKING_H

#include "FSM/FSMState.h"

class State_Pronking : public FSMState{
public:
    State_Pronking(CtrlComponents *ctrlComp);
    ~State_Pronking(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    float _targetPos[12] = {0.0, 0.97, -2.1, 0.0, 0.97, -2.1, 
                            0.0, 0.97, -2.1, 0.0, 0.97, -2.1};
    float _startPos[12];
    float _duration = 1000;   //steps
    float _percent = 0;       //%
};

#endif  // Pronking_H