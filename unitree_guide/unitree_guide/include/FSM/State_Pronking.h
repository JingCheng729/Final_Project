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
    float _targetPos[12] ={0, 0.8, -1.65, 0, 0.8, -1.65, 
                                 0, 0.8, -1.65, 0, 0.8, -1.65};;
    float _startPos[12], _currentPos[12], _currentVel[12], _currentAcc[12],  _currenttao[12];
    float _stand = 300;   //steps  300*0.002 = 0.6s
    float _timePass = 0;     //time pass in run after stand
    float _percent = 0;       //%
    float _duration = 1000;   //steps

    float _jointQd[2] = {0.0}, _jointQd_d[2] = {0.0}, _jointQd_dd[2] = {0.0};
    float _jointQ[2] = {0.0}, _jointQ_d[2] = {0.0}, _jointQ_dd[2] = {0}, _error[2] = {0.0}, _derror[2] = {0.0}, _r[2] = {0.0}, _yp[2] = {1.0};
    float _tao[12] = {0.0};

    float _legY[2][5] = {0.0}, _legYT[5][2] = {0.0}, _P[5] = {0.5}, _pHatDot[5] = {0.0};


    int motiontime = 0;
    float dt = 0.001;     // 0.001~0.01

    float _gamma = 1;
    float _alpha = 10;
    float _K = 1.5;
};

#endif  // Pronking_H