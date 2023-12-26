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

    void plotData(std::vector<std::vector<double>>& desd_data, std::vector<std::vector<double>>& actual_data);

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
    float dt = 0.002;     // 0.001~0.01

    float _gamma = 1;
    // float _alpha = 20; //ada
    float _K = 3;

    std::vector<std::vector<double>> _desQ, _actQ, _fEst, _pEst, _reTao;
    std::vector<double> _desQ1, _desQ2,  _actQ1, _actQ2, _fEst1, _fEst2,_reTao1, _reTao2, _pEst1, _pEst2, _pEst3, _pEst4, _pEst5;

    float _K_1 = 1;
    float _K_s = 1;
    float _K_n = 1;
    float _alpha = 1;
    float _one = 1.0;

    float _Gamma1 = 5;
    float _Gamma2 = 10;

    Eigen::VectorXd _xd;
    // Eigen::Matrix<float, 5, 1>_vXd;
    Eigen::Matrix<float, 7, 5> _vHat;
    Eigen::Matrix<float, 6, 2> _wHat;

    Eigen::Matrix<float, 2, 1> _fHat;

    // Eigen::Matrix<float, 5, 1> _sigma;
    // Eigen::Matrix<float, 6, 1> _sigmaVec;
    // Eigen::Matrix<float, 6, 5> _sigmaPrimeVec = Eigen::Matrix<float, 6, 5>::Ones();
        
    Eigen::MatrixXd identityMatrix5 = Eigen::MatrixXd::Identity(5, 5); // 5x5 identity matrix
    Eigen::Matrix<float, 6, 2> _wHatDot  = Eigen::Matrix<float, 6, 2>::Zero();
    Eigen::Matrix<float, 7, 5> _vHatDot = Eigen::Matrix<float, 6, 2>::Zero();


};

#endif  // Pronking_H