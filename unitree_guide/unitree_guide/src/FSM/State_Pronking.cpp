/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_Pronking.h"
#include "thirdParty/matplotlibcpp.h"

// namespace plt = matplotlibcpp;

State_Pronking::State_Pronking(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::PRONKING, "pronking"){}

void State_Pronking::enter(){
    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealStanceGain(i);
        }
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }
    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _startPos[i] = _lowState->motorState[i].q;
    }
    _ctrlComp->setAllStance();
}

void State_Pronking::run(){
    motiontime += 1;


    if ( motiontime >= 0 && motiontime < _stand)
    {
            _percent += (float)1/_stand;
            _percent = _percent > 1 ? 1 : _percent;
            for(int j=0; j<12; j++){
                _lowCmd->motorCmd[j].q = (1 - _percent)*_startPos[j] + _percent*_targetPos[j]; 
            }
    }
    else if ( motiontime >= _stand )
    {
        _timePass = dt * (motiontime - _stand);
        for(int j=0; j<12; j++){
            _currentPos[j] = _lowState->motorState[j].q;
            _currentVel[j] = _lowState->motorState[j].dq;
            _currentAcc[j] = _lowState->motorState[j].ddq;
            _currenttao[j] = _lowState->motorState[j].tauEst;
        }        

        _jointQd[0] = 0.8 + 0.2*sin(M_PI*0.5*_timePass);
        _jointQd[1] = -1.65 + 0.2*cos(M_PI*0.5*_timePass);
        _jointQd_d[0] = 0.2*M_PI*0.5*cos(M_PI*0.5*_timePass);
        _jointQd_d[1] = -0.2*M_PI*0.5*sin(M_PI*0.5*_timePass);
        _jointQd_dd[0] = -0.2*M_PI*0.5*M_PI*0.5*sin(M_PI*0.5*_timePass);
        _jointQd_dd[1] = -0.2*M_PI*0.5*M_PI*0.5*cos(M_PI*0.5*_timePass);


        _jointQ[0] = _lowState->motorState[10].q;
        _jointQ[1] = _lowState->motorState[11].q;
        _jointQ_d[0] = _lowState->motorState[10].dq;
        _jointQ_d[1] = _lowState->motorState[11].dq;
        _jointQ_dd[0] = _lowState->motorState[10].ddq;
        _jointQ_dd[1] = _lowState->motorState[11].ddq;

        for(int j=0; j<2; j++){
            _error[j] = _jointQd[j] - _jointQ[j];
            _derror[j] = _jointQd_d[j] - _jointQ_d[j];
            _r[j] = _derror[j] + _alpha * _error[j];
        }          

        _legY[0][0] = _jointQd_dd[0];
        _legY[0][1] = _jointQd_dd[0] + _jointQd_dd[1];
        _legY[0][2] = (_jointQd_dd[0] + 0.5 * _jointQd_dd[1]) * cos(_jointQ[1]) - _jointQ_d[0] * _jointQ_d[1] * sin(_jointQ[1]) - 0.5 * sin(_jointQ[1]) * _jointQ_d[1];
        _legY[0][3] = cos(_jointQ[0]);
        _legY[0][4] = cos(_jointQ[0] + _jointQ[1]);
        _legY[1][0] = 0.0;
        _legY[1][1] = _jointQd_dd[0] + _jointQd_dd[1];
        _legY[1][2] = 0.5 * cos(_jointQ[1]) * _jointQd_dd[0] + 0.5 * sin(_jointQ[1]) * _jointQ_d[0];
        _legY[1][3] = 0.0;
        _legY[1][4] = cos(_jointQ[0] + _jointQ[1]);

        for(int p_i=0; p_i<5; p_i++){
            _P[p_i] += _pHatDot[p_i] * dt;
        }
        _yp[0] = 0;
        _yp[1] = 0;
        for(int p_i = 0; p_i < 2; p_i++){
            for (int p_j = 0; p_j < 5; p_j++){
                _yp[p_i] += _legY[p_i][p_j] * _P[p_j];
            }
        }  

        for(int j=0; j<10; j++){

            _tao[j] = 0.0;

        }

        for(int j=0; j<2; j++){

            _tao[10+j] =  _K * _r[j] + _error[j] + _yp[j];//20* _error[j] + 4 * _derror[j];

        }
            for(int j=0; j<10; j++){
                _lowCmd->motorCmd[j].q = _targetPos[j]; 
            }

            for(int j=10; j<12; j++){
                     _lowCmd->motorCmd[j].mode = 0x0A;
                   _lowCmd->motorCmd[j].q = (2.146E+9f);
                   _lowCmd->motorCmd[j].dq = (16000.0f);
                   _lowCmd->motorCmd[j].Kp = 0;
                   _lowCmd->motorCmd[j].Kd = 0;                
                   _lowCmd->motorCmd[j].tau = _tao[j];
                //    _lowCmd->motorCmd[j].q = _jointQd[j-10]; 
            }


_actQ1.push_back(_jointQ[0]);
_actQ2.push_back(_jointQ[1]);
_desQ1.push_back(_jointQd[0]);
_desQ2.push_back(_jointQd[1]);
_fEst1.push_back(_yp[0]);
_fEst2.push_back(_yp[1]);
_reTao1.push_back(_tao[10]);
_reTao2.push_back(_tao[11]);
_pEst1.push_back(_P[0]);
_pEst2.push_back(_P[1]);
_pEst3.push_back(_P[2]);
_pEst4.push_back(_P[3]);
_pEst5.push_back(_P[4]);

        // for (int p_i = 0; p_i < 5; p_i++){
        //     _pEst[p_i].push_back(_P[p_i]);
        // }        

            // Eigen::Matrix<double, 12, 1> taoMatrix;
            // for(int j=0; j<12; j++){
            //     taoMatrix(j) = _tao[j]; //
            // }

            // _lowCmd->setTau(taoMatrix);

        // Compute the transpose of the matrix
        for(int p_i = 0; p_i < 2; p_i++){
            for (int p_j = 0; p_j < 5; p_j++){
                _legYT[p_j][p_i] = _legY[p_i][p_j];
            }
        }   

        for(int p_i = 0; p_i < 5; p_i++){
            for (int p_j = 0; p_j < 2; p_j++){
                _pHatDot[p_i] = _gamma * _legYT[p_i][p_j] * _r[p_j];
            }
        }   

            std::cout<< "_P[p_i]" << _P[4] << std::endl;
            std::cout<< "_error[1]" << _error[1] << std::endl;
            std::cout<< "_r[1]" << _r[1] << std::endl;
            std::cout<< "_yp[1]" << _yp[1] << std::endl;
            std::cout<< "__currenttao[j][1]" << _currenttao[11] << std::endl;
            // std::cout<< "motorState_1" << _lowState->motorState[1].q << std::endl;
            // std::cout<< "footForce_1" << _lowState->footForce[1] << std::endl;
    }

}

void State_Pronking::exit(){
 
    _percent = 0;


     _desQ.push_back(_actQ1);
     _desQ.push_back(_actQ2);
        _actQ.push_back(_desQ1);
        _actQ.push_back(_desQ2);
        _fEst.push_back(_fEst1);
        _fEst.push_back(_fEst2);
        _reTao.push_back(_reTao1);
        _reTao.push_back(_reTao2);
        _pEst.push_back(_pEst1);
        _pEst.push_back(_pEst2);
        _pEst.push_back(_pEst3);
        _pEst.push_back(_pEst4);
        _pEst.push_back(_pEst5);

    plotData(_desQ, _actQ);
    plotData(_fEst, _fEst);
    plotData(_reTao, _reTao);
    plotData(_pEst, _pEst);
}

FSMStateName State_Pronking::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::PRONKING;
    }
}

void State_Pronking::plotData(std::vector<std::vector<double>>& desd_data, std::vector<std::vector<double>>& actual_data){
    
    matplotlibcpp::figure_size(1200, 780);

    int size1 = desd_data.size();
    int size2 = actual_data.size();
    size_t numCols = (size1 > 0) ? desd_data[0].size() : 0;

    std::vector<double> time(numCols);
    for(int i = 0; i < numCols; i++){
        time[i] = i * 0.002;
    }

    // Function to choose line color based on a numeric value (0 to 5)
    auto chooseLineColor_ = [](int value) -> std::string {
        // Define an array of colors
        std::vector<std::string> colors = {"r--", "g--", "b--", "c--", "m--", "y--"};

        // Check if the value is within the valid range
        if (value >= 0 && value <= 5) {
            return colors[value];
        } else {
            return "k-";  // Default to black line for out-of-range values
        }
    };

    auto chooseLineColor = [](int value) -> std::string {
        // Define an array of colors
        std::vector<std::string> colors = {"r-", "g-", "b-", "c-", "m-", "y-"};

        // Check if the value is within the valid range
        if (value >= 0 && value <= 5) {
            return colors[value];
        } else {
            return "k-";  // Default to black line for out-of-range values
        }
    };
   
    if (size1 > 0) {
        for (int i = 0; i < size1; i++) {
            std::string label = "desired " + std::to_string(i);
            // snprintf(label, sizeof(label), "actual %d", i);
            matplotlibcpp::plot(time, actual_data[i], chooseLineColor(i));
        }
    }
    
    if (size2 >0 )
    {
        for(int i = 0; i < size2; i++){
            char label[16];
            snprintf(label, sizeof(label), "desired %d", i);
            matplotlibcpp::plot(time, desd_data[i], chooseLineColor_(i));
        }
    }

    //plt::ylim(0.4, 1.1);
    matplotlibcpp::legend();

    matplotlibcpp::show();
}