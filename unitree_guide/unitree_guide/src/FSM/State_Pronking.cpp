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

        // Calculate the total size of the resulting 1D array
        // size_t size1 = sizeof(_jointQd) / sizeof(_jointQd[0]);
        // size_t size2 = sizeof(_jointQd_d) / sizeof(_jointQd_d[0]);
        // size_t size3 = sizeof(_jointQ_dd) / sizeof(_jointQ_dd[0]);
        // size_t totalSize = 1+ size1 + size2 + size3;


        // Use loops to concatenate the arrays
        // size_t index = 0;
        // for (size_t i = 0; i < 1; ++i) {
        //     _xd[index++] = _one;
        // }            
        // for (size_t i = 0; i < size1; ++i) {
        //     _xd[index++] = _jointQd[i];
        // }
        // for (size_t i = 0; i < size2; ++i) {
        //     _xd[index++] = _jointQd_d[i];
        // }
        // for (size_t i = 0; i < size3; ++i) {
        //     _xd[index++] = _jointQ_dd[i];
        // }

        // Compute the transpose of the matrix
        // for(int p_i = 0; p_i < 7; p_i++){
        //     for (int p_j = 0; p_j < 5; p_j++){
        //         _vHatT[p_j][p_i] = _vHat[p_i][p_j];
        //     }
        // }                     

        // // Compute the transpose of the matrix
        // for(int p_i = 0; p_i < 6; p_i++){
        //     for (int p_j = 0; p_j < 2; p_j++){
        //         _wHatT[p_j][p_i] = _wHat[p_i][p_j];
        //     }
        // }

        _xd << 1, _jointQd[0], _jointQd[1], _jointQd_d[0], _jointQd_d[1], _jointQd_dd[0], _jointQd_dd[1];
        // Compute the vxd
        // for(int p_i = 0; p_i < 5; p_i++){
        //     for (int p_j = 0; p_j < 7; p_j++){
        //         _vXd[p_i] += _vHatT[p_i][p_j] * _xd[p_j];
        //     }
        // }    
        _vXd = _vHat.transpose() * _xd;  

        // Compute the activation function
        // float sigmaVec[6] = {1.0};
        // Calculate sigma = exp(-(vq)^2/2) (5x1)
        Eigen::Matrix<float, 5, 1> sigma = (-0.5 * _vXd.array().square()).exp();

        // Create sigma_vec by adding 1 to the beginning and stacking sigma (6x1)

        _sigmaVec << 1.0, sigma;

        // Calculate the diagonal matrix -exp(-(vxd^2)/2) * vxd' (5x5)

        // Calculate sigma as a diagonal matrix
        Eigen::Matrix<float, 5, 5> diag_matrix;

        // float vxd[5] = {_vXd[0], _vXd[1], _vXd[2], _vXd[3], _vXd[4]};

        // for (int i = 0; i < 5; i++) {
        //     diag_matrix(i, i) = sigma(i);
        // }

        diag_matrix = -sigma * _vXd.transpose();
        // Multiply the diagonal matrix by _vXd
        Eigen::Matrix<float, 5, 5> result = diag_matrix * identityMatrix5;

        // Create sigma_prime by stacking [1 1 1 1 1] as the first row and diag_matrix as the second row (6x5)
        _sigmaPrimeVec << Eigen::Matrix<float, 1, 5>::Ones(5), result; 

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

        //update
        _wHat += _wHatDot * dt;
        _vHat += _vHatDot * dt;
        

        
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

        _fHat = _wHat.transpose() * _sigmaVec;

        for(int j=0; j<2; j++){

            _tao[10+j] =  _K_1 * _r[j] + _K_n * _r[j] + _error[j] + _fHat[j];

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
                _fEst1.push_back(_fHat[0]);
                _fEst2.push_back(_fHat[1]);
                _reTao1.push_back(_tao[10]);
                _reTao2.push_back(_tao[11]);


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

        ///update law
        Eigen::Matrix<float, 2, 1> rV;
        rV << _r[0], 
              _r[1];
        _wHatDot = _Gamma1 * _sigmaVec * rV.transpose();
        _vHatDot = _Gamma2 * _xd * rV.transpose() *_wHat.transpose() * _sigmaPrimeVec;
       

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


    plotData(_desQ, _actQ);
    plotData(_fEst, _fEst);
    plotData(_reTao, _reTao);

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