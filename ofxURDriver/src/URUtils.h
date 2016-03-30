//
//  URUtils.h
//  urModernDriverTest
//
//  Created by dantheman on 3/30/16.
//
//
#pragma once

/// Takes ofMatrix4x4 and returns row-major array in UR World Cords
double* toUR(ofMatrix4x4 input){
    double* T = new double[16];
    
    for(int i = 0; i < 4; i++){
        T[i] = (double)input._mat[i][0];
        T[i+(4)] = (double)input._mat[i][1];
        T[i+(8)] = (double)input._mat[i][2];
        T[i+(12)] = (double)input._mat[i][3];
    }
    return T;
}
