//
//  URUtils.h
//  urModernDriverTest
//
//  Created by dantheman on 3/30/16.
//
//
#pragma once

/// \brief Converts a 4x4 matrix to a 1D array
/// \param input ofMatrix4x4 to convert
/// \return row-major array in UR World Cords
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

/// \brief Converts a 3D point from millimeters to meters
/// \param v ofVec3f to convert
/// \return returns a copy of the point in meters
ofVec3f toMeters(ofVec3f v){
    return ofVec3f(v/1000);
}

/// \brief Converts a 3D point from meters to millimeters
/// \param v ofVec3f to convert
/// \return copy of the point in millimeters
ofVec3f toMM(ofVec3f v){
    return ofVec3f(v*1000);
}

/// \brief Converts an orientation from meters to millimeters
/// \param v ofQuaternion to convert
/// \return copy of the orientation in millimeters
ofQuaternion toMM(ofQuaternion q){
    return ofQuaternion(q*1000);
}

/// \brief Converts an orientation from millimeters to meters
/// \param v ofQuaternion to convert
/// \return copy of the orientation in meters
ofQuaternion toMeters(ofQuaternion q){
    return ofQuaternion(q/1000);
}