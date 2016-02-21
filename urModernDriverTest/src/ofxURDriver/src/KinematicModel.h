//
//  KinectModel.h
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
//
//

#pragma once
#include "ofMain.h"
class KinematicModel{
public:
    KinematicModel();
    ~KinematicModel();
    void setup();
    void update();
    void draw();
    
    float elapsed_time, last_time;
    ofVec3f pt;
    vector<ofNode> joints;
    vector<double> jointsRaw;
    vector<ofQuaternion> jointsQ;
    vector<ofVec3f> jointLength;
    vector<ofVec3f> angles;
    ofNode tool;
    ofNode toolD;
    ofEasyCam cam;
};
