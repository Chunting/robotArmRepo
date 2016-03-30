//
//  URMove.h
//  ofxURDriver
//
//  Created by Dan Moore on 2/20/16.
//  Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//


#pragma once
#include "ofMain.h"
#include "UR5KinematicModel.h"
#include "ur_kin.h"
#include "ofxTiming.h"
class URMove {
public:
    URMove();
    ~URMove();
    void setup();
    void update();
    void draw();
    void computeVelocities();
    void updatePathDebug();

    void addTargetPoint(Joint target);
    void urKinematics(vector<double> input);
    void urKinematics(ofMatrix4x4 input);
    void urKinematics(double o, double t, double th, double f, double fi, double s);
//    void urKinematics(vector<double> input);
    void setCurrentJointPosition(vector<double> pos);
    float getAcceleration();
    ofParameterGroup movementParams;
    vector<double> getTargetJointPos();
    vector<double> getCurrentSpeed();
    ofEasyCam cam;
    
protected:
    float distance;
    int selectSolution();
    vector<ofEasyCam> cams;
    vector<UR5KinematicModel*> previews;
    //Motion Capture Visualization
    int selectedSolution;
    URKinematics kinematics;
    vector<double> currentPose;
    vector<vector<double> > inversePosition;
    ofMatrix4x4 mat;
    ofParameter<float> maxSpeed;
    ofParameter<float> minSpeed;
    ofParameter<int> timeDiff;
    float deltaT;
    float deltaTime;
    RateTimer deltaTimer;
    vector<double> lastPosition;
    vector<double> currentJointSpeeds;
    vector<double> lastJointSpeeds;
    vector<double> acceleration;
    double avgAccel;
    double lastAvgAccel;
    deque<vector<float> > jointSpeedHistory;
    deque<Joint> positions;
//    ofEasyCam cam;
    ofPolyline targetLine;
    float totalLength;
    float totalArea;
    ofPoint nearestPoint;
    ofPoint nearestDataPoint;
    float lengthAtIndex;
    ofPoint pointAtIndex;
    ofPoint pointAtLength;
    ofPoint pointAtPercent;
    float indexAtLength;
    float sinTime;
    float sinIndex;
    float sinIndexLength;
    float lengthAtIndexSin;
    ofPoint pointAtIndexSin;
    ofPoint pointAtPercentSin;
    float angleAtIndex;
    float angleAtIndexSin;
    ofVec3f rotAtIndex;
    ofVec3f rotAtIndexSin;
    float rotMagAtIndex;
    float rotMagAtIndexSin;
    ofVec3f normalAtIndex;
    ofVec3f tangentAtIndexSin;
    ofVec3f normalAtIndexSin;
    ofVec3f rotationAtIndexSin;
    Joint targetPoint;
    Joint newTargetPoint;
    unsigned int nearestIndex;
    float rotAngle;
    ofNode node;
};