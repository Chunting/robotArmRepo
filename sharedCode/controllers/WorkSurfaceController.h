#pragma once
#include "ofMain.h"
#include "WorkSurface.h"
#include "ofxNatNet.h"
#include "RobotParameters.h"
#include "TwoDWorkSurface.h"
#include "ThreeDWorkSurface.h"
class WorkSurfaceController{
public:
    WorkSurfaceController();
    ~WorkSurfaceController();
    void setup(RobotParameters & params);
    void update();
    void draw();
    void updateWorksurface(vector<ofxNatNet::Marker> &markers);
    void updateWorksurface(ofxNatNet::RigidBody &rb);
    Joint getNextPoint();
    
    ThreeDWorkSurface threeDSurface;
    RobotParameters * robotParams;
    TwoDWorkSurface twoDSurface;
    ofxNatNet::RigidBody prev;
    Joint currentPt;
    float startTime;
    
};