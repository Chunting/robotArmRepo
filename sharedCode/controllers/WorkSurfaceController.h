#pragma once
#include "ofMain.h"
#include "WorkSurface.h"
#include "ofxNatNet.h"
#include "RobotParameters.h"

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
    
    RobotParameters * robotParams;
    WorkSurface workSurface;
    ofxNatNet::RigidBody prev;
    Joint currentPt;
    float startTime;
    
};