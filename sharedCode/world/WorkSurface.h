//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

#pragma once
#include "ofMain.h"
#include "ofxTiming.h"
#include "UR5KinematicModel.h"
#include "RobotParameters.h"
class WorkSurface{
public:
    
    WorkSurface(){};
    ~WorkSurface(){};
    
    enum CORNER{
        UL = 0,
        UR,
        LL,
        LR
    };
    
    virtual void setup(RobotParameters * parameters){};
    virtual void update(Joint _currentTCP){};
    virtual void draw(){};
    virtual Joint getTargetPoint(float t){};
    virtual void addPoint(ofVec3f pt){};
    virtual void addStroke(ofPolyline stroke){};
    virtual void setCorners(vector<ofPoint> pts){};
    virtual void setCorner(CORNER i, ofPoint pt){};
    virtual void addStrokes(vector<ofPolyline> strokes, float retractDist = 1){};
    ofParameterGroup workSurfaceParams;
protected:
    
    RobotParameters * parameters;
    ofParameter<ofVec3f> position;
    ofParameter<ofVec3f> rotation;
    ofParameter<float> retractDistance;
    ofParameter<float> drawingScale;
    ofParameter<float> rotateDrawing;
    ofParameter<ofVec3f> drawingOffset;
    ofParameter<float> feedRate;
    
    ofMesh surfaceMesh;
    Joint currentTCP;
    Joint targetTCP;
    
    vector<ofPolyline> lines;
    vector<ofPolyline> strokes_original;
};