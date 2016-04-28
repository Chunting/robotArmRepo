

#pragma once
#include "ofMain.h"

class Path{
public:
    Path(){};
    ~Path(){};
    virtual void setup(){};
    virtual ofMatrix4x4 getNextPose(){ return ofMatrix4x4();};
    virtual void addPoint(ofVec3f pt){};
    virtual void addPath(vector<ofVec3f> pts){};
    virtual void addPath(ofPolyline line){};
    virtual void addPaths(vector<ofPolyline> lines){};
    virtual void draw(){};
    int ptIndex;
    virtual int size(){return 0;};
    ofPolyline path;
};