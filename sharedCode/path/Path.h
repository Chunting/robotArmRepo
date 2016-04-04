

#pragma once
#include "ofMain.h"

class Path{
public:
    virtual void setup();
    virtual void addPoint(ofVec3f pt);
    virtual void addPath(vector<ofVec3f> pts);
    virtual void addPath(ofPolyline line);
    virtual void addPaths(vector<ofPolyline> lines);
};