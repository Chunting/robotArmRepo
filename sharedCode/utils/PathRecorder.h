#pragma once
#include "ofMain.h"

class PathRecorder{
public:
    PathRecorder();
    ~PathRecorder();

    void draw();
    void startRecording();
    void endRecording();
    void addPose(vector<ofParameter<float> > pose, float timestamp);
    
    bool recording;
    string path;
    ofFile file;
    float startTime;
    int count;
    
};