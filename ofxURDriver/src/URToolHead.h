//
//  URToolHead.h
//  ofxURDriver
//
//  Created by Dan Moore on 2/20/16.
//
//

#pragma once
#include "ofMain.h"
struct Tool{
    ofMesh mesh;
    ofVec3f bBoxMin;
    ofVec3f bBoxMax;
};

class URToolHead{
public:
    URToolHead(){
        
    };
    ~URToolHead(){
        
    };
    void setOrientation(ofQuaternion orientation);
    ofMatrix4x4 getMatrix();
    void setup();
    void update();
    void draw();
    void setTool(Tool t);
    Tool getCurrentTool();
protected:
    Tool currentTool;
    vector<Tool> availableTools;
    ofMatrix4x4 rot;
};