#pragma once

#include "ofMain.h"
#include "ofxNatNet.h"
#include "ofxOsc.h"
#include "URDriver.h"
#include "URMove.h"
#include "UR5KinematicModel.h"
#include "ofxGui.h"
#include "GMLPath.h"
#include "WorkSurface.h"
class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void exit();
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    void testMotors();
    WorkSurface workSurface;
    ofParameterGroup robotArmParams;
    ofParameterGroup joints;
    ofParameter<ofVec3f> targetPointPos;
    ofParameter<ofVec3f> targetPointAngles;
    ofParameter<float> avgAccel;
    vector<ofParameter<float> > jointPos;
    vector<ofParameter<float> > targetJointPos;
     vector<ofParameter<float> > jointVelocities;
    ofParameter<ofVec3f> toolPoint;
    ofParameter<bool> bMove;
    ofParameter<bool> figure8;
    ofParameter<bool> bTrace;
    
    ofxPanel panel;
    ofxPanel panelWorkSurface;
    ofxPanel panelJoints;
    
    bool stop;

    ofxNatNet natnet;

    ofEasyCam cam;
    //Motion Capture OSC Server
    ofxOscSender sender;
    ofxURDriver robot;
    URMove movement;
    float acceleration;
    vector<double> speeds;
    
    Joint targetPoint;
    ofNode parent;
    
    int count;
    
    bool move;
    
    GMLPath gml;
};
