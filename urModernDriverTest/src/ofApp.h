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

#define N_CAMERAS 2

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
    ofParameter<ofVec3f> targetPointOrientation;
    ofParameter<float> avgAccel;
    vector<ofParameter<float> > jointPos;
    vector<ofParameter<float> > targetJointPos;
    vector<ofParameter<float> > jointVelocities;
    ofParameter<ofVec3f> toolPoint;
    ofParameter<bool> bMove;
    ofParameter<bool> bFigure8;
    ofParameter<bool> bTrace;
    ofParameter<bool> bFollow;
    ofParameter<bool> bCopy;
    
    ofxPanel panel;
    ofxPanel panelWorkSurface;
    ofxPanel panelJoints;
    
    bool stop;
    
    ofxNatNet natnet;
    void setupNatNet();
    void updateNatNet();
    void drawNatNet();
    
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
    float tagStartTime;
    
    // 3D Navigation
    ofEasyCam cams[N_CAMERAS];
    ofMatrix4x4 savedCamMats[N_CAMERAS];
    string viewportLabels[N_CAMERAS];
    int activeCam;
    
    /**
         Use hotkeys to cyle through preset viewports.
             @param key
                 't' = Top View      <br/>
                 'l' = Left View     <br/>
                 'f' = Front View    <br/>
                 'p' = Perspective   <br/>
                 'c' = Custom View   <br/>
                 's' = Save current for Custom View
     */
    void handleViewportPresets(int key);
    
    /**
         Highlights the active viewport.
     */
    void hightlightViewports();
};
