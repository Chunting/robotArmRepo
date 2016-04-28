//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
#pragma once
#define USE_TIMELINE
#define N_CAMERAS 2
#include "ofMain.h"
#include "ofxNatNet.h"
#include "ofxOsc.h"
#include "ofxGui.h"
#include "GMLPath.h"
#include "WorkSurface.h"
#include "ofxGameCamera.h"
#include "RobotController.h"
#include "PathController.h"
#include "RobotParameters.h"
#include "NatNetController.h"
#include "WorkSurfaceController.h"
#include "3DPath.h"
#include "ofxGizmo.h"
#ifdef USE_TIMELINE
#include "ofxTimeline.h"
#include "ofxTLNodeTrack.h"
#endif
#include "ofxSyphon.h"

//#define ENABLE_NATNET


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
    void moveArm();
    
    void setupViewports();
    void setupGUI();
    void positionGUI();
    void setupTimeline();
    /// \brief 3D mesh with paths for robot to follow

    ofxTLNodeTrack* nodeTrack;
    ofxTimeline timeline;
    
    ofRectangle viewportReal;
    ofRectangle viewportSim;
    
    RobotParameters parameters;

    ofxPanel panel;
    ofxPanel panelWorkSurface;
    ofxPanel panelJoints;
    ofxPanel panelTargetJoints;
    ofxPanel panelJointsIK;
    ofxPanel panelJointsSpeed;
    
    ofxGizmo gizmo;
    ofNode tcpNode;
    
    RobotController robot;
    NatNetController natNet;

   
    WorkSurfaceController workSurface;
    
    Joint workSurfaceTargetTCP;
    
    float acceleration;
    vector<double> speeds;


    
    ofNode parent;
    
    int count;
    
    bool move;
    bool hideRobot;
    
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
    
    ofxSyphonServer syphon;
    
};
