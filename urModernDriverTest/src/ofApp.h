//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
#pragma once
#define N_CAMERAS 2
#include "ofMain.h"
#include "ofxGui.h"
#include "RobotController.h"
#include "RobotParameters.h"
#include "ofxGizmo.h"
#include "ofxTimeline.h"
#include "ofxTLNodeTrack.h"
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
    
<<<<<<< HEAD
    RobotController robot;
=======
    ofxNatNet natnet;
    void setupNatNet();
    void updateNatNet();
    void drawNatNet();
>>>>>>> parent of 31eef24... Drawing on tracked surface
    
    float acceleration;
    vector<double> speeds;


    
    ofNode parent;
    
    int count;
    
   
    bool hideRobot;

    
    // 3D Navigation
    vector<ofEasyCam*> cams;
    vector<ofMatrix4x4> savedCamMats;
    vector<string> viewportLabels;
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
    
<<<<<<< HEAD
=======
    /// Draws the plane, markers, and orientation axes of a given RigidBody.
    /// @param rb
    ///     Rigid Body passed by NatNet
    void drawRigidBody(const ofxNatNet::RigidBody &rb);
    void drawHistory();

    
    /// Transforms a recorded toolpath based on the movement of a RigidBody.
    /// @param rb
    ///     Rigid Body that transforms the recorded toolpath
    void updateWorksurface(const ofxNatNet::RigidBody &rb);
    
    
    /*
        Common Quaternion Values
        from: http://www.ogre3d.org/tikiwiki/Quaternion+and+Rotation+Primer
     
         w          x           y           z           Description
         
         1          0           0           0           Identity quaternion, no rotation
         0          1           0           0           180¡ turn around X axis
         0          0           1           0           180¡ turn around Y axis
         0          0           0           1           180¡ turn around Z axis
         sqrt(0.5)	sqrt(0.5)	0           0           90¡ rotation around X axis
         sqrt(0.5)	0           sqrt(0.5)	0           90¡ rotation around Y axis
         sqrt(0.5)	0           0           sqrt(0.5)	90¡ rotation around Z axis
         sqrt(0.5)	-sqrt(0.5)	0           0           -90¡ rotation around X axis
         sqrt(0.5)	0           -sqrt(0.5)	0           -90¡ rotation around Y axis
         sqrt(0.5)	0           0           -sqrt(0.5)	-90¡ rotation around Z axis
     */
>>>>>>> parent of 31eef24... Drawing on tracked surface
};
