//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
#pragma once

#include "ofMain.h"
#include "ofxNatNet.h"
#include "ofxOsc.h"
#include "ofxGui.h"
#include "GMLPath.h"
#include "WorkSurface.h"

#include "RobotController.h"
#include "PathController.h"
#include "RobotParameters.h"
#include "NatNetController.h"

#define N_CAMERAS 2
#define ENABLE_NATNET

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
    
    void updateWorksurface(const ofxNatNet::RigidBody &rb);
    
    /// \brief set the corners of the work surface using unlabled markers
    bool useUnlabledMarkers;
    
    /// \brief Transforms a recorded toolpath based on the movement of a RigidBody.
    /// \param markers
    ///     List of markers to use as corners (should be 4)
    void updateWorksurface(vector<ofxNatNet::Marker> &markers);
    
    /// \brief 3D mesh with paths for robot to follow
    WorkSurface workSurface;
    RobotParameters parameters;

    ofxPanel panel;
    ofxPanel panelWorkSurface;
    ofxPanel panelJoints;
    
    RobotController robot;
    NatNetController natNet;
    ofPolyline rbWorksrf;
    ofEasyCam cam;
    //Motion Capture OSC Server
    ofxOscSender sender;
   
    URMove movement;
    float acceleration;
    vector<double> speeds;
    
    /// \brief Tool Center Point for Robot
    ///
    /// Note that if a tool is not defined, the
    /// default TCP is Joint 5
//    Joint tcp;
    

    
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
    
    /// Highlights the active viewport.
    void hightlightViewports();
    
    



};
