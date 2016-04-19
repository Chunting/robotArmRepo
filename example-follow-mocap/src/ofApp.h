		#pragma once

//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

//--------------------------------------------------------------
//
//
// Robot following a Rigid Body from an
// Optitrack Motion Capture system.
//
//
//--------------------------------------------------------------

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxGameCamera.h"
#include "NatNetController.h"
#include "RobotController.h"
#include "RobotParameters.h"
#define N_CAMERAS 2

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

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
    
        // GUI
        RobotParameters parameters;
        
        void setupUserPanel();
        void setupDebugPanel();
        void setupCameras();
        void drawGUI();
        
        ofxPanel panel;
        ofxPanel panelJoints;
        ofxPanel panelTargetJoints;
        ofxPanel panelJointsIK;
        ofxPanel panelJointsSpeed;
        
        // ROBOT
        RobotController robot;
        float acceleration;
        vector<double> speeds;
		
        // MOCAP
        NatNetController mocap;
        bool attach; // attach/detach geometry to rigid body
    
    
        /* 3D Navigation */
        
        void updateActiveCamera();
        ofxGameCamera cams[N_CAMERAS];
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
