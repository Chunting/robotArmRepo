#pragma once

//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

//--------------------------------------------------------------
//
//
// Robot following targets on a path EXAMPLE
//
//
//--------------------------------------------------------------


#include "ofMain.h"
#include "URDriver.h"
#include "URMove.h"
#include "UR5KinematicModel.h"
#include "ofxGui.h"
#include "GMLPath.h"
#include "RobotParameters.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
    
        /// \brief Move the path using arrow keys
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
    
        // Robot Definitions
        ofxURDriver robot;
        URMove movement;
        float acceleration;
        vector<double> speeds;
    
        Joint targetTCP;
    
        // GUI Controls
        RobotParameters parameters;
        ofxPanel panel;
        ofEasyCam cam;
    
        // Path Generator
        ofPoint centroid;
        ofPolyline path;
        int pathIndex;
        /// \brief Draws a 3D curve with a given amplitude and frequency
        ofPolyline buildPath();
    
        // 3D Navigation Helpers
        void handleViewportPresets(int key);
        void hightlightViewports();
        ofMatrix4x4 savedCamMat;
        string viewportLabel;
        int activeCam;
};
