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
#include "ofxPtf.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
    
        /// \brief Use the arrow keys to move the path
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
    
        /* Robot Definitions */
    
        ofxURDriver robot;
        URMove movement;
        float acceleration;
        vector<double> speeds;
        Joint targetTCP;
    
    
        /* GUI Controls */
    
        RobotParameters parameters;
        ofxPanel panel;
        ofxPanel panelJoints;
        ofEasyCam cam;
    
    
        /* Path Generator */
    
        ofPoint centroid;
        bool pause;
        int ptIndex;
        
        
        /// \brief Creates a periodic 3D path.
        /// Adapted from: <a href="http://openframeworks.cc/ofBook/chapters/lines.html">ofBook/chapters/lines.html</a>
        ofPolyline buildPath();
        
        /// Periodic 3D path
        ofPolyline path;
        
        /// \brief Perpendicular Frame Generator
        ofxPtf ptf;
        
        /// \brief Make the z-axis of the perp frame the forward-facing axis
        ///
        /// Note: by default the X-Axis is the forward-facing axis
        ofMatrix4x4 zForward(ofMatrix4x4 originalMat);
        bool makeZForward;
        
        /// \brief Make the z-axis of the perp frame the outwards-facing axis.
        ///
        /// Note: by default the X-Axis is the forward-facing axis
        ofMatrix4x4 zOut(ofMatrix4x4 originalMat);
        bool makeZOut;
        
        /// \brief orientation of current perp frame
        ofMatrix4x4 orientation;
        
        /// \brief Creates the 2D polygon to loft along the path
        /// \param radius radius of polygon
        /// \param res resolution of polygon
        ofPolyline buildProfile(float radius, int res);
        
        /// \brief polygonal profile to loft
        ofPolyline profile;

        /* Test Paths for Orientation */
        ofPolyline path_XZ;
        ofPolyline path_YZ;
        ofPolyline path_SPIRAL;
        ofPolyline parsePts(string filename);

    
        /* 3D Navigation Helpers */
    
        void handleViewportPresets(int key);
        void hightlightViewports();
        ofMatrix4x4 savedCamMat;
        string viewportLabel;
        int activeCam;
};
