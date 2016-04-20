#pragma once

//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

//--------------------------------------------------------------
//
//
// Robot following targets on a surface EXAMPLE
//
//
//--------------------------------------------------------------

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxGameCamera.h"
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

    
        ofMesh srf;
  
<<<<<<< HEAD
    
<<<<<<< HEAD
        RobotParameters parameters;
    
        void setupUserPanel();
        void setupDebugPanel();
        void setupCameras();
        void setupGeometry();
        void drawGeometry();
        void drawGUI();
    
        ofxPanel panel;
        ofxPanel panelJoints;
        ofxPanel panelTargetJoints;
        ofxPanel panelJointsIK;
        ofxPanel panelJointsSpeed;
        
        
        RobotController robot;
        float acceleration;
        vector<double> speeds;


    
=======
>>>>>>> origin/master
=======
    
>>>>>>> origin/master
        /// \brief example toolpath for projecting
        void buildToolpath(ofPolyline &path);
    
        /// \brief orthogonal projection of a 2D curve onto a 3D surface
        /// \param mesh 3D surface for proection
        /// \param path2D 2D toolpath to project
        /// \param path resulting 3D projected toolpath
        void projectToolpath(ofMesh mesh, ofPolyline &path2D, ofPolyline &path);
    
        /// \brief 2D toolpath to project onto surface
        ofPolyline toolpath2D;
        /// \brief 3D toolpath on surface
        ofPolyline toolpath;
        /// \brief Orientation quaternions at each 3D toolpath point
        vector<ofQuaternion> toolpathOrients;

    
<<<<<<< HEAD
<<<<<<< HEAD
        int pathIndex;
    
    
        /* 3D Navigation */
=======
>>>>>>> origin/master
=======
>>>>>>> origin/master
    
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
