#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "RobotController.h"
#include "RobotParameters.h"
#include "PathController.h"
#include "Path3D.h"
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
		
        RobotParameters parameters;
        
        void setupUserPanel();
        void setupDebugPanel();
        void setupCameras();
        void drawGUI();
        void drawGeometry();
    
        ofxPanel panel;
        ofxPanel panelJoints;
        ofxPanel panelTargetJoints;
        ofxPanel panelJointsIK;
        ofxPanel panelJointsSpeed;
        
        
        RobotController robot;
        void moveArm();
    
        Path3D path;
        bool pause;
    
        PathController paths;
    
        
        // 3D Navigation
        void updateActiveCamera();
        ofEasyCam cams[N_CAMERAS];
        ofMatrix4x4 savedCamMats[N_CAMERAS];
        string viewportLabels[N_CAMERAS];
        int activeCam;
        
        /**
         Use hotkeys to cyle through preset viewports.
         @param key
             '1' = Top View      <br/>
             '2' = Left View     <br/>
             '3' = Front View    <br/>
             '4' = Perspective   <br/>
         */
        void handleViewportPresets(int key);
        
        /// Highlights the active viewport.
        void hightlightViewports();

};
