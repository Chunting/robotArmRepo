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
		
        ofEasyCam cam;
    
        ofMesh srf;
    
        void buildToolpath(ofPolyline &path);
        void projectToolpath(ofMesh mesh, ofPolyline &path2D, ofPolyline &path);
        vector<ofMeshFace> testFaces;
        vector<ofVec3f> testPts;
        vector<ofVec3f> testPtNormals;
    
        ofPolyline toolpath;
        ofPolyline toolpath2D;
    
};
