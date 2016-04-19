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
    
    
};
