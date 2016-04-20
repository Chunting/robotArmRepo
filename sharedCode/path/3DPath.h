#pragma once
#include "ofMain.h"
#include "ofxPtf.h"
class ThreeDPath{
public:
    /* Path Generator */
    void setup();

    ofVec3f getNextNormal();
    ofMatrix4x4 getNextPose();
    void draw();
    void keyPressed(int key);
    
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
    
    ofMatrix4x4 flip(ofMatrix4x4 originalMat);
    
    /// \brief orientation of current perp frame
    ofMatrix4x4 orientation;
    
    /// \brief Creates the 2D polygon to loft along the path
    /// \param radius radius of polygon
    /// \param res resolution of polygon
    ofPolyline buildProfile(float radius, int res);
    
    /// \brief Creates perpendicular frames on a path
    /// \param polyline path to create frames on
    void buildPerpFrames(ofPolyline polyline);
    
    /// \brief polygonal profile to loft
    ofPolyline profile;
    
    bool reverse;
    
    /* Test Paths for Orientation */
    ofPolyline path_XZ;
    ofPolyline path_YZ;
    ofPolyline path_SPIRAL;
    ofPolyline path_PERIODIC;
    void parsePts(string filename, ofPolyline &polyline);
};
