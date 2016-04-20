//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

#pragma once
#include "ofMain.h"
#include "ofxTiming.h"
#include "UR5KinematicModel.h"
#include "WorkSurface.h"
class TwoDWorkSurface : public WorkSurface{
public:
    TwoDWorkSurface(){
        
    };
    ~TwoDWorkSurface(){
        
    };

    void setup(RobotParameters * parameters);
    void update(Joint targetTCP);
    void draw();
    Joint getTargetPoint(float t);
    void setCorners(vector<ofPoint> pts);
    void setCorner(CORNER i, ofPoint pt);
 
    void addPoint(ofVec3f pt);
    void addStroke(ofPolyline stroke);
    void calcNormals(bool flip = false);
    /// Assign a set of strokes to a TwoDWorkSurface and add a
    /// retract/approach distance to the start and end of each stroke.
    /// @param retractDist
    ///     distance (in meters) to retract
    void addStrokes(vector<ofPolyline> strokes, float retractDist = 1);

   
  
    ofPolyline rbWorksrf;
    
    vector<ofParameter<ofPoint> > targetPoints;
    

    ofPoint crossed;
    vector<ofPoint > corners;
    ofPlanePrimitive plane;
    ofRectangle rect;


    
};