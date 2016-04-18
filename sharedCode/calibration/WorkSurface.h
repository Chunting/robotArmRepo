//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

#pragma once
#include "ofMain.h"
#include "UR5KinematicModel.h"
class WorkSurface{
public:
    WorkSurface(){
        
    };
    ~WorkSurface(){
        
    };
    enum CORNER{
        UL = 0,
        UR,
        LL,
        LR
    };
    void setup();
    void update(ofVec3f toolPointPos);
    void draw();
    void setCorners(vector<ofPoint> pts);
    void setCorner(CORNER i, ofPoint pt);
    void setRotation(float x, float y, float z);
    void addPoint(ofVec3f pt);
    void addStroke(ofPolyline stroke);
    void calcNormals();
    /// Assign a set of strokes to a worksurface and add a
    /// retract/approach distance to the start and end of each stroke.
    /// @param retractDist
    ///     distance (in meters) to retract
    void addStrokes(vector<ofPolyline> strokes, float retractDist = 1);
    void setRotationX(float x);
    void setRotationY(float x);
    void setRotationZ(float x);
    Joint getTargetPoint(float t);
    ofParameterGroup workSurfaceParams;
    ofPolyline rbWorksrf;
    ofMesh mesh;
    ofParameter<ofVec3f> position;
    ofParameter<ofVec3f> size;
    ofParameter<ofVec3f> rotation;
    ofParameter<ofVec3f> qAxis;
    ofParameter<float> qAngle;
    ofParameter<float> retractDistance;
    ofParameter<float> drawingScale;
    ofParameter<float> rotateDrawing;
    ofParameter<ofVec3f> drawingOffset;
    vector<ofParameter<ofPoint> > targetPoints;
    ofParameter<float> fixOrientationX;
    ofParameter<float> fixOrientationY;
    ofParameter<float> fixOrientationZ;
    ofQuaternion fix;
    ofQuaternion orientation;
    ofQuaternion orientationX;
    ofQuaternion orientationY;
    ofQuaternion orientationZ;
    ofPoint crossed;
    vector<ofPoint > corners;
    ofPlanePrimitive plane;
    ofRectangle rect;
    vector<ofPolyline> lines;
    vector<ofPolyline> strokes_original;
    ofPolyline workArea;
    int targetIndex;
    ofVec3f normal;
    float startTime;
    ofNode toolPoint;
    Joint targetToolPoint;
};