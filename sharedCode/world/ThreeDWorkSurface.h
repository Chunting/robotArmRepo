#pragma once
#include "ofMain.h"
#include "WorkSurface.h"
#include "ofxNatNet.h"
class ThreeDWorkSurface : public WorkSurface{
public:
    ThreeDWorkSurface();
    ~ThreeDWorkSurface();
    
    void setup(RobotParameters * params);
    void update(Joint currentTCP);
    void draw();
    
    ofxNatNet::RigidBody optitrackRb;

    
    Joint getTargetPose(float t);
    void addPoint(ofVec3f pt);
    void addStroke(ofPolyline stroke);
    void setCorners(vector<ofPoint> pts);
    void setCorner(CORNER i, ofPoint pt);
    void addStrokes(vector<ofPolyline> strokes, float retractDist = 1);
    
    ofQuaternion eulerToQuat(ofVec3f rotationEuler) ;
    /// \brief example toolpath for projecting
    void buildToolpath(ofPolyline &path);
    
    /// \brief orthogonal projection of a 2D curve onto a 3D surface
    /// \param mesh 3D surface for proection
    /// \param path2D 2D toolpath to project
    /// \param path resulting 3D projected toolpath
    void projectToolpath(ofMesh & mesh, vector<ofPolyline> &path2D, vector<ofPolyline> &path);
    void projectToolpath(ofMesh & mesh, ofPolyline &path2D, ofPolyline &path);
    
    /// \brief 2D toolpath to project onto surface
    ofPolyline toolpath2D;
    /// \brief 3D toolpath on surface
    ofPolyline toolpath;
    /// \brief Orientation quaternions at each 3D toolpath point
    vector<ofQuaternion> toolpathOrients;
    vector<ofPolyline> lines2D;
    ofQuaternion targetOrientation;
    int pathIndex;
    
};