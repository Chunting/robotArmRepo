#pragma once
#include "ofMain.h"
#include "WorkSurface.h"
#include "ofxNatNet.h"
class ThreeDWorkSurface : public WorkSurface{
public:
    ThreeDWorkSurface();
    ~ThreeDWorkSurface();
    
    void setup(RobotParameters * params);
    
    /// \brief creates a 3D worksurface from an imported mesh
    /// \param filename mesh to import from data folder
    void setup(string filename);
    
    /// \brief creates a 3D worksurface and toolpaths from an imported mesh and polylines
    /// \param filename mesh to import from data folder
    /// \param polylines toolpaths to project onto 3D mesh
    void setup(string filename, vector<ofPolyline> polylines);
    
    void update();
    void update(Joint currentTCP);
    void draw();
    void draw(bool showNormals);
    
//    ofxNatNet::RigidBody optitrackRb;

    
    Joint getTargetPose(float t);
    void addPoint(ofVec3f pt);
    void addStroke(ofPolyline stroke);
    void setCorners(vector<ofPoint> pts);
    void setCorner(CORNER i, ofPoint pt);
    void addStrokes(vector<ofPolyline> strokes, float retractDist = 1);
    
    ofQuaternion eulerToQuat(ofVec3f rotationEuler) ;
    /// \brief example toolpath for projecting
    void buildToolpath(ofPolyline &path);
    void buildToolpath(ofPolyline &path, ofVec3f centroid);
    
    /// \brief orthogonal projection of a 2D curve onto a 3D surface
    /// \param mesh 3D surface for proection
    /// \param path2D 2D toolpath to project
    /// \param path resulting 3D projected toolpath
    void projectToolpath(ofMesh & mesh, ofPolyline &path2D, ofPolyline &path);
    
    void projectToolpath(ofMesh & mesh, vector<ofPolyline> &paths2D, vector<ofPolyline> &paths);
    
    void projectToolpath(ofMesh & mesh, vector<ofPolyline> &paths2D, vector<ofPolyline> &paths, float srfOffset);
    
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