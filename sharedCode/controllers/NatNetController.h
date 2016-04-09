//
//  NatNetController.h
//  urModernDriverTest
//
//  Created by dantheman on 4/4/16.
//
//

#pragma once
#include "ofMain.h"
#include "ofxNatNet.h"
class NatNetController{
public:
    NatNetController();
    ~NatNetController();
    void setup(string myIP, string serverIP);
    void update();
    void draw();
    bool isFrameNew();
    
    vector<ofxNatNet::Marker> getCurrentMarkers();
    ofxNatNet::RigidBody getCurrentRigidBody();
    
    ofxNatNet natnet;
    ofParameter<bool> useUnlabledMarkers;
    ofParameter<bool> isMoving;
    ofParameter<bool> record;
    ofPolyline rbWorksrf;
    /// \brief Stores previous rigid bodies
    vector<ofxNatNet::RigidBody> recordedPath;
    ofxNatNet::RigidBody currentRigidBody;
    vector<ofxNatNet::Marker> markers;
    
    
    /// \brief Draws the plane, markers, and orientation axes of a given RigidBody.
    /// \param rb
    ///     Rigid Body passed by NatNet
    void drawRigidBody(const ofxNatNet::RigidBody &rb);
    void drawHistory();
 
    
    /// \brief Transforms a recorded toolpath based on the movement of a RigidBody.
    /// \param rb
    ///     Rigid Body that transforms the recorded toolpath

    
    
    /*
     Common Quaternion Values
     from: http://www.ogre3d.org/tikiwiki/Quaternion+and+Rotation+Primer
     
     w          x           y           z           Description
     
     1          0           0           0           Identity quaternion, no rotation
     0          1           0           0           180° turn around X axis
     0          0           1           0           180° turn around Y axis
     0          0           0           1           180° turn around Z axis
     sqrt(0.5)	sqrt(0.5)	0           0           90° rotation around X axis
     sqrt(0.5)	0           sqrt(0.5)	0           90° rotation around Y axis
     sqrt(0.5)	0           0           sqrt(0.5)	90° rotation around Z axis
     sqrt(0.5)	-sqrt(0.5)	0           0           -90° rotation around X axis
     sqrt(0.5)	0           -sqrt(0.5)	0           -90° rotation around Y axis
     sqrt(0.5)	0           0           -sqrt(0.5)	-90° rotation around Z axis
     */
    
    //    ofQuaternion POS_90_X = ofQuaternion(.707,   0,  0,  .707);  //  90º about X-Axis
    //    ofQuaternion POS_90_Y = ofQuaternion(0,  .707,   0,  .707);  //  90º about Y-Axis
    //    ofQuaternion POS_90_Z = ofQuaternion(0,  0,  .707,   .707);  //  90º about Z-Axis
    //    ofQuaternion NEG_90_X = ofQuaternion(-.707,   0,  0,  .707); // -90º about X-Axis
    //    ofQuaternion NEG_90_Y = ofQuaternion(0,  -.707,   0,  .707); // -90º about Y-Axis
    //    ofQuaternion NEG_90_Z = ofQuaternion(0,  0,  -.707,   .707); // -90º about Z-Axis
};