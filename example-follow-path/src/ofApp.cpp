    //Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

    //--------------------------------------------------------------
    //
    //
    // Robot following targets on a path EXAMPLE
    //
    //
    //--------------------------------------------------------------

    //
    // This example shows you how to:
    //
    // 1. Load and interpolate a path for the robot.
    // 2. Calculate a target TCP from point on path.
    // 3. Move the robot based on a target TCP.
    // 4. Move the path while moving the robot.


#include "ofApp.h"
#include "URUtils.h"



//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(120);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_SILENT);
    
    // set up the GUI
    parameters.setup();
    panel.setup(parameters.robotArmParams);
    panel.setPosition(10, 10);
    panel.loadFromFile("settings.xml");

    // connect to the robot
    robot.setup("192.168.1.9",0, 1); // use the IP address of your robot here
    robot.start();

    // set up kinematic model
    movement.setup();
    panel.add(movement.movementParams);
    panelJoints.setup(parameters.joints);
    panelJoints.setPosition(ofGetWindowWidth()-panelJoints.getWidth()-10, 10);

    // assign speeds and disable movement
    speeds.assign(6, 0);
    parameters.bMove = false;

    // get the current pose on start up
    parameters.bCopy = true;

    
    // build 2D perp plane
    float w = .035;
    plane2D.addVertex(0,-w/2, w/2);
    plane2D.addVertex(0, w/2, w/2);
    plane2D.addVertex(0, w/2, -w/2);
    plane2D.addVertex(0,-w/2, -w/2);
    plane2D.close();
    
    // build path
    pathIndex = 0;
    centroid = ofPoint(.4,-.3,.25); // position in meters
    path = buildPath();  

}

//--------------------------------------------------------------
void ofApp::update(){
   
    // pass the current joints from the robot to the kinematic solver
    vector<double> currentJointPos = robot.getJointPositions();
    movement.setCurrentJointPosition(currentJointPos);
    
    // update GUI params
    for(int i = 0; i < currentJointPos.size(); i++){
        parameters.jointPos[i] = (float)currentJointPos[i];
    }
    parameters.tcpPosition = robot.getToolPoint();
    
    // set target TCP to a default orientation, then modify
    targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
    targetTCP.rotation*= ofQuaternion(90, ofVec3f(1, 0, 0));
 
    
    // assign the target pose to the current robot pose
    if(parameters.bCopy){
        parameters.bCopy = false;
        
        // get the robot's position
        targetTCP.position = robot.getToolPoint();
        // get the robot's orientation
        // targetTCP.rotation = .... <-- why is this working without grabbing the current orientation?
        
        // update GUI params
        parameters.targetTCPPosition = targetTCP.position;
        parameters.targetTCPOrientation = ofVec4f(targetTCP.rotation.x(), targetTCP.rotation.y(), targetTCP.rotation.z(), targetTCP.rotation.w());
        
    }
    

    // find the current point on the path
    if (path.getVertices().size() > 3){
        pathIndex = (pathIndex + 1) % path.getVertices().size();
        
        // set the current perp plane
        plane3D = planes[pathIndex];
        
        // set the orientation
        ofVec3f orient;
        if (pathIndex == path.getVertices().size()-1){
            orient = path.getVertices()[0];
        }else{
            orient = path.getVertices()[pathIndex+1];
        }
    
    }
    
    targetTCP.position = plane3D.getVertices()[0].getMiddle(plane3D.getVertices()[2]);

    
    // calculate normal of plane3D
    u = plane3D.getVertices()[3] - plane3D.getVertices()[2];
    v = plane3D.getVertices()[3] - plane3D.getVertices()[0];
    u.normalize();
    v.normalize();
    norm = u.getCrossed(v);
    norm.normalize();
    
  
    
//    targetTCP.rotation = ofQuaternion(norm);// * ofQuaternion(-90, ofVec3f(1,0,0));

    
    
    // send the target TCP to the kinematic solver
    movement.addTargetPoint(targetTCP);
    movement.update();
    
    
    // get back the target joint trajectories
    vector<double> target = movement.getTargetJointPos();
    for(int i = 0; i < target.size(); i++){
        parameters.targetJointPos[i] = (float)target[i];
    }
    
    // set the joint speeds
    vector<double> tempSpeeds;
    tempSpeeds.assign(6, 0);
    tempSpeeds = movement.getCurrentSpeed();
    for(int i = 0; i < tempSpeeds.size(); i++){
        parameters.jointVelocities[i] = (float)tempSpeeds[i];
    }
    // move the robot to the target TCP
    parameters.avgAccel = movement.getAcceleration();
    if(parameters.bMove){
        robot.setSpeed(tempSpeeds, parameters.avgAccel);
    }
    
    // update which easyCam is active
    if (ofGetMouseX() < ofGetWindowWidth()/2)
        activeCam = 0;
    else
        activeCam = 1;
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(0);
    ofEnableAlphaBlending();
    
    ofPushStyle();
    ofSetColor(255,160);
    ofDrawBitmapString("OF FPS "+ofToString(ofGetFrameRate()), 30, ofGetWindowHeight()-50);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.getThreadFPS()), 30, ofGetWindowHeight()-65);
     ofPopStyle();
    
    cam.begin(ofRectangle(0, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
    
    // show the realtime robot
    robot.model.draw();
    
   
    // show the 3D path
    ofPushMatrix();
    ofPushStyle();
    ofScale(1000); // scale from meter to millimeters for visualizing
    
    ofSetColor(ofColor::aqua);
    path.draw();
    
    
    // show the target point
    ofSetColor(ofColor::yellow, 100);
    if (path.size() > 0)
        ofDrawSphere(path.getVertices()[pathIndex], .01);

    
    // manually set the perp plane axes
    ofVec3f o = plane3D.getVertices()[0].getMiddle(plane3D.getVertices()[2]);
    u.scale(.03);
    v.scale(.03);
    norm.scale(.03);
    u += o;
    v += o;
    norm += o;
    
    // manually draw the perp plane axes
    ofSetColor(255, 0, 0);
    ofDrawLine(o, u);
    ofSetColor(0, 255, 0);
    ofDrawLine(o, v);
    ofSetColor(0, 0, 255);
    ofDrawLine(o, norm);

  
    ofSetColor(ofColor::yellow, 100);
    for (auto &plane : planes){
        plane.draw();
    }
    
    // ??? how to convert from 3 vector axis to transformation matrix ???
    ofPushMatrix();
    ofPoint p = plane3D.getVertices()[0].getMiddle(plane3D.getVertices()[2]);
    u.normalize();
    v.normalize();
    norm.normalize();
    
    ofMatrix4x4 oriented;
    oriented.set(u.x, u.y, u.z, 0,          // local X-Axis
                 v.x, v.y, v.z, 0,          // local Y-Axis
                 norm.x, norm.y, norm.z, 0, // local Z-Axis
                 p.x, p.y, p.z, 1);         // global position
    glMultMatrixf(oriented.getPtr());
//    ofDrawAxis(.03);   // not working ...
    ofPopMatrix();

    
    ofSetLineWidth(.03);
    ofSetColor(ofColor::aliceBlue);
    plane3D.draw();
 
    ofPopStyle();
    ofPopMatrix();

    cam.end();
    
    
    // draw simulated robot
    movement.draw();
    
    // draw the GUI
    panel.draw();
    panelJoints.draw();
    
    hightlightViewports();
}

//--------------------------------------------------------------
ofPolyline ofApp::buildPath(){
    planes.clear();
    ofPolyline temp;
    float freq = .150;  // robot coordinates are in meters
    float amp  = .015;
    
    ofNode n0;
    ofNode n1;
    ofNode n2;
    
    n0.setPosition(centroid);
    n1.setParent(n0);
    n1.setPosition(0, 0, freq);
    n2.setParent(n1);
    n2.setPosition(0, amp, 0);
    
    while (totalRotation < 360){
        float step = .5;
        totalRotation += step;
        n0.pan(step);
        n1.tilt(2);
        n2.tilt(-2);    // counteract spin
        
        temp.addVertex(n2.getGlobalPosition());//.rotate(90, ofVec3f(1,0,0)));

        ofPolyline oriented;
        for (auto &p : plane2D.getVertices()){

            ofVec3f o = ofVec3f(p.x,p.y,p.z);
            o = o * n2.getGlobalOrientation();
            o = o + temp.getVertices()[temp.getVertices().size()-1];
            
            oriented.addVertex(o);
        }
        oriented.close();
        planes.push_back(oriented);
    }
    
    temp.close();
    return temp;
}

//--------------------------------------------------------------

void ofApp::keyPressed(int key){
    float step = .01;   // 10 millimeters
    
    if(key == 'm'){
        parameters.bMove = !parameters.bMove;
    }
    
    else if (key == OF_KEY_UP){
        centroid.y += step;
    }else if(key == OF_KEY_DOWN){
        centroid.y -= step;
    }else if(key == OF_KEY_RIGHT){
        centroid.x += step;
    }else if(key == OF_KEY_LEFT){
        centroid.x -= step;
    }else if(key == ' '){
        centroid = ofPoint(0,0,0);
    }
    
//    path = buildPath();
    
    handleViewportPresets(key);
    
}

//--------------------------------------------------------------
void ofApp::handleViewportPresets(int key){
    
    float dist = 2000;
    float zOffset = 450;
    
    // TOP VIEW
    if (key == 't'){
        cam.reset();
        cam.setDistance(dist);
        viewportLabel = "TOP VIEW";
    }
    // LEFT VIEW
    else if (key == 'l'){
        cam.reset();
        cam.rotate(90, ofVec3f(0,1,0));
        cam.rotate(90, ofVec3f(1,0,0));
        cam.setDistance(dist);
        cam.boom(zOffset);
        viewportLabel = "LEFT VIEW";
    }
    // FRONT VIEW
    else if (key == 'f'){
        cam.reset();
        cam.rotate(90, ofVec3f(1,0,0));
        cam.setDistance(dist);
        cam.boom(zOffset);
        viewportLabel = "FRONT VIEW";
    }
    // PERSPECTIVE VIEW
    else if (key == 'p'){
        // hardcoded perspective camera
        ofMatrix4x4 perspective = ofMatrix4x4(0.721792, 0.689126, -0.0641996, 0,
                                              -0.0677436, 0.162658, 0.984354, 0,
                                              0.688787, -0.70615, 0.164089, 0,
                                              777.021, -719.789,  366.449, 1);
        cam.reset();
        cam.setGlobalPosition(perspective.getTranslation());
        cam.setGlobalOrientation(perspective.getRotate());
        viewportLabel = "PERSPECTIVE VIEW";
    }
    // CUSTOM  VIEW
    else if (key == 'c'){
        cam.reset();
        cam.setGlobalPosition(savedCamMat.getTranslation());
        cam.setGlobalOrientation(savedCamMat.getRotate());
        viewportLabel = "SAVED VIEW";
    }
    // Record custom view port
    if (key == 's'){
        savedCamMat = cam.getGlobalTransformMatrix();
        viewportLabel= "New Viewport Saved!";
        cout << ofToString(savedCamMat) << endl;
    }
}

//--------------------------------------------------------------
void ofApp::hightlightViewports(){
    ofPushStyle();
    
    float w = 6;
    ofSetLineWidth(w);
    
    // highlight right viewport
    if (activeCam == 1){
        ofSetColor(ofColor::white,80);
        ofDrawLine(ofGetWindowWidth()/2, 0, ofGetWindowWidth()/2, ofGetWindowHeight());
        ofDrawLine(ofGetWindowWidth()/2, w/2, ofGetWindowWidth(), w/2);
        ofDrawLine(ofGetWindowWidth()-w/2, 0, ofGetWindowWidth()-w/2, ofGetWindowHeight());
        ofDrawLine(ofGetWindowWidth()/2, ofGetWindowHeight()-w/2, ofGetWindowWidth(), ofGetWindowHeight()-w/2);
        ofSetColor(ofColor::white,40);
        ofDrawLine(0, w/2, ofGetWindowWidth()/2, w/2);
        ofDrawLine(w/2, 0, w/2, ofGetWindowHeight());
        ofDrawLine(0, ofGetWindowHeight()-w/2, ofGetWindowWidth()/2, ofGetWindowHeight()-w/2);
    }
    // hightligh left viewport
    else{
        ofSetLineWidth(w);
        ofSetColor(ofColor::white,80);
        ofDrawLine(ofGetWindowWidth()/2, 0, ofGetWindowWidth()/2, ofGetWindowHeight());
        ofDrawLine(0, w/2, ofGetWindowWidth()/2, w/2);
        ofDrawLine(w/2, 0, w/2, ofGetWindowHeight());
        ofDrawLine(0, ofGetWindowHeight()-w/2, ofGetWindowWidth()/2, ofGetWindowHeight()-w/2);
        ofSetColor(ofColor::white,40);
        ofDrawLine(ofGetWindowWidth()/2, w/2, ofGetWindowWidth(), w/2);
        ofDrawLine(ofGetWindowWidth()-w/2, 0, ofGetWindowWidth()-w/2, ofGetWindowHeight());
        ofDrawLine(ofGetWindowWidth()/2, ofGetWindowHeight()-w/2, ofGetWindowWidth(), ofGetWindowHeight()-w/2);
    }
    
    // show Viewport info
    
    
    ofSetColor(ofColor::white,200);
    ofDrawBitmapString(viewportLabel, 30, ofGetWindowHeight()-30);
    ofDrawBitmapString("REALTIME", ofGetWindowWidth()/2 - 90, ofGetWindowHeight()-30);
    ofDrawBitmapString(viewportLabel, ofGetWindowWidth()/2+30, ofGetWindowHeight()-30);
    ofDrawBitmapString("SIMULATED", ofGetWindowWidth() - 100, ofGetWindowHeight()-30);
    
    ofPopStyle();
}


//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
    
}
