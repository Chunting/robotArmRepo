#include "WorkSurfaceController.h"
#include "URUtils.h"


WorkSurfaceController::WorkSurfaceController(){
    
}
WorkSurfaceController::~WorkSurfaceController(){
    
}

void WorkSurfaceController::setup(RobotParameters & params){
 
    robotParams = &params;
    twoDSurface.setup(robotParams);
    threeDSurface.setup(robotParams);
    // temp fix ... set up rigid body worksurface for drawing/following mocap
    float w = 400;
    float h = 300;
    float offset = 00;//400;

}
void WorkSurfaceController::update(){
    if(robotParams != NULL){
        twoDSurface.update(robotParams->targetTCP);
        threeDSurface.update(robotParams->targetTCP);
    }
}
void WorkSurfaceController::draw(){
    twoDSurface.draw();
    threeDSurface.draw();
}

Joint WorkSurfaceController::getNextPose(){
    return threeDSurface.getTargetPose(ofGetElapsedTimef()-startTime);
}

void WorkSurfaceController::updateWorksurface(vector<ofxNatNet::Marker> &markers){
    
//    if (markers.size() != 4)
//        cout << "wrong number of unlabled makers for the worksurface: " << markers.size() << endl;
//    else{
//        
//        // update mocap worksurface
//        twoDSurface.rbWorksrf.getVertices()[0] = markers[0];
//        twoDSurface.rbWorksrf.getVertices()[1] = markers[1];
//        twoDSurface.rbWorksrf.getVertices()[2] = markers[2];
//        twoDSurface.rbWorksrf.getVertices()[3] = markers[3];
//        
//        // update worksurface corners
//        twoDSurface.targetPoints[0] = toMeters(markers[0]);
//        twoDSurface.targetPoints[1] = toMeters(markers[1]);
//        twoDSurface.targetPoints[2] = toMeters(markers[2]);
//        twoDSurface.targetPoints[3] = toMeters(markers[3]);
//    }
    
}

void WorkSurfaceController::updateWorksurface(ofxNatNet::RigidBody &rb){

//    // find the difference between the current transformation matrix
//    ofMatrix4x4 diff = prev.matrix.getInverse() * rb.matrix;
//    
//    if (robotParams->bFollow){
//        ofQuaternion tempQ = robotParams->targetTCP.rotation;
//        ofVec3f tempP = toMM(robotParams->targetTCP.position);
//        
//        tempP = tempP * diff;
//        tempQ = rb.getMatrix().getRotate();
//        
//        robotParams->targetTCP.rotation = tempQ;
//        robotParams->targetTCP.position = toMeters(tempP);
//        
//    }
//    
//    //    // apply matrix to each of the recorded bodies
//    //    for (auto &tp: natNet.recordedPath){
//    //        tp.matrix *= diff;
//    //
//    //        // update markers
//    //        for (int i=0; i<tp.markers.size(); i++)
//    //            tp.markers[i] = tp.markers[i] * diff;
//    //    }
//    
//    
//    // initialize rigidbody worksurface
//    if (twoDSurface.rbWorksrf.getVertices()[0].z == 0){
//        // orient worksurface with rigid body
//        twoDSurface.rbWorksrf.getVertices()[0] = twoDSurface.rbWorksrf.getVertices()[0] * rb.matrix;
//        twoDSurface.rbWorksrf.getVertices()[1] = twoDSurface.rbWorksrf.getVertices()[1] * rb.matrix;
//        twoDSurface.rbWorksrf.getVertices()[2] = twoDSurface.rbWorksrf.getVertices()[2] * rb.matrix;
//        twoDSurface.rbWorksrf.getVertices()[3] = twoDSurface.rbWorksrf.getVertices()[3] * rb.matrix;
//    }
//    else{
//        // update rigidbody worksurface
//        twoDSurface.rbWorksrf.getVertices()[0] = twoDSurface.rbWorksrf.getVertices()[0] * diff;
//        twoDSurface.rbWorksrf.getVertices()[1] = twoDSurface.rbWorksrf.getVertices()[1] * diff;
//        twoDSurface.rbWorksrf.getVertices()[2] = twoDSurface.rbWorksrf.getVertices()[2] * diff;
//        twoDSurface.rbWorksrf.getVertices()[3] = twoDSurface.rbWorksrf.getVertices()[3] * diff;
//        
//        // update worksurface corner
//        twoDSurface.targetPoints[0] = toMeters(twoDSurface.rbWorksrf.getVertices()[0]);// / 1000;
//        twoDSurface.targetPoints[1] = toMeters(twoDSurface.rbWorksrf.getVertices()[1]);// / 1000;
//        twoDSurface.targetPoints[2] = toMeters(twoDSurface.rbWorksrf.getVertices()[2]);// / 1000;
//        twoDSurface.targetPoints[3] = toMeters(twoDSurface.rbWorksrf.getVertices()[3]);// / 1000;
//        
//        // override worksurface orientation
//        twoDSurface.orientation = rb.getMatrix().getRotate();
//    }
//    
//    prev = rb;

}
