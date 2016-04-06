#include "WorkSurfaceController.h"
#include "URUtils.h"


WorkSurfaceController::WorkSurfaceController(){
    
}
WorkSurfaceController::~WorkSurfaceController(){
    
}

void WorkSurfaceController::setup(RobotParameters & params){
    workSurface.setup();
    robotParams = &params;
    // temp fix ... set up rigid body worksurface for drawing/following mocap
    float w = 400;
    float h = 300;
    float offset = 00;//400;
    workSurface.rbWorksrf.addVertex(ofVec3f(-w/2,  h/2 + offset, 0)); // UL
    workSurface.rbWorksrf.addVertex(ofVec3f( w/2,  h/2 + offset, 0)); // LL
    workSurface.rbWorksrf.addVertex(ofVec3f( w/2, -h/2 + offset, 0)); // LR
    workSurface.rbWorksrf.addVertex(ofVec3f(-w/2, -h/2 + offset, 0)); // LL
    workSurface.rbWorksrf.close();
}
void WorkSurfaceController::update(){
    workSurface.update(robotParams->targetTCPPosition);

}
void WorkSurfaceController::draw(){
    
}

Joint WorkSurfaceController::getNextPoint(){
    return workSurface.getTargetPoint(ofGetElapsedTimef()-startTime);
}

void WorkSurfaceController::updateWorksurface(vector<ofxNatNet::Marker> &markers){
    
    if (markers.size() != 4)
        cout << "wrong number of unlabled makers for the worksurface: " << markers.size() << endl;
    else{
        
        // update mocap worksurface
        workSurface.rbWorksrf.getVertices()[0] = markers[0];
        workSurface.rbWorksrf.getVertices()[1] = markers[1];
        workSurface.rbWorksrf.getVertices()[2] = markers[2];
        workSurface.rbWorksrf.getVertices()[3] = markers[3];
        
        // update worksurface corners
        workSurface.targetPoints[0] = toMeters(markers[0]);
        workSurface.targetPoints[1] = toMeters(markers[1]);
        workSurface.targetPoints[2] = toMeters(markers[2]);
        workSurface.targetPoints[3] = toMeters(markers[3]);
    }
    
}

void WorkSurfaceController::updateWorksurface(ofxNatNet::RigidBody &rb){
    
    // find the difference between the current transformation matrix
    ofMatrix4x4 diff = prev.matrix.getInverse() * rb.matrix;
    
    if (robotParams->bFollow){
        ofQuaternion tempQ = toMM(robotParams->targetTCP.rotation);
        ofVec3f tempP = toMM(robotParams->targetTCP.position);;
        
        tempP = tempP * diff;
        tempQ = rb.getMatrix().getRotate();
        
        robotParams->targetTCP.rotation = toMeters(tempQ);
        robotParams->targetTCP.position = toMeters(tempP);
        
    }
    
    //    // apply matrix to each of the recorded bodies
    //    for (auto &tp: natNet.recordedPath){
    //        tp.matrix *= diff;
    //
    //        // update markers
    //        for (int i=0; i<tp.markers.size(); i++)
    //            tp.markers[i] = tp.markers[i] * diff;
    //    }
    
    
    // initialize rigidbody worksurface
    if (workSurface.rbWorksrf.getVertices()[0].z == 0){
        // orient worksurface with rigid body
        workSurface.rbWorksrf.getVertices()[0] = workSurface.rbWorksrf.getVertices()[0] * rb.matrix;
        workSurface.rbWorksrf.getVertices()[1] = workSurface.rbWorksrf.getVertices()[1] * rb.matrix;
        workSurface.rbWorksrf.getVertices()[2] = workSurface.rbWorksrf.getVertices()[2] * rb.matrix;
        workSurface.rbWorksrf.getVertices()[3] = workSurface.rbWorksrf.getVertices()[3] * rb.matrix;
    }
    else{
        // update rigidbody worksurface
        workSurface.rbWorksrf.getVertices()[0] = workSurface.rbWorksrf.getVertices()[0] * diff;
        workSurface.rbWorksrf.getVertices()[1] = workSurface.rbWorksrf.getVertices()[1] * diff;
        workSurface.rbWorksrf.getVertices()[2] = workSurface.rbWorksrf.getVertices()[2] * diff;
        workSurface.rbWorksrf.getVertices()[3] = workSurface.rbWorksrf.getVertices()[3] * diff;
        
        // update worksurface corner
        workSurface.targetPoints[0] = toMeters(workSurface.rbWorksrf.getVertices()[0]);// / 1000;
        workSurface.targetPoints[1] = toMeters(workSurface.rbWorksrf.getVertices()[1]);// / 1000;
        workSurface.targetPoints[2] = toMeters(workSurface.rbWorksrf.getVertices()[2]);// / 1000;
        workSurface.targetPoints[3] = toMeters(workSurface.rbWorksrf.getVertices()[3]);// / 1000;
        
        // override worksurface orientation
        workSurface.orientation = rb.getMatrix().getRotate();
    }
    
    prev = rb;
}
