#include "PathController.h"


PathController::PathController():currentState(NOT_READY){
    
//    this->paths = *new vector<Path>;
    pathIndex = 0;
}

PathController::~PathController(){
    
    
}

void PathController::setup(){
    
//    this->paths = *new vector<Path>;
    pathIndex = 0;
}

void PathController::setup(vector<Path *> paths){
    
    cout << "number of points sent to path controller: " << paths[0]->size() << endl;
    
    this->paths = paths;
    
     cout << "number of points in path controller AFTER assignment: " << this->paths[0]->size() << endl;
    
    pathIndex = 0;
}


void PathController::update(){
    if(currentState == NOT_READY){
        
    }
//    
//    if (!pause){
//
//        if (paths[pathIndex].ptIndex >= paths[pathIndex].size()-1){
//            paths[pathIndex].ptIndex = 0;
//            pathIndex = (pathIndex+1) % paths.size();
//        }
//        paths[pathIndex].getNextPose();
//    
//    }
    
    // check if we are done drawing
    
}

void PathController::draw(){
    
    
//    cout << "number of points in path controller DRAW: " << this->paths[0]->size() << endl;
    ofPushMatrix();
    ofPushStyle();
    ofScale(1000);
    
    if (paths.size() > 0){
        
        this->paths[0]->draw(); // paths[0] has the polyline, but paths[0]->draw() polyline is empty.
        
//        cout << "pts in polyline: " << paths[0]->path.size() << endl;
    }
//
    
    // show all paths
    ofSetColor(ofColor::lightYellow);
    ofSetLineWidth(3);
    for (auto &p : paths){
        
        p->getPolyline().draw();
        
//        cout << "number of points in path controller DRAW: " << p->getPolyline().size() << endl;
    }
    
    ofPopStyle();
    ofPopMatrix();

    
}

void PathController::pauseDrawing(){
    pause = true;
}

void PathController::startDrawing(){
    pause = false;
}

void PathController::endDrawing(){
    
}

void PathController::loadPath(string file){
  
    
}