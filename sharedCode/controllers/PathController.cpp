#include "PathController.h"


PathController::PathController():currentState(NOT_READY){
    
}

PathController::~PathController(){
    
    
}

void PathController::setup(){
    pathIndex = 0;
}

void PathController::setup(vector<Path *> paths){
    this->paths = paths;
    pathIndex = 0;
}


void PathController::update(){
    
    if (!pause){

        // update the path & point indices
        if (paths[pathIndex]->getPtIndex() >= paths[pathIndex]->size()-1){ // there's a bug here with ptf :(
            paths[pathIndex]->setPtIndex(0);
            pathIndex = (pathIndex+1) % paths.size();
        }
        
        paths[pathIndex]->getNextPose();
    }
    
    

    // check if we are done drawing
    
}

ofMatrix4x4 PathController::getNextPose(){
    return paths[pathIndex]->getPoseAt(paths[pathIndex]->getPtIndex());
}

void PathController::draw(){
    
    ofPushMatrix();
    ofPushStyle();
    ofScale(1000);
    
    for (auto &p : paths){
        
        p->draw();
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

void PathController::keyPressed(int key){
    for (auto &p : paths){
        p->keyPressed(key);
    }
}

void PathController::loadPath(string file){
  
    
}