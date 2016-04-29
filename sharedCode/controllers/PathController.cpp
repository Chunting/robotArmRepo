#include "PathController.h"


PathController::PathController():currentState(NOT_READY){
    
//    this->paths = *new vector<Path>;
    pathIndex = 0;
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

        if (paths[pathIndex]->getPtIndex() >= paths[pathIndex]->size()-1){ // this is a bug :(
            paths[pathIndex]->setPtIndex(0);
            pathIndex = (pathIndex+1) % paths.size();
        }
        paths[pathIndex]->getNextPose();
    
    }
    
    // check if we are done drawing
    
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

void PathController::loadPath(string file){
  
    
}