#include "PathController.h"


PathController::PathController():currentState(NOT_READY){
    
    
}

PathController::~PathController(){
    
    
}

void PathController::setup(){
    
    
}

void PathController::setup(vector<Path> paths){
    this->paths = paths;
    pathIndex = 0;
}

void PathController::update(){
    if(currentState == NOT_READY){
        
    }

    if (paths[pathIndex].ptIndex >= paths[pathIndex].size()-1){
        paths[pathIndex].ptIndex = 0;
        pathIndex = (pathIndex+1) % paths.size();
    }
    paths[pathIndex].getNextPose();
    
    if (pause){
        paths[pathIndex].ptIndex = (paths[pathIndex].ptIndex-1) % paths.size();
    }
    
}

void PathController::draw(){
    
    // show all paths
    for (auto &p : paths){
        p.path.draw();
    }
    
    // show active path
    paths[pathIndex].draw();
    
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