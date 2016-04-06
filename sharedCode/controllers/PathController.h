//
//  PathController.h
//  urModernDriverTest
//
//  Created by dantheman on 4/4/16.
//
//
#pragma once
#include "ofMain.h"
#include "Path.h"
#include "GMLPath.h"
#include "3DPath.h"
class PathController{
public:
    PathController();
    ~PathController();
    
    enum PathState{
        NOT_READY = 0,
        LOADED,
        DRAWING,
        PAUSED,
        FINISHED,
        READY
    };
    
    enum PathType{
        BASE_PATH = 0,
        GML_PATH,
        TWO_D_PATH,
        THREE_D_PATH,
        RECORDED_PATH
    };
    
    void setup();
    void update();
    void draw();
    void pauseDrawing();
    void startDrawing();
    void endDrawing();
    void loadPath(string file);
    
    PathState currentState;
};