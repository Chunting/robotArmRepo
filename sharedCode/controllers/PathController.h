//
//  PathController.h
//  urModernDriverTest
//
//  Created by dantheman on 4/4/16.
//
//
#pragma once
#include "ofMain.h"
class PathController{
public:
    PathController();
    ~PathController();
    
    void setup();
    void update();
    void draw();
    void pauseDrawing();
    void startDrawing();
    void endDrawing();
    void loadPath(string file);
};