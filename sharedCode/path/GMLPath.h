//
//  GMLPath.h
//  Lemur
//
//  Created by Dan Moore on 7/7/13.
// Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#pragma once
#include "ofxGML.h"

class GMLPath {
    string filepath;
    
public:
    GMLPath(){};
    ~GMLPath(){};

    void setup(float x = 0, float y = 0, float width = 1, float height = 1);
    void loadFile(string _filepath);
    void draw();
    vector<ofPolyline> getPath(float scale);
    tagReader reader;
    vector<ofPolyline> polys;
    vector<ofPolyline> scaledLines;
    float aspectRatio;
};
