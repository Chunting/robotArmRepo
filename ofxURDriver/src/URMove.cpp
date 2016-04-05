//
//  URMove.cpp
//  ofxURDriver
//
//  Created by Dan Moore on 2/20/16.
//  Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
#include "URMove.h"
#include "URUtils.h"
URMove::URMove(){
    
}
URMove::~URMove(){
    
}
void URMove::setup(){
    movementParams.setName("UR Movements");
    movementParams.add(reachPoint.set("Step", false));
    movementParams.add(minSpeed.set("MIN Speed", 0.0, 0.0, TWO_PI*10));
    movementParams.add(maxSpeed.set(" MAX Speed", 0.0, 0.0, TWO_PI*10));
    movementParams.add(deltaTime.set("Delta T", 0.0, 0.0, 1.0));
    movementParams.add(targetTCPLerpSpeed.set("TCP LerpSpeed", 0.9, 0.001, 1.0));
    movementParams.add(jointSpeedLerpSpeed.set("Join LerpSpeed", 0.9, 0.001, 1.0));
    movementParams.add(jointAccelerationMultipler.set("Acceleration M", 200, 1, 1000));
    
    for(int i = 0; i < 8; i++){
        previews.push_back(new UR5KinematicModel());
        previews.back()->setup();
        cams.push_back(ofEasyCam());
        
    }
    selectedSolution = -1;
    deltaTimer.setSmoothing(true);
    distance = 0;
    acceleration.assign(6, 0.0);
    lastJointSpeeds.assign(6, 0.0);
    currentJointSpeeds.assign(6, 0);
}


void URMove::update(){
    deltaTimer.tick();
    deltaTime = deltaTimer.getPeriod();
    
    
    if(reachPoint){
        if((targetPoint.position - newTargetPoint.front().position).length() < 0.01){
            newTargetPoint.pop_front();
        }
    }
    if(newTargetPoint.size() > 0){
        targetPoint.position = targetPoint.position.interpolate(newTargetPoint.front().position, targetTCPLerpSpeed);
        targetPoint.rotation.slerp(targetTCPLerpSpeed, targetPoint.rotation, newTargetPoint.front().rotation);
    }
    
    mat.setTranslation(targetPoint.position);
    mat.setRotate(targetPoint.rotation);
    urKinematics(mat);
}

vector<double> URMove::getTargetJointPos(){
    if(selectedSolution > -1){
        return inversePosition[selectedSolution];
    }else{
        return currentPose;
    }
}
float URMove::getAcceleration(){
    return avgAccel*jointAccelerationMultipler;
}
vector<double> URMove::getCurrentSpeed(){
    computeVelocities();
    return currentJointSpeeds;
}

void URMove::setCurrentJointPosition(vector<double> pose){
    currentPose = pose;
    urKinematics(currentPose);
    selectedSolution = selectSolution();
}

void URMove::computeVelocities(){
    if(selectedSolution != -1){
        if(currentPose.size() > 0){
            lastAvgAccel = avgAccel;
            avgAccel = FLT_MIN;
            lastJointSpeeds = currentJointSpeeds;
            for(int i = 0; i < inversePosition[selectedSolution].size(); i++){
                currentJointSpeeds[i] = (inversePosition[selectedSolution][i]-currentPose[i])/deltaTime;
                float tempMin = minSpeed;
                float tempMax = maxSpeed;
                currentJointSpeeds[5] = 0;
                minSpeed = MIN(tempMin, currentJointSpeeds[i]);
                maxSpeed = MAX(tempMax, currentJointSpeeds[i]);
                if(abs(currentJointSpeeds[i]) > PI){
                    ofLog(OF_LOG_ERROR)<<"TOO FAST "<<ofToString(currentJointSpeeds[i], 10)<<endl;
                }
                
                acceleration[i] = currentJointSpeeds[i]-lastJointSpeeds[i];
                avgAccel = MAX(acceleration[i], avgAccel);
                
            }
        }
    }
}

void URMove::addTargetPoint(Joint target){
    if(reachPoint){
        if(newTargetPoint.size() > 0){
            if((newTargetPoint.back().position-target.position).length() > 0.1){
                newTargetPoint.push_back(target);
            }
        }else{
            newTargetPoint.push_back(target);
            
        }
    }else{
        newTargetPoint.push_front(target);
        if(newTargetPoint.size() > 1){
            newTargetPoint.pop_back();
        }
    }
    targetLine.addVertex(toMM(target.position));
}


void URMove::draw(){
    if(inversePosition.size() > 0){
        float x = ofGetWindowWidth()/2;
        float y = 0;
        for(int j = 0; j < cams.size(); j++){
            cams[j].begin(ofRectangle(x, y, ofGetWindowWidth()/2/4, ofGetWindowHeight()/2));
            ofPushMatrix();
            previews[j]->draw();
            ofPopMatrix();
            ofPushMatrix();
            targetLine.draw();
            ofSetColor(255, 0, 255, 200);
            ofDrawSphere(toMM(targetPoint.position), 5);
            ofSetColor(255, 255, 0, 200);
            if(newTargetPoint.size() > 0){
                ofDrawSphere(toMM(newTargetPoint.front().position), 5);
            }
            ofPopMatrix();
            cams[j].end();
            
            x+=ofGetWindowWidth()/2/4;
            
            if(x >= ofGetWindowWidth()){
                x = ofGetWindowWidth()/2;
                y = ofGetWindowHeight()/2;
            }
        }
        
    }
    
    
    
    ofPushMatrix();
    ofTranslate(ofGetWindowWidth()-100, 0);
    for(int i = 0; i < currentJointSpeeds.size(); i++){
        ofSetColor(255, 255, 0);
        if(i%2==0)
            ofSetColor(255, 0, 255);
        ofDrawRectangle(0, i*50, ofMap(currentJointSpeeds[i], -TWO_PI, TWO_PI, 0, 100, true), 50);
    }
    ofPopMatrix();
}

int URMove::selectSolution(){
    vector<int> nearestSolution;
    vector<int> count;
    if(currentPose.size() > 0 && inversePosition.size() > 0){
        vector<double> minDistances;
        vector<vector<double> > diffs;
        diffs.resize(inversePosition.size());
        nearestSolution.resize(inversePosition[0].size());
        count.resize(inversePosition.size());
        minDistances.assign(inversePosition.size(), DBL_MAX);
        for(int i = 0; i < inversePosition.size(); i++){
            diffs[i].resize(inversePosition[i].size());
            for(int j = 0; j < inversePosition[i].size(); j++){
                diffs[i][j]=(inversePosition[i][j]-abs(currentPose[j]));
            }
        }
        
        for(int i = 0; i < diffs.size(); i++){
            for(int j = 0; j < diffs[i].size(); j++){
                if(diffs[i][j] < minDistances[i]){
                    nearestSolution[j] = i;
                    minDistances[i] = diffs[i][j];
                    distance = minDistances[i];
                }
            }
        }
        vector<int> count;
        count.resize(inversePosition.size());
        for(int i =0; i < nearestSolution.size();i++){
            count[nearestSolution[i]]++;
        }
        int nearest = INT_MIN;
        int max = INT_MIN;
        for(int i = 0; i < count.size(); i++){
            if(count[i] > max){
                nearest = i;
                max = count[i];
            }
        }
        ofLog()<<"nearest "<<nearest<<endl;
        return 0;
    }else{
        return -1;
    }
}

void URMove::urKinematics(vector<double> input){
    if(input.size() == 6){
        urKinematics(input[0], input[1], input[2], input[3], input[4], input[5]);
    }
}

void URMove::urKinematics(ofMatrix4x4 input){
    double q_sols[8*6];
    double* T = new double[16];
    
    T = toUR(input);
    
    int num_sols = kinematics.inverse(T, q_sols);
    inversePosition.clear();
    for(int i=0;i<num_sols;i++){
        vector<double> fooSol;
        fooSol.push_back(q_sols[i*6]);
        fooSol.push_back(q_sols[i*6+1]);
        fooSol.push_back(q_sols[i*6+2]);
        fooSol.push_back(q_sols[i*6+3]);
        fooSol.push_back(q_sols[i*6+4]);
        fooSol.push_back(q_sols[i*6+5]);
        inversePosition.push_back(fooSol);
    }
    
    if(inversePosition.size() > 0){
        for(int i = 0; i < inversePosition.size(); i++){
            previews[i]->jointsRaw = inversePosition[i];
            //            cout<<ofToString(previews[i]->jointsRaw)<<endl;
            previews[i]->jointsProcessed.resize(previews[i]->jointsRaw.size());
            for(int j = 0; j < previews[i]->joints.size(); j++){
                if(j == 0){
                    inversePosition[i][j] = inversePosition[i][j]-PI;
                    previews[i]->jointsRaw[j]= inversePosition[i][j];
                }
                if(j == 1 || j == 3){
                    inversePosition[i][j] = inversePosition[i][j]-TWO_PI;
                    previews[i]->jointsRaw[j]= inversePosition[i][j];
                }
                previews[i]->jointsProcessed[j] = ofRadToDeg(previews[i]->jointsRaw[j]);
                if(j == 1 || j == 3){
                    previews[i]->jointsProcessed[j]+=90;
                }
                previews[i]->joints[j].rotation.makeRotate(previews[i]->jointsProcessed[j], previews[i]->joints[j].axis);
            }
        }
    }
    
}
void URMove::urKinematics(double o, double t, double th, double f, double fi, double s){
    double q[6] = {o, t, th, f, fi, s};
    double* T = new double[16];
    kinematics.forward(q, T);
    double q_sols[8*6];
    int num_sols = kinematics.inverse(T, q_sols);
    inversePosition.clear();
    for(int i=0;i<num_sols;i++){
        vector<double> fooSol;
        fooSol.push_back(q_sols[i*6+0]);
        fooSol.push_back(q_sols[i*6+1]);
        fooSol.push_back(q_sols[i*6+2]);
        fooSol.push_back(q_sols[i*6+3]);
        fooSol.push_back(q_sols[i*6+4]);
        fooSol.push_back(q_sols[i*6+5]);
        inversePosition.push_back(fooSol);
    }
}

