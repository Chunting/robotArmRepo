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
    movementParams.add(jointSpeedLerpSpeed.set("Joint LerpSpeed", 0.9, 0.001, 1.0));
    
    movementParams.add(avgAcceleration.set("avgAcceleration", 0, 0, 200));
    movementParams.add(jointAccelerationMultipler.set("Acceleration M", 200, 1, 1000));
    movementParams.add(speedDivider.set("Speed Divider", 1, 1, 10));
    movementParams.add(followLerp.set("followLerp", 1, 0, 1.0));
    
    
    for(int i = 0; i < 8; i++){
        previews.push_back(new UR5KinematicModel());
        previews.back()->setup();
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
    
    
    
    if(newTargetPoint.size() > 0){
        targetPoint.position = targetPoint.position.interpolate(newTargetPoint.front().position, targetTCPLerpSpeed);
        targetPoint.rotation.slerp(targetTCPLerpSpeed, targetPoint.rotation, newTargetPoint.front().rotation);
        newTargetPoint.pop_front();
    }
    
    mat.setTranslation(targetPoint.position);
    mat.setRotate(targetPoint.rotation);
    selectedSolution = selectSolution();
    ofMatrix4x4 matX;
    urKinematics(mat);
    
}

vector<double> URMove::getTargetJointPos(){
    if(selectedSolution > -1){
        return previews[selectedSolution]->jointsRaw;
    }else{
        return currentPose;
    }
}
float URMove::getAcceleration(){
    avgAcceleration = avgAccel*jointAccelerationMultipler;
    return avgAccel*jointAccelerationMultipler;
}
vector<double> URMove::getCurrentSpeed(){
    computeVelocities();
    return currentJointSpeeds;
}

void URMove::setCurrentJointPosition(vector<double> pose){
    currentPose = pose;
}

void URMove::computeVelocities(){
    if(selectedSolution != -1){
        if(currentPose.size() > 0){
            lastAvgAccel = avgAccel;
            avgAccel = FLT_MIN;
            lastJointSpeeds = currentJointSpeeds;
            for(int i = 0; i < previews[selectedSolution]->jointsRaw.size(); i++){
                currentJointSpeeds[i] = (previews[selectedSolution]->jointsRaw[i]-currentPose[i])/deltaTime/speedDivider;
                float tempMin = minSpeed;
                float tempMax = maxSpeed;
                minSpeed = MIN(tempMin, currentJointSpeeds[i]);
                maxSpeed = MAX(tempMax, currentJointSpeeds[i]);
                if(abs(currentJointSpeeds[i]) > PI){
                    ofLog(OF_LOG_VERBOSE)<<"TOO FAST "<<ofToString(currentJointSpeeds[i], 10)<<endl;
                }
                
                acceleration[i] = currentJointSpeeds[i]-lastJointSpeeds[i];
                avgAccel +=acceleration[i];
                avgAccel/=2;
                
            }
        }
    }else{
        for(int i = 0; i < currentJointSpeeds.size(); i++){
            currentJointSpeeds[i] = 0;
        }
    }
}

void URMove::addTargetPoint(Joint target){
    
    newTargetPoint.push_back(target);
    
    targetLine.addVertex(toMM(target.position));
    if(targetLine.size() > 400){
        targetLine.getVertices().erase(targetLine.getVertices().begin(), targetLine.getVertices().begin()+1);
    }
}


void URMove::draw(int i){
    if(inversePosition.size() > 0 && i <inversePosition.size()){
        ofSetColor(255, 0, 255, 150);
        previews[i]->draw();
        targetLine.draw();
        ofSetColor(255, 0, 255, 200);
        ofDrawSphere(toMM(targetPoint.position), 5);
        ofSetColor(255, 255, 0, 200);
        if(newTargetPoint.size() > 0){
            ofDrawSphere(toMM(newTargetPoint.front().position), 5);
        }
    }
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
        ofLog(OF_LOG_NOTICE)<<"nearest "<<nearest<<endl;
        //        if(inversePosition.size() >= 7)
        //            return nearest;
        //        else
        return 0;
        //        return nearest;
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
    preInversePosition = inversePosition;
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
            previews[i]->jointsProcessed.resize(previews[i]->jointsRaw.size());
            for(int j = 0; j < previews[i]->joints.size(); j++){
                if(j == 0){
                    inversePosition[i][j] = inversePosition[i][j]-PI;
                }
                if(j == 1 || j == 3){
                    if(inversePosition[i][j] > PI){
                        inversePosition[i][j]  = ofMap(inversePosition[i][j], PI, TWO_PI, -PI, 0, true);
                    }
                    previews[i]->jointsRaw[j] = inversePosition[i][j];
                }
                previews[i]->jointsRaw[j] = inversePosition[i][j];
                if(preInversePosition.size() > 0){
                    if(i == selectedSolution){
                        if(preInversePosition[i][j]-inversePosition[i][j] > PI){
                            ofLog()<<"JOINT WRAPS SOL "<<ofToString(i)<<" Joint "<<ofToString(j)<<endl;
                        }
                    }
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


vector<double> URMove::getRawJointPos(){
    if(selectedSolution > -1){
        return inversePosition[selectedSolution];
    }else{
        return currentPose;
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

