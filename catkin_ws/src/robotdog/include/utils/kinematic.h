/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 From : https://github.com/unitreerobotics/unitree_guide
***********************************************************************/

#ifndef KINEMATIC_H
#define KINEMATIC_H

#include "common/mathTools.h"

// Helper functions for inverse kinematics
float q1_ik(float py, float pz, float l1){
    float q1;
    float L = sqrt(pow(py,2)+pow(pz,2)-pow(l1,2));
    q1 = atan2(pz*l1+py*L, py*l1-pz*L);
    return q1;
}

float q3_ik(float b3z, float b4z, float b){
    float q3, temp;
    temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2))/(2*fabs(b3z*b4z));
    if(temp>1) temp = 1;
    if(temp<-1) temp = -1;
    q3 = acos(temp);
    q3 = -(M_PI - q3); //0~180
    return q3;
}

float q2_ik(float q1, float q3, float px, float py, float pz, float b3z, float b4z){
    float q2, a1, a2, m1, m2;
    
    a1 = py*sin(q1) - pz*cos(q1);
    a2 = px;
    m1 = b4z*sin(q3);
    m2 = b3z + b4z*cos(q3);
    q2 = atan2(m1*a1+m2*a2, m1*a2-m2*a1);
    return q2;
}

// Inverse kinematics
Vec3 FootPose2Q(Vec3 pEe, int legID){
    const float _sideSign = (legID % 2 == 0) ? -1 : 1;
    const float _abadLinkLength = 0.08;
    const float _hipLinkLength = 0.213;
    const float _kneeLinkLength = 0.213;

    Vec3 _pHip2B;
    switch(legID){
    case 0: _pHip2B << 0.1881, -0.04675, 0; break;
    case 1: _pHip2B << 0.1881,  0.04675, 0; break;
    case 2: _pHip2B << -0.1881, -0.04675, 0; break;
    case 3: _pHip2B << -0.1881,  0.04675, 0; break;
    default: std::cerr << "[ERROR] legID must be 0-3\n"; exit(-1);
    } 

    Vec3 pEe2H = pEe - _pHip2B;

    float q1, q2, q3;
    Vec3 qResult;
    float px, py, pz;
    float b2y, b3z, b4z, a, b, c;

    px = pEe2H(0);
    py = pEe2H(1);
    pz = pEe2H(2);

    b2y = _abadLinkLength * _sideSign;
    b3z = -_hipLinkLength;
    b4z = -_kneeLinkLength;
    a = _abadLinkLength;
    c = sqrt(pow(px, 2) + pow(py, 2) + pow(pz, 2)); // whole length
    b = sqrt(pow(c, 2) - pow(a, 2)); // distance between shoulder and footpoint

    q1 = q1_ik(py, pz, b2y);
    q3 = q3_ik(b3z, b4z, b);
    q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z);

    qResult(0) = q1;
    qResult(1) = q2;
    qResult(2) = q3;

    return qResult;
}

Vec12 FeetPoses2Q(const Vec34 &vecP){
    Vec12 q;
    for(int i(0); i < 4; ++i){
        q.segment(3*i, 3) = FootPose2Q(vecP.col(i), i);
    }
    return q;
}


#endif //KINEMATIC_H