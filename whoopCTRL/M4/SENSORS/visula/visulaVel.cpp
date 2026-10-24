#include "headers/visulaVel.h"

visualVel::visualVel(){}

visualVel::vw visualVel::linReg(const float * x, const float * y, int n){
    float sumX2 = 0.0f; float sumXY = 0.0f; float sumX = 0.0f; float sumY = 0.0f;
    float sum2X = 0.0f; float a = 0.0f; float b = 0.0f;

    visualVel::vw out;
    if (n < 2) return out;

    for(int i=0;i<n;i++){sumX2 += x[i]*x[i]; sumXY += x[i]*y[i]; sumX += x[i]; sumY += y[i];}
    sum2X = sumX*sumX;
    if ((n*sumX2) - sum2X == 0.0f) return out;

    a = ((n*sumXY) - (sumX*sumY))/((n*sumX2) - sum2X);
    b = (sumY - a*sumX)/n;

    out.v = a; out.w = b;

    return out;
}
// (x/f)*Z

visualVel::visualVelocityVector visualVel::estimateVelocity(uint32_t tUs, const sensors::Attitude& att, const sensors::Vec3& rot, const sensors::ToFReading& tof, float dZ, const msg::FlowHeader& flow, const msg::PointMsg* pts, int n){
    visualVel::visualVelocityVector out;
    float sumVx = 0.0f; float sumVy = 0.0f; int used = 0;

    if (!flow.valid || !tof.valid || flow.fx <= 0.0f || flow.fy <= 0.0f || pts == nullptr || n <= 0) return out;

    float dt = (flow.t1Us - flow.t0Us) * 1e-6f;            // time between the two frames, seconds
    if (dt <= 0.0f) return out;

    float Z = tof.rangeM;

    for(int i=0;i<n;i++){
        if (!pts[i].inlier) continue;                      // RANSAC said: not moving with the floor

        // the point halfway through its motion, from the optical centre, as tan of its angle (x/f)
        float xn = (pts[i].x - pts[i].dx*0.5f - flow.cx) / flow.fx; float yn = (pts[i].y - pts[i].dy*0.5f - flow.cy) / flow.fy;

        // derotation: the flow the gyro rotation alone makes (small angles), 1 + xn*xn = sec^2 of the point's angle
        // height change: closer to the floor (dZ < 0) every point moves out from the centre, delta p = -p*dZ/Z
        sumVx += -((pts[i].dx / flow.fx) - (xn*yn*rot.x - (1.0f + xn*xn)*rot.y + yn*rot.z) + xn*dZ/Z) * Z / dt;
        sumVy += -((pts[i].dy / flow.fy) - ((1.0f + yn*yn)*rot.x - xn*yn*rot.y - xn*rot.z) + yn*dZ/Z) * Z / dt;
        used++;
    }
    if (used == 0) return out;

    out.vx = sumVx/used; out.vy = sumVy/used;              // camera frame, m/s

    return out;
}




