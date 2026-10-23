#include "headers/visualVel.h"

visualVel::vw visualVel::vw(float * x, float * y, n){
    float sumX2 = 0.0f; float sumXY = 0.0f; float sumX = 0.0f; float sumY = 0.0f;
    float sum2X = 0.0f; float a = 0.0f; float b = 0.0f;

    visualVel::vw out;
  
    for(int i=0;i<n;i++){sumX2 += x[i]*x[i]; sumXY += x[i]*y[i]; sumX += x[i]; sumY += y[i];}
    sum2X = sumX*sumX;

    a = ((sumY*sumX2) - (sumX*sumXY))/((n*sumX2) - sum2X);
    b = (sumY - a)/sumX;

    out.v = a; out.w = b;

    return out;
}
// (x/f)*Z

visualVel::visualVelocityVector visualVel::estimateVelocity(uint32_t tUs, const sensors::Attitude& att, const sensors::ToFReading& tof, const msg::FlowHeader& flow){
    visualVel::visualVelocityVector out;

    if (!flow.valid || !tof.valid || flow.fx <= 0.0f || flow.fy <= 0.0f) return out;   

    float dt = (flow.t1Us - flow.t0Us) * 1e-6f;            // time between the two frames, seconds
    if (dt <= 0.0f) return out;

    float Z = tof.rangeM;
    out.vx = -(flow.tx / flow.fx) * Z / dt;                
    out.vy = -(flow.ty / flow.fy) * Z / dt;

    return out;
}




