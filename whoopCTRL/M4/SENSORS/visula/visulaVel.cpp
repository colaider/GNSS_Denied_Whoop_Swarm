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
void visualVel::undistort(float u, float v, const msg::FlowHeader& flow, float& x, float& y) {
    const float* k = drones::cal().camDist;                 // OpenCV order: k1, k2, p1, p2, k3
    const float xd = (u - flow.cx) / flow.fx, yd = (v - flow.cy) / flow.fy;
    x = xd; y = yd;
    for (int i = 0; i < 5; i++) {                      // the model maps undistorted -> distorted: inverted by iterating
        const float r2  = x*x + y*y;
        const float rad = 1.0f + r2*(k[0] + r2*(k[1] + r2*k[4]));
        const float tx  = 2.0f*k[2]*x*y + k[3]*(r2 + 2.0f*x*x);
        const float ty  = k[2]*(r2 + 2.0f*y*y) + 2.0f*k[3]*x*y;
        x = (xd - tx) / rad; y = (yd - ty) / rad;
    }
}

// (x/f)*Z

visualVel::visualVelocityVector visualVel::estimateVelocity(uint32_t tUs, const sensors::Attitude& att, const sensors::Vec3& rot, const sensors::ToFReading& tof, float dZ, const msg::FlowHeader& flow, const msg::PointMsg* pts, int n){
    visualVel::visualVelocityVector out;
    if (!flow.valid || !tof.valid || flow.fx <= 0.0f || flow.fy <= 0.0f || pts == nullptr || n <= 0) return out;

    float dt = (flow.t1Us - flow.t0Us) * 1e-6f;            // time between the two frames, seconds
    if (dt <= 0.0f) return out;

    float Z = tof.rangeM;
    static float px[MAXN], py[MAXN], qx[MAXN], qy[MAXN];
    int used = 0;
    for(int i=0;i<n && used<MAXN;i++){
        if (!pts[i].inlier) continue; 

        float x0, y0, x1, y1;
        undistort(pts[i].x - pts[i].dx, pts[i].y - pts[i].dy, flow, x0, y0);
        undistort(pts[i].x, pts[i].y, flow, x1, y1);
        const float xn = 0.5f * (x0 + x1), yn = 0.5f * (y0 + y1);

        px[used] = xn;  py[used] = yn;
        qx[used] = (x1 - x0) - (xn*yn*rot.x - (1.0f + xn*xn)*rot.y) + xn*dZ/Z;
        qy[used] = (y1 - y0) - ((1.0f + yn*yn)*rot.x - xn*yn*rot.y) + yn*dZ/Z;
        used++;
    }
    if (used == 0) return out;

    // velocity: the yaw taken out with the gyro, then the average. (The yaw from the line fit as intercept
    // was noisy, +-15..100 deg/s: points off the centre turned it into 10-50 cm/s of false speed, log 17.11 23:3x)
    float a = 0.0f, b = 0.0f;
    for(int i=0;i<used;i++){ a += qx[i] - py[i]*rot.z; b += qy[i] + px[i]*rot.z; }
    a /= used;  b /= used;
    out.vx = -a * Z / dt; out.vy = -b * Z / dt;            // camera frame, m/s

    if (used >= 4) {                                       // the yaw the camera sees, for the log only (vs the gyro)
        const vw lx = linReg(py, qx, used); const vw ly = linReg(px, qy, used);
        out.yawRate = 0.5f * (lx.v - ly.v) / dt;
    }
    return out;
}




