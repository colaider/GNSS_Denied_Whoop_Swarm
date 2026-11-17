// Messages - packet types and payload layouts, shared by the M7, the M4 and the PC tools.
// Packets are sent with comm::Communication. The PC side (frame_receiver.py) mirrors the
// layouts with Python struct formats, given next to each size check.
//
// Rules: always packed, always a size check, add new fields at the end, change both sides.

#pragma once

#include <stdint.h>

namespace msg {

enum PacketType : uint8_t {
  PKT_FRAME = 1,     // camera frame: FrameHeader + pixels
  PKT_LOG   = 2,     // text
  PKT_RAW   = 3,     // raw Bayer image: RawHeader + bytes (debug)
  PKT_CMD   = 4,     // command, first payload byte = command code
  PKT_FLOW  = 5,     // optical flow: FlowHeader + count * PointMsg

  // trajectory (PC -> M7 -> M4, the M7 only forwards; replies come back the same way)
  PKT_SEG          = 6,    // SegHeader + n * TrajPoint: one piece of the trajectory
  PKT_SEG_ACK      = 7,    // SegAck: segment accepted (or already had it)
  PKT_SEG_NACK     = 8,    // SegNack: segment refused, with the reason
  PKT_TRAJ_START   = 9,    // TrajStart: start flying the queued segments
  PKT_TRAJ_REPLACE = 10,   // TrajReplace: drop queued (not yet flown) segments from here on
  PKT_FLIGHT_CMD   = 11,   // FlightCommand: hold / land / abort / kill, skips the queue
  PKT_TELEM        = 12,   // Telemetry: M4 state at 10-20 Hz

  PKT_SENSORS      = 13,   // SensorState: what the M4's sensors measured (M4 -> PC, testing)

  // remote control (PC keyboard -> WiFi -> M7 -> M4 -> Betaflight)
  PKT_REMOTE       = 14,   // RemoteCommand: sticks / arm / disarm / motor test, ~50 Hz
  PKT_REMOTE_STATE = 15,   // RemoteState: what the M4 and the FC report, 10 Hz

  // hover test (PC -> M4: FlightCommand, M4 -> PC: HoverTelem)
  PKT_HOVER_TELEM  = 17,   // HoverTelem: controller in / out, every loop step at 50 Hz

  PKT_FRAME_ROWS   = 18,   // FrameRows + nRows * width pixels: a slice of an image (WiFi: fits a datagram)

  // Kalman filter calibration (M4 -> PC): the filters' inputs, to run them again offline
  PKT_CAL_IMU      = 19,   // CAL_IMU_PER_PACKET * CalImuSample: every IMU sample (416 Hz)
  PKT_CAL_TOF      = 20,   // CalTof: every ToF reading
  PKT_CAL_CAM      = 21,   // CalCam: every flow packet (camera velocity)

  PKT_GAINS        = 22,   // Gains: controller gains in flight (PC -> M4, autoTune.py)
  PKT_POINT_DEPTH  = 23,   // PointDepthHeader + count * PointDepthMsg: tracked points in 3D (M4 -> PC, watching)
};

// Packet types the M7 passes from the PC to the M4 unchanged.
inline bool forM4(uint8_t type) {
  return type == PKT_SEG || type == PKT_TRAJ_START || type == PKT_TRAJ_REPLACE ||
         type == PKT_FLIGHT_CMD || type == PKT_REMOTE || type == PKT_GAINS;
}

// Packet types the M7 passes from the M4 to the PC unchanged.
inline bool fromM4(uint8_t type) {
  return type == PKT_SEG_ACK || type == PKT_SEG_NACK || type == PKT_TELEM || type == PKT_LOG ||
         type == PKT_SENSORS || type == PKT_REMOTE_STATE || type == PKT_HOVER_TELEM ||
         type == PKT_CAL_IMU || type == PKT_CAL_TOF || type == PKT_CAL_CAM || type == PKT_POINT_DEPTH;
}

struct __attribute__((packed)) FrameHeader {     // PKT_FRAME payload: this + pixels
  uint32_t seq, tEndUs, grabUs, procUs, grabErrors;
  uint16_t width, height;
  float    fx, fy, cx, cy;                       // intrinsics of the image
};
static_assert(sizeof(FrameHeader) == 40, "Python '<5I2H4f'");

struct __attribute__((packed)) FrameRows {       // PKT_FRAME_ROWS payload: this + nRows * width pixels
  uint32_t seq;
  uint16_t width, height, row0;
  uint8_t  nRows;
};
static_assert(sizeof(FrameRows) == 11, "Python '<I3HB'");

struct __attribute__((packed)) RawHeader {       // PKT_RAW payload: this + raw Bayer bytes
  uint32_t seq;
  uint16_t width, height;
};
static_assert(sizeof(RawHeader) == 8, "Python '<I2H'");

struct __attribute__((packed)) FlowHeader {      // PKT_FLOW payload: this + count * PointMsg
  uint32_t seq, t0Us, t1Us, flowUs;              // t0/t1: shared clock; flowUs: compute time
  float    tx, ty, angle, scale, rmsPx, fx, fy, cx, cy;  // frame motion (pixels, radians), camera
  uint16_t nTracked, nInliers, count;            // count = PointMsg entries that follow
  uint8_t  valid;
};
static_assert(sizeof(FlowHeader) == 59, "Python '<4I9f3HB'");

struct __attribute__((packed)) PointMsg {        // one tracked point
  uint16_t id, age;
  float    x, y, dx, dy;                         // position now, motion since the last frame
  uint8_t  inlier;
};
static_assert(sizeof(PointMsg) == 21, "Python '<2H4fB'");

// The most points one flow packet carries, and its size: 59 + 96 * 21 = 2075 bytes.
// Every receiver must accept a packet this big (comm::MAX_RX_PAYLOAD).
constexpr int      FLOW_MAX_POINTS = 96;
constexpr uint32_t FLOW_MAX_BYTES  = sizeof(FlowHeader) + FLOW_MAX_POINTS * sizeof(PointMsg);

// ---------------------------------------------------------------- trajectory
// A trajectory is uploaded as a queue of segments while it is being flown: the PC keeps
// ~2 s queued ahead. Each segment's first point = the previous segment's last point
// (position + velocity), so cubic Hermite interpolation is smooth across segments.
// Positions are in the drone's own frame, x/y/z in metres, relative to where the trajectory
// starts (trajTest: the hover point, StateMachine::origin).

struct __attribute__((packed)) TrajPoint {       // one waypoint, 32 bytes
  uint32_t tMs;                                  // time from the trajectory start
  float    x, y, z;                              // position, m
  float    yaw;                                  // heading, rad
  float    vx, vy, vz;                           // velocity at this point, m/s
};
static_assert(sizeof(TrajPoint) == 32, "Python '<I7f'");

constexpr uint8_t SEG_MAX_POINTS = 6;            // keeps one segment under 256 bytes

struct __attribute__((packed)) SegHeader {       // PKT_SEG payload: this + n * TrajPoint
  uint16_t trajId;                               // which trajectory this belongs to
  uint16_t segSeq;                               // 0, 1, 2 ... (+1 per segment)
  uint8_t  n;                                    // points that follow: 2 .. SEG_MAX_POINTS
};
static_assert(sizeof(SegHeader) == 5, "Python '<2HB'");
static_assert(sizeof(SegHeader) + SEG_MAX_POINTS * sizeof(TrajPoint) <= 256,
              "a segment must fit one received packet (comm::MAX_RX_PAYLOAD)");

struct __attribute__((packed)) SegAck {          // PKT_SEG_ACK payload. A resent segment that
  uint16_t trajId, segSeq;                       // was already accepted is ACKed again.
  uint8_t  queued;                               // segments in the queue (incl. the one flying)
  uint8_t  freeSlots;                            // how many more fit
  uint32_t bufferedMs;                           // trajectory time left in the queue
};
static_assert(sizeof(SegAck) == 10, "Python '<2H2BI'");

enum NackReason : uint8_t {
  NACK_QUEUE_FULL    = 1,                        // wait for an ACK freeing a slot
  NACK_GAP           = 2,                        // segSeq is not the next one: see expectedSeq
  NACK_DISCONTINUOUS = 3,                        // first point != last point of the previous one
  NACK_LIMITS        = 4,                        // speed / acceleration / height / geofence
  NACK_BAD_FORMAT    = 5,                        // wrong size, n out of range, times not rising
  NACK_WRONG_TRAJ    = 6,                        // trajId is not the active trajectory
};

struct __attribute__((packed)) SegNack {         // PKT_SEG_NACK payload
  uint16_t trajId, segSeq;
  uint8_t  reason;                               // NackReason
  uint16_t expectedSeq;                          // the segSeq the M4 wants next
};
static_assert(sizeof(SegNack) == 7, "Python '<2HBH'");

struct __attribute__((packed)) TrajStart {       // PKT_TRAJ_START payload
  uint16_t trajId;
  uint32_t delayMs;                              // start this long after the M4 receives it
};
static_assert(sizeof(TrajStart) == 6, "Python '<HI'");

struct __attribute__((packed)) TrajReplace {     // PKT_TRAJ_REPLACE payload
  uint16_t trajId;
  uint16_t fromSegSeq;                           // drop queued segments >= this; the segment
};                                               // being flown always finishes
static_assert(sizeof(TrajReplace) == 4, "Python '<2H'");

enum FlightCmd : uint8_t {
  CMD_HOLD  = 1,                                 // stop smoothly and hold position
  CMD_LAND  = 2,                                 // land where it is
  CMD_ABORT = 3,                                 // clear the trajectory queue, then hold
  CMD_KILL  = 4,                                 // motors off now
  CMD_START = 5,                                 // take off and hover (StateMachine::start), after CMD_CALIBRATE
  CMD_PING  = 6,                                 // nothing: the PC is still there (lost link -> SAFETY)
  CMD_CALIBRATE = 7,                             // on the ground, still 2 s: gyro bias + start frame (StateMachine::calibrate)
};

struct __attribute__((packed)) FlightCommand {   // PKT_FLIGHT_CMD payload
  uint8_t cmd;                                   // FlightCmd
};
static_assert(sizeof(FlightCommand) == 1, "Python '<B'");

enum FlightState : uint8_t {
  ST_IDLE     = 0,                               // nothing queued
  ST_READY    = 1,                               // segments queued, not started
  ST_RUNNING  = 2,                               // flying the trajectory
  ST_HOLD     = 3,                               // holding position (command or trajectory end)
  ST_UNDERRUN = 4,                               // queue ran dry: braked, holding
  ST_LANDING  = 5,
  ST_DISARMED = 6,
};

struct __attribute__((packed)) Telemetry {       // PKT_TELEM payload, M4 -> PC at 10-20 Hz
  uint32_t tUs;                                  // shared clock
  uint8_t  state;                                // FlightState
  uint16_t trajId, segSeq;                       // segment being flown
  uint8_t  queued;                               // segments in the queue
  uint32_t bufferedMs;                           // trajectory time left in the queue
  float    x, y, z, yaw;                         // estimated pose (m, rad)
  float    vx, vy, vz;                           // estimated velocity, m/s
  float    ex, ey, ez;                           // tracking error = desired - estimated, m
};
static_assert(sizeof(Telemetry) == 54, "Python '<IB2HBI10f'");

// ---------------------------------------------------------------- sensor test
// bits 0-3: flags; bits 4-5: horizontal Fix, bits 6-7: vertical Fix (0 tracking, 1 coasting, 2 lost)
enum SensorFlags : uint8_t { SENS_CAM_VALID = 1, SENS_IMU_READY = 2, SENS_TOF_VALID = 4, SENS_STILL = 8,
                             SENS_HFIX_SHIFT = 4, SENS_VFIX_SHIFT = 6 };

struct __attribute__((packed)) SensorState {     // PKT_SENSORS payload, with every camera update
  uint32_t tUs;                                  // shared clock, when it was sent
  uint32_t camUs;                                // when the camera velocity was measured
  uint32_t flowSeq;                              // the camera frame it came from
  uint8_t  flags;                                // SensorFlags
  float    camX, camY, camZ, camVx, camVy;       // camera: position (z = ToF height) + velocity, world
  float    imuX, imuY, imuVx, imuVy;             // IMU alone: position + velocity, world
  float    roll, pitch, yaw;                     // attitude, rad
  float    rangeM;                               // last ToF range (along the beam), m
  float    accX, accY, accZ;                     // accelerometer, body axes, g (gravity included)
  // the camera velocity's steps (Sensors::cameraDebug), for the sanity check
  float    rotX, rotY, rotZ;                     // gyro rotation over the frame pair, camera axes, rad
  float    dZ;                                   // distance change over the frame pair, m
  float    camVxC, camVyC;                       // velocity in camera axes, m/s
  // fused (the Kalman filters, Sensors::pose / velocity)
  float    fusX, fusY, fusZ, fusVx, fusVy, fusVz;   // position (z = height above the floor) + velocity, world
  float    stdX, stdY, stdZ, stdVx, stdVy, stdVz;   // how sure the filters are, 1 std
  float    biasX, biasY, biasZ;                  // accelerometer bias found, m/s^2 (x / y body, z world)
  float    accWX, accWY, accWZ;                  // IMU world acceleration (gravity removed), m/s^2: the filters' input
  float    height;                               // ToF height, tilt corrected, m: the vertical filter's measurement
  // checking the noise values: innovation = measured - predicted, should stay within +-its std
  float    camInnovX, camInnovY, camInnovStdX, camInnovStdY;   // last camera velocity update, m/s
  float    tofInnov, tofInnovStd;                // last ToF height update, m
  float    camStd, tofStd;                       // the measurement std given to the filters
  int8_t   camResult, tofResult;                 // 2/1 applied, 0 refused (outlier), -1 too old,
                                                 // 3 reset (first after LOST); ToF 2: floor step
  uint16_t camForced, tofSteps;                  // camera forced in after refusals / floor steps, since startup
};
static_assert(sizeof(SensorState) == 215, "Python '<3IB49f2b2H'");

// ---------------------------------------------------------------- remote control
enum RemoteAction : uint8_t {
  RC_STICKS = 0,                                 // fly: sticks below (Betaflight::setSticks)
  RC_ARM    = 1,                                 // start the arm sequence (throttle forced low)
  RC_DISARM = 2,                                 // disarm now
  RC_MOTOR  = 3,                                 // motor test, disarmed only: `motor` at `throttle`
};

struct __attribute__((packed)) RemoteCommand {   // PKT_REMOTE payload, PC -> M4
  uint16_t seq;                                  // +1 per packet
  uint8_t  action;                               // RemoteAction
  uint8_t  mode;                                 // drone::FlightMode: 0 ACRO, 1 ANGLE, 2 HORIZON
  uint8_t  motor;                                // RC_MOTOR: 0..3 = motor 1..4, 255 = all
  float    roll, pitch, yaw;                     // sticks -1..1
  float    throttle;                             // 0..1 (RC_MOTOR: motor speed 0..1)
};
static_assert(sizeof(RemoteCommand) == 21, "Python '<H3B4f'");

struct __attribute__((packed)) RemoteState {     // PKT_REMOTE_STATE payload, M4 -> PC
  uint32_t tUs;                                  // shared clock
  uint16_t cmdSeq;                               // the last RemoteCommand received
  uint8_t  fcLink;                               // 1 = the FC answers
  uint8_t  armed;                                // 0 disarmed, 1 arming, 2 armed
  uint8_t  mode;                                 // FlightMode sent
  uint8_t  blockerCount;                         // Betaflight's arming blockers: the last bit (count - 1)
  uint32_t blockers;                             // is always ARM_SWITCH
  float    battery;                              // V
  float    roll, pitch, yaw;                     // the FC's attitude, deg
  uint16_t rc[8];                                // the last RC frame sent, us
};
static_assert(sizeof(RemoteState) == 46, "Python '<IH4BI4f8H'");

// ---------------------------------------------------------------- hover test
enum HoverFlags : uint8_t { HOV_ARMED = 1, HOV_CAM_OK = 2, HOV_TOF_OK = 4, HOV_FC_LINK = 8, HOV_CALIBRATED = 16 };

struct __attribute__((packed)) HoverTelem {      // PKT_HOVER_TELEM payload, M4 -> PC
  uint32_t tUs;                                  // shared clock
  uint8_t  state;                                // ctrl::StateMachine::State
  uint8_t  flags;                                // HoverFlags (CAM_OK / TOF_OK: that filter is not LOST)
  float    x, y, z, yaw;                         // fused pose, m / rad
  float    vx, vy, vz, vyaw;                     // its rate, m/s / rad/s
  float    tx, ty, tz, tyaw;                     // setpoint (StateMachine::setpoint)
  float    roll, pitch, yawRate, throttle;       // sent to Betaflight: deg, deg, deg/s, 0..1
  float    ix, iy, iz;                           // the controller's integral, m/s^2
  float    ax, ay, az;                           // IMU world acceleration (gravity removed), m/s^2
  float    attRoll, attPitch;                    // measured attitude, deg (does Betaflight follow?)
  float    battery;                              // V
  // the ToF as it came (is a height jump the sensor, the tilt correction or the drone?)
  float    tofRange;                             // last reading along the beam, m (valid or not)
  float    tofHeight;                            // last valid one, tilt corrected: the height filter's input, m
  float    tofSignal;                            // return signal, MCPS (weak: dark / far / grazing)
  uint8_t  tofValid, tofStatus;                  // our check / the sensor's range status (0 = ok)
  uint16_t floorSteps;                           // height filter jumps to the ToF so far (refused 3x in a row)
  float    camYawRate, gyroYawRate;              // turn about the camera's view axis, deg/s: fitted from the points / the
                                                 // gyro over the same frame pair (should agree; 0 = no fit)
};
static_assert(sizeof(HoverTelem) == 130, "Python '<I2B28f2BH2f'");

// ---------------------------------------------------------------- gain tuning
struct __attribute__((packed)) Gains {           // PKT_GAINS payload: PosCTRL gains, x y z yaw each
  float kp[4], kd[4], ki[4];
};
static_assert(sizeof(Gains) == 48, "Python '<12f'");

// ---------------------------------------------------------------- point depth (sensors::PointDepth)
struct __attribute__((packed)) PointDepthHeader {   // PKT_POINT_DEPTH payload: this + count * PointDepthMsg
  uint32_t tUs;                                  // shared clock
  uint8_t  count;
};
static_assert(sizeof(PointDepthHeader) == 5, "Python '<IB'");

struct __attribute__((packed)) PointDepthMsg {   // one point, the surest first
  uint16_t id;
  int16_t  x, y, z;                              // start frame, mm (z: height above the floor)
  uint16_t std;                                  // depth 1 std, mm
};
static_assert(sizeof(PointDepthMsg) == 10, "Python '<H3hH'");
constexpr int POINT_DEPTH_MAX = 48;              // 5 + 48 * 10 = 485 bytes: one WiFi datagram (508)

// ---------------------------------------------------------------- Kalman filter calibration
struct __attribute__((packed)) CalImuSample {    // one IMU sample, as the filters got it
  uint32_t tUs;                                  // shared clock
  float    ax, ay, az;                           // accelerometer, body, g (gravity included)
  float    gx, gy, gz;                           // gyro, body, rad/s, bias removed
  float    awx, awy, awz;                        // world acceleration, gravity removed, m/s^2: the filters' input
  float    yaw;                                  // rad
};
static_assert(sizeof(CalImuSample) == 44, "Python '<I10f'");
constexpr int CAL_IMU_PER_PACKET = 5;            // 220 bytes: one RPC message

struct __attribute__((packed)) CalTof {          // PKT_CAL_TOF payload
  uint32_t tUs;                                  // middle of the measurement
  float    rangeM, height, signalMcps;           // along the beam / tilt corrected (the filter's measurement), m
  uint8_t  valid, status;
  float    std;                                  // the std the filter got, m
  int8_t   result;                               // 1 applied, 0 refused, -1 too old, 2 floor step, 3 reset,
                                                 // -2 not given to the filter (invalid / tilted)
  float    innov, innovStd;                      // measured - predicted, m, and what it should be
};
static_assert(sizeof(CalTof) == 31, "Python '<I3f2Bfb2f'");

struct __attribute__((packed)) CalCam {          // PKT_CAL_CAM payload
  uint32_t tUs;                                  // middle of the frame pair: the velocity's time
  uint8_t  valid;                                // it gave a velocity (the rest only means something then)
  uint16_t nInliers;
  float    vx, vy;                               // world, m/s: the filter's measurement
  float    rmsPx, fx, pairS, rangeM, rot;        // what its std is made of: tracking error px, focal px,
                                                 // frame pair s, ToF range m, gyro rotation over the pair rad
  float    std;                                  // the std the filter got, m/s
  int8_t   result;                               // 2 applied, 0 refused, -1 too old, 3 reset, -2 no velocity
  float    innovX, innovY, innovStdX, innovStdY; // measured - predicted, m/s, and what it should be
};
static_assert(sizeof(CalCam) == 56, "Python '<IBH8fb4f'");

}  // namespace msg
