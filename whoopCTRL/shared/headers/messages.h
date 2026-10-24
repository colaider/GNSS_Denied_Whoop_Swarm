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
};

// Packet types the M7 passes from the PC to the M4 unchanged.
inline bool forM4(uint8_t type) {
  return type == PKT_SEG || type == PKT_TRAJ_START || type == PKT_TRAJ_REPLACE ||
         type == PKT_FLIGHT_CMD;
}

// Packet types the M7 passes from the M4 to the PC unchanged.
inline bool fromM4(uint8_t type) {
  return type == PKT_SEG_ACK || type == PKT_SEG_NACK || type == PKT_TELEM || type == PKT_LOG ||
         type == PKT_SENSORS;
}

struct __attribute__((packed)) FrameHeader {     // PKT_FRAME payload: this + pixels
  uint32_t seq, tEndUs, grabUs, procUs, grabErrors;
  uint16_t width, height;
  float    fx, fy, cx, cy;                       // intrinsics of the image
};
static_assert(sizeof(FrameHeader) == 40, "Python '<5I2H4f'");

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

// The most points one flow packet carries, and its size: 59 + 64 * 21 = 1403 bytes.
// Every receiver must accept a packet this big (comm::MAX_RX_PAYLOAD).
constexpr int      FLOW_MAX_POINTS = 64;
constexpr uint32_t FLOW_MAX_BYTES  = sizeof(FlowHeader) + FLOW_MAX_POINTS * sizeof(PointMsg);

// ---------------------------------------------------------------- trajectory
// A trajectory is uploaded as a queue of segments while it is being flown: the PC keeps
// ~2 s queued ahead. Each segment's first point = the previous segment's last point
// (position + velocity), so cubic Hermite interpolation is smooth across segments.
// Positions are in the drone's own frame: origin = takeoff point, x/y/z in metres.

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
enum SensorFlags : uint8_t { SENS_CAM_VALID = 1, SENS_IMU_READY = 2, SENS_TOF_VALID = 4 };

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
};
static_assert(sizeof(SensorState) == 101, "Python '<3IB22f'");

}  // namespace msg
