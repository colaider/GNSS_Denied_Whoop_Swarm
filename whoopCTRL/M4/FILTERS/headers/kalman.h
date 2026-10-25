// Kalman filter building blocks, shared by HorizontalKF and VerticalKF. Header only.
//
//   Gaussian<N>         the estimate: state x and its covariance P
//   kfPredict()         x = F x + b,  P = F P F' + Q        (one IMU step)
//   kfUpdate()          one scalar measurement z = H x + noise, with an outlier gate
//   KfHistory<N, U, L>  the last L steps (state after each step + that step's inputs), so a
//                       measurement that arrives late can be applied at its own time and the
//                       steps after it replayed (camera ~70 ms late, ToF ~15 ms late)
//
// Measurements are applied one scalar at a time (no matrix inverse): fine when their noises
// are independent, as here. No heap memory, nothing to free.

#pragma once

#include <stdint.h>

namespace filters {

template <int N>
struct Gaussian {
  float x[N] = {};
  float P[N][N] = {};
};

// x = F x + b,  P = F P F' + Q
template <int N>
void kfPredict(Gaussian<N>& g, const float F[N][N], const float b[N], const float Q[N][N]) {
  float x[N], FP[N][N];
  for (int i = 0; i < N; i++) {
    x[i] = b[i];
    for (int k = 0; k < N; k++) x[i] += F[i][k] * g.x[k];
  }
  for (int i = 0; i < N; i++)
    for (int j = 0; j < N; j++) {
      FP[i][j] = 0.0f;
      for (int k = 0; k < N; k++) FP[i][j] += F[i][k] * g.P[k][j];
    }
  for (int i = 0; i < N; i++)
    for (int j = i; j < N; j++) {                  // symmetric: compute one half, mirror it
      float s = Q[i][j];
      for (int k = 0; k < N; k++) s += FP[i][k] * F[j][k];
      g.P[i][j] = g.P[j][i] = s;
    }
  for (int i = 0; i < N; i++) g.x[i] = x[i];
}

// One scalar measurement z = H x + noise (variance R). Rejected (returns false, nothing
// changes) when the innovation is more than `gate` standard deviations off (gate <= 0: never).
// innovation: z - H x before the update, if wanted.
template <int N>
bool kfUpdate(Gaussian<N>& g, const float H[N], float z, float R, float gate, float* innovation = nullptr) {
  float PH[N], hx = 0.0f, S = R;
  for (int i = 0; i < N; i++) {
    PH[i] = 0.0f;
    for (int k = 0; k < N; k++) PH[i] += g.P[i][k] * H[k];
    hx += H[i] * g.x[i];
  }
  for (int i = 0; i < N; i++) S += H[i] * PH[i];
  const float y = z - hx;
  if (innovation) *innovation = y;
  if (S <= 0.0f || (gate > 0.0f && y * y > gate * gate * S)) return false;

  for (int i = 0; i < N; i++) g.x[i] += PH[i] / S * y;           // K = P H' / S
  for (int i = 0; i < N; i++)
    for (int j = i; j < N; j++) g.P[i][j] = g.P[j][i] = g.P[i][j] - PH[i] * PH[j] / S;   // P - K H P
  return true;
}

// The last L filter steps. Entry i (0 = oldest) holds the state AFTER step i and step i's
// inputs, so the steps after any entry can be replayed.
template <int N, int U, int L>
class KfHistory {
 public:
  struct Entry {
    uint32_t    tUs = 0;                         // time of the step (the IMU sample)
    float       dt  = 0.0f;
    float       u[U] = {};                       // the step's inputs
    Gaussian<N> g;                               // the estimate after the step
  };

  void clear() { n_ = head_ = 0; }
  int  count() const { return n_; }
  Entry& at(int i) { return e_[(head_ - n_ + i + L) % L]; }

  void push(uint32_t tUs, float dt, const float u[U], const Gaussian<N>& g) {
    Entry& e = e_[head_];
    e.tUs = tUs;
    e.dt = dt;
    for (int k = 0; k < U; k++) e.u[k] = u[k];
    e.g = g;
    head_ = (head_ + 1) % L;
    if (n_ < L) n_++;
  }

  // The newest entry at or before tUs (wrap-safe); -1 if tUs is older than the whole history.
  int find(uint32_t tUs) {
    for (int i = n_ - 1; i >= 0; i--)
      if ((int32_t)(tUs - at(i).tUs) >= 0) return i;
    return -1;
  }

 private:
  Entry e_[L];
  int   head_ = 0, n_ = 0;
};

// A Kalman filter whose measurements may arrive late. predict() runs at every IMU sample and
// is remembered; a late measurement is applied at its own time (the step at or just before
// it), then the steps after it are replayed with their remembered inputs. The derived class
// gives the motion model (model()).
// Measurements must arrive in time order (true per sensor): an older one than the last
// applied is refused, its replay would drop the newer one.
template <int N, int U, int L>
class DelayedKF {
 public:
  virtual ~DelayedKF() {}

  // Starts over from g at time tUs (history forgotten).
  void reset(uint32_t tUs, const Gaussian<N>& g) {
    g_ = g;
    tUs_ = lastMeasUs_ = tUs;
    hist_.clear();
    rejects_ = 0;
  }

  // One step to time tUs with the inputs u. A gap longer than maxDtS is stepped as maxDtS
  // (nothing is known about it; integrating seconds of noise would only wreck P) and the
  // history before it is dropped.
  void predict(uint32_t tUs, const float u[U]) {
    float dt = (int32_t)(tUs - tUs_) * 1e-6f;
    if (dt <= 0.0f) return;
    if (dt > maxDtS_) {
      dt = maxDtS_;
      hist_.clear();
    }
    step(g_, dt, u);
    tUs_ = tUs;
    hist_.push(tUs, dt, u, g_);
  }

  // m (<= MAXM) scalar measurements, all taken at time tUs: z[k] = H[k] x + noise (variance R[k]).
  // Gated together: refused as a whole when y' S^-1 y > gate^2 * m (y = innovations, S their
  // covariance), i.e. about `gate` std off per measurement; gate <= 0: never refused. One
  // measurement of a set is never taken without the other (one axis accepted, the other
  // refused forever, was a bug). Returns m if applied, 0 if refused, -1 if tUs is too old
  // (before the history, or before the last measurement).
  // innov[k] / innovVar[k] (optional): z - H x before the update and its variance (S diagonal).
  // ordered = false: a made-up measurement (e.g. "not moving") that may be newer than real ones
  // still to come; it does not move the time order, and a later replay may drop it.
  static constexpr int MAXM = 3;
  int update(uint32_t tUs, int m, const float H[][N], const float z[], const float R[], float gate,
             float innov[] = nullptr, float innovVar[] = nullptr, bool ordered = true) {
    int i;
    Gaussian<N>* g = rewind(tUs, i, ordered);
    if (!g || m > MAXM) return -1;

    float y[MAXM], S[MAXM][MAXM];                  // innovations and their covariance H P H' + R
    for (int k = 0; k < m; k++) {
      y[k] = z[k];
      for (int a = 0; a < N; a++) y[k] -= H[k][a] * g->x[a];
      for (int l = 0; l < m; l++) {
        float s = (k == l) ? R[k] : 0.0f;
        for (int a = 0; a < N; a++)
          for (int b = 0; b < N; b++) s += H[k][a] * g->P[a][b] * H[l][b];
        S[k][l] = s;
      }
      if (innov) innov[k] = y[k];
      if (innovVar) innovVar[k] = S[k][k];
    }
    if (gate > 0.0f && mahalanobis2(m, S, y) > gate * gate * m) {
      rejects_++;                                  // nothing changed: no replay needed
      return 0;
    }
    for (int k = 0; k < m; k++) kfUpdate(*g, H[k], z[k], R[k], 0.0f);
    rejects_ = 0;
    replay(i);
    return m;
  }

  // Sets the states idx[k] to z[k] with variance R[k] at time tUs (their correlations are
  // forgotten): for a jump the filter refuses to follow (a step in the floor, a lost track).
  bool force(uint32_t tUs, int m, const int idx[], const float z[], const float R[]) {
    int i;
    Gaussian<N>* g = rewind(tUs, i, true);
    if (!g) return false;
    for (int k = 0; k < m; k++) {
      const int s = idx[k];
      g->x[s] = z[k];
      for (int a = 0; a < N; a++) g->P[s][a] = g->P[a][s] = 0.0f;
      g->P[s][s] = R[k];
    }
    rejects_ = 0;
    replay(i);
    return true;
  }

  const Gaussian<N>& estimate() const { return g_; }
  uint32_t timeUs() const { return tUs_; }
  int      rejects() const { return rejects_; }     // measurements refused in a row
  void     setMaxDt(float s) { maxDtS_ = s; }

 protected:
  // The motion over dt with the inputs u: x' = F x + b, noise Q.
  virtual void model(float dt, const float u[U], float F[N][N], float b[N], float Q[N][N]) const = 0;

 private:
  // y' S^-1 y for a small symmetric S (Gaussian elimination on a copy); huge if S is singular
  static float mahalanobis2(int m, const float S[MAXM][MAXM], const float y[MAXM]) {
    float A[MAXM][MAXM + 1];
    for (int k = 0; k < m; k++) {
      for (int l = 0; l < m; l++) A[k][l] = S[k][l];
      A[k][m] = y[k];
    }
    for (int c = 0; c < m; c++) {
      if (A[c][c] <= 0.0f) return 1e30f;
      for (int r = c + 1; r < m; r++) {
        const float f = A[r][c] / A[c][c];
        for (int l = c; l <= m; l++) A[r][l] -= f * A[c][l];
      }
    }
    float v[MAXM], d = 0.0f;                       // back substitution: v = S^-1 y
    for (int c = m - 1; c >= 0; c--) {
      float s = A[c][m];
      for (int l = c + 1; l < m; l++) s -= A[c][l] * v[l];
      v[c] = s / A[c][c];
      d += y[c] * v[c];
    }
    return d;
  }

  void step(Gaussian<N>& g, float dt, const float u[U]) const {
    float F[N][N], b[N], Q[N][N];
    model(dt, u, F, b, Q);
    kfPredict(g, F, b, Q);
  }

  // The estimate at time tUs to change (i = its history entry, -1: the current one), or nullptr.
  Gaussian<N>* rewind(uint32_t tUs, int& i, bool ordered) {
    if (ordered && (int32_t)(tUs - lastMeasUs_) < 0) return nullptr;
    i = -1;
    if (hist_.count() > 0 && (int32_t)(tUs - tUs_) < 0) {   // late: find its step
      i = hist_.find(tUs);
      if (i < 0) return nullptr;                   // older than the history
    }
    if (ordered) lastMeasUs_ = tUs;
    return i < 0 ? &g_ : &hist_.at(i).g;
  }

  // Re-runs the steps after entry i from its (changed) estimate; the last one is the current estimate.
  void replay(int i) {
    if (i < 0) return;
    Gaussian<N> g = hist_.at(i).g;
    for (int j = i + 1; j < hist_.count(); j++) {
      typename KfHistory<N, U, L>::Entry& e = hist_.at(j);
      step(g, e.dt, e.u);
      e.g = g;
    }
    g_ = g;
  }

  Gaussian<N>       g_;
  uint32_t          tUs_ = 0, lastMeasUs_ = 0;
  KfHistory<N, U, L> hist_;
  int               rejects_ = 0;
  float             maxDtS_  = 0.05f;
};

}  // namespace filters
