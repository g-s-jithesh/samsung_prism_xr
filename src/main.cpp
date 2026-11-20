// Path Finder — IMU-only 6-DoF trajectory estimator.
// Usage: pathfinder <imu_csv> <out_txt>
// Streams the EuRoC IMU CSV one row at a time; O(1) memory.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

struct Sample {
    unsigned long long ts;  // ns
    double w[3], a[3];
    bool valid;             // false = row present but values blank/NaN
};

// Parse "ts,wx,wy,wz,ax,ay,az". Returns false for comments/blank lines.
static bool parse_row(char* line, Sample& s) {
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '#' || *line == '\r' || *line == '\n' || *line == 0) return false;
    char* p = line;
    char* end;
    s.ts = strtoull(p, &end, 10);
    if (end == p) return false;
    p = end;
    double v[6];
    s.valid = true;
    for (int i = 0; i < 6; ++i) {
        if (*p == ',') ++p;
        v[i] = strtod(p, &end);
        if (end == p || std::isnan(v[i]) || std::isinf(v[i])) s.valid = false;
        p = end;
        while (*p && *p != ',') ++p;  // skip unparsable field
    }
    for (int i = 0; i < 3; ++i) { s.w[i] = v[i]; s.a[i] = v[i + 3]; }
    return true;
}

// ---- small vector/quaternion helpers (q = w,x,y,z; R(q) maps body -> world) ----
static void cross(const double a[3], const double b[3], double o[3]) {
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
static double norm3(const double a[3]) { return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }

static void qnormalize(double q[4]) {
    double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int i = 0; i < 4; ++i) q[i] /= n;
}
// q <- q * exp(0.5 * th), th = rotation vector in body frame
static void qintegrate(double q[4], const double th[3]) {
    double ang = norm3(th), h = 0.5 * ang;
    double s = ang > 1e-12 ? sin(h) / ang : 0.5, c = cos(h);
    double d[4] = {c, th[0] * s, th[1] * s, th[2] * s};
    double r[4] = {
        q[0] * d[0] - q[1] * d[1] - q[2] * d[2] - q[3] * d[3],
        q[0] * d[1] + q[1] * d[0] + q[2] * d[3] - q[3] * d[2],
        q[0] * d[2] - q[1] * d[3] + q[2] * d[0] + q[3] * d[1],
        q[0] * d[3] + q[1] * d[2] - q[2] * d[1] + q[3] * d[0]};
    memcpy(q, r, sizeof r);
    qnormalize(q);
}
// v_world = R(q) v_body
static void rotate(const double q[4], const double v[3], double o[3]) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    o[0] = (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - w * z) * v[1] + 2 * (x * z + w * y) * v[2];
    o[1] = 2 * (x * y + w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - w * x) * v[2];
    o[2] = 2 * (x * z - w * y) * v[0] + 2 * (y * z + w * x) * v[1] + (1 - 2 * (x * x + y * y)) * v[2];
}

// ---- estimator ----
static const double INIT_SEC = 1.0;   // static window for bias/gravity
static const double KP = 0.5;         // Mahony proportional gain (tilt from accel)
static const double KI = 0.005;       // Mahony integral gain (gyro bias tracking)
static const double ACC_GATE = 0.1;   // trust accel only when ||a|-g| < ACC_GATE*g
static const double MAX_DT = 0.1;     // s; larger gaps are dropouts
#ifndef VEL_TAU
#define VEL_TAU 1.0                   // s; velocity decays toward 0 (bounds double-integration drift)
#endif
#ifndef ACC_TAU
#define ACC_TAU 1.0                   // s; running mean of world accel (removes tilt/bias DC leak)
#endif
#ifndef POS_TAU
#define POS_TAU 3.0                   // s; position leaks toward origin (room-scale motion is bounded)
#endif
static const double ZUPT_W = 0.05;    // rad/s; below this and |a|~g => treat as stationary

struct State {
    double q[4] = {1, 0, 0, 0};
    double p[3] = {0, 0, 0}, v[3] = {0, 0, 0};
    double bg[3] = {0, 0, 0};
    double abar[3] = {0, 0, 0};     // running mean of gravity-free world accel
    double g = 9.81;
    double w_prev[3] = {0, 0, 0};   // last bias-corrected rate
    unsigned long long ts_prev = 0;
};

// Static init from the buffered window: gyro bias = mean rate, roll/pitch from mean accel.
static void init_state(State& st, const Sample* buf, int n) {
    double wm[3] = {0, 0, 0}, am[3] = {0, 0, 0};
    int k = 0;
    for (int i = 0; i < n; ++i) {
        if (!buf[i].valid) continue;
        for (int j = 0; j < 3; ++j) { wm[j] += buf[i].w[j]; am[j] += buf[i].a[j]; }
        ++k;
    }
    if (k == 0) return;
    for (int j = 0; j < 3; ++j) { wm[j] /= k; am[j] /= k; }
    // ponytail: assumes the start is near-static; a moving start biases bg (Mahony KI corrects slowly)
    if (norm3(wm) < 0.1) memcpy(st.bg, wm, sizeof wm);
    st.g = norm3(am);
    // q such that R(q) * a_hat = +z
    double u[3] = {am[0] / st.g, am[1] / st.g, am[2] / st.g}, ez[3] = {0, 0, 1}, ax[3];
    cross(u, ez, ax);
    double s = norm3(ax), c = u[2];
    if (s < 1e-9) {
        if (c < 0) { st.q[0] = 0; st.q[1] = 1; st.q[2] = 0; st.q[3] = 0; }
        return;
    }
    double ang = atan2(s, c), h = sin(0.5 * ang) / s;
    st.q[0] = cos(0.5 * ang); st.q[1] = ax[0] * h; st.q[2] = ax[1] * h; st.q[3] = ax[2] * h;
}

static void step(State& st, const Sample& s) {
    double dt = st.ts_prev ? (s.ts - st.ts_prev) * 1e-9 : 0.0;
    st.ts_prev = s.ts;
    double w[3];
    bool gap = dt > MAX_DT;
    if (s.valid && !gap) {
        for (int j = 0; j < 3; ++j) w[j] = s.w[j] - st.bg[j];
        // Mahony: pull estimated up-axis toward measured specific force
        double an = norm3(s.a);
        if (fabs(an - st.g) < ACC_GATE * st.g) {
            double ez[3] = {0, 0, 1}, up_b[3], e[3], qc[4] = {st.q[0], -st.q[1], -st.q[2], -st.q[3]};
            double ah[3] = {s.a[0] / an, s.a[1] / an, s.a[2] / an};
            rotate(qc, ez, up_b);
            cross(ah, up_b, e);
            for (int j = 0; j < 3; ++j) { w[j] += KP * e[j]; st.bg[j] -= KI * e[j] * dt; }
        }
    } else {
        memcpy(w, st.w_prev, sizeof w);  // dropout / invalid row: hold last rate
    }
    double th[3];
    for (int j = 0; j < 3; ++j) th[j] = 0.5 * (w[j] + st.w_prev[j]) * dt;  // RK2 midpoint
    qintegrate(st.q, th);
    memcpy(st.w_prev, w, sizeof w);

    // Position: world accel minus gravity, damped velocity. Gaps/invalid rows: constant velocity.
    if (s.valid && !gap) {
        double aw[3];
        rotate(st.q, s.a, aw);
        aw[2] -= st.g;
        double ka = 1 - exp(-dt / ACC_TAU);
        for (int j = 0; j < 3; ++j) {
            st.abar[j] += (aw[j] - st.abar[j]) * ka;
            st.v[j] += (aw[j] - st.abar[j]) * dt;   // high-passed accel
        }
        bool still = norm3(w) < ZUPT_W && fabs(norm3(s.a) - st.g) < 0.02 * st.g;
        // ponytail: exponential velocity damping instead of a full EKF; upgrade path = error-state EKF
        double k = exp(-dt / (still ? 0.05 : VEL_TAU));
        for (int j = 0; j < 3; ++j) st.v[j] *= k;
    }
    double kp = exp(-dt / POS_TAU);
    for (int j = 0; j < 3; ++j) st.p[j] = st.p[j] * kp + st.v[j] * dt;
}

// TUM line: "sec.nanosec tx ty tz qx qy qz qw" (q stored internally as w,x,y,z).
static void write_pose(FILE* f, unsigned long long ts, const double p[3], const double q[4]) {
    fprintf(f, "%llu.%09llu %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n",
            ts / 1000000000ULL, ts % 1000000000ULL,
            p[0], p[1], p[2], q[1], q[2], q[3], q[0]);
}

int main(int argc, char** argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s <imu_csv> <out_txt>\n", argv[0]);
        return 2;
    }
    FILE* in = fopen(argv[1], "rb");
    if (!in) { perror(argv[1]); return 1; }
    FILE* out = fopen(argv[2], "wb");
    if (!out) { perror(argv[2]); return 1; }

    fputs("# timestamp tx ty tz qx qy qz qw\n", out);
    char line[512];
    Sample s;
    State st;
    // Buffer the init window (fixed size), then replay it through the filter.
    static Sample buf[1024];
    int n = 0;
    bool ready = false;
    while (fgets(line, sizeof line, in)) {
        if (!parse_row(line, s)) continue;
        if (!ready) {
            buf[n++] = s;
            if (n < 1024 && (s.ts - buf[0].ts) * 1e-9 < INIT_SEC) continue;
            init_state(st, buf, n);
            for (int i = 0; i < n; ++i) { step(st, buf[i]); write_pose(out, buf[i].ts, st.p, st.q); }
            ready = true;
            continue;
        }
        step(st, s);
        write_pose(out, s.ts, st.p, st.q);
    }
    if (!ready) {  // sequence shorter than the init window
        init_state(st, buf, n);
        for (int i = 0; i < n; ++i) { step(st, buf[i]); write_pose(out, buf[i].ts, st.p, st.q); }
    }
    fclose(in);
    return fclose(out) == 0 ? 0 : 1;
}
