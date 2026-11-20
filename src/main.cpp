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
    double p[3] = {0, 0, 0}, q[4] = {1, 0, 0, 0};
    while (fgets(line, sizeof line, in)) {
        if (!parse_row(line, s)) continue;
        write_pose(out, s.ts, p, q);
    }
    fclose(in);
    return fclose(out) == 0 ? 0 : 1;
}
