# Build + run the estimator. Usage:
#   docker build -t pathfinder .
#   docker run --rm -v "$PWD/data:/data" pathfinder /data/imu.csv /data/estimated_trajectory.txt
FROM gcc:14.2.0 AS build
WORKDIR /app
COPY src/main.cpp .
RUN g++ -O2 -std=c++17 -static -o pathfinder main.cpp

FROM debian:bookworm-slim
COPY --from=build /app/pathfinder /usr/local/bin/pathfinder
ENTRYPOINT ["/usr/local/bin/pathfinder"]
