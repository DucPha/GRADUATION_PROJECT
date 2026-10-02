#include "camera_lane.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

using namespace std::chrono;

int main(int argc, char** argv) {
    const int seconds = (argc > 1) ? std::atoi(argv[1]) : 20;
    const int target_fps = (argc > 2) ? std::atoi(argv[2]) : 30;

    CameraLane cam(-1, target_fps, true, true, -4);
    if (!cam.start()) {
        std::printf("FAIL start: %s\n", cam.last_error().c_str());
        return 1;
    }

    std::printf("opened %dx%d, startup measured %.2f FPS\n",
                cam.actual_width(), cam.actual_height(), cam.measured_fps());

    const auto t0 = steady_clock::now();
    uint64_t last_id = 0;
    int got = 0, valid = 0;
    float proc_ms_sum = 0.0f;
    std::string last_cmd = "?";

    while (duration<double>(steady_clock::now() - t0).count() < seconds) {
        LaneOutput out;
        if (!cam.get_latest(out, 0.0f)) continue;
        if (out.frame_id == last_id) {
            std::this_thread::sleep_for(microseconds(500));
            continue;
        }
        last_id = out.frame_id;
        ++got;
        if (out.valid) ++valid;
        proc_ms_sum += out.processing_ms;
        last_cmd = out.camera_cmd;
        std::this_thread::sleep_for(microseconds(500));
    }

    const double el = duration<double>(steady_clock::now() - t0).count();
    std::printf("--- %ds run ---\n", seconds);
    std::printf("end-to-end FPS      : %.2f\n", got / el);
    std::printf("lane-valid ratio    : %.1f%%\n", 100.0 * valid / std::max(1, got));
    std::printf("avg process ms      : %.2f\n", proc_ms_sum / std::max(1, got));
    std::printf("last cmd            : %s\n", last_cmd.c_str());

    cam.stop();
    return 0;
}
