#ifndef FPSHANDLER_H
#define FPSHANDLER_H

#include <chrono>
#include <thread>

using namespace std::chrono;

class FpsHandler {
public:
    static FpsHandler& instance() {
        static FpsHandler instance;
        return instance;
    }
    FpsHandler(const FpsHandler&) = delete;
    FpsHandler &operator=(const FpsHandler&) = delete;

    // Values
    int TARGET_FPS = 500;
    double TARGET_FTIME = 1000 / TARGET_FPS;
    double MAX_FTIME = 1000 / 5;

    // Frame handler
    double dTime = 0;
    double dTimeSec = 0;
    high_resolution_clock::time_point prevFTime = high_resolution_clock::now();

    // Other
    int fps = 0;
    int fpsAccumulate = 0;
    int fCount = 0;

    double startFrame();
    void endFrame();

private:
    FpsHandler() {}
};

#endif