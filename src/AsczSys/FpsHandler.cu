#include <FpsHandler.cuh>

double FpsHandler::startFrame() {
    auto curFTime = high_resolution_clock::now();
    duration<double, std::milli> elapsed = curFTime - prevFTime;
    prevFTime = curFTime;

    double fTime = elapsed.count();

    // Avoid large drop in frame rate
    if (fTime > MAX_FTIME) fTime = MAX_FTIME;

    // Calculate FPS at every second
    /*
    Why cant you calculate fps every frame?

    Its like taking a picture of a moving car and then
    telling people to calculate the speed of the car
    That just doesn't make sense
    */
    fpsAccumulate += fTime;
    fCount++;

    if (fpsAccumulate >= 1000) {
        fps = fCount;
        fCount = 0;
        fpsAccumulate = 0;
    }

    // Update dTime
    dTime = fTime;
    dTimeSec = dTime / 1000;
    return fTime;
}

void FpsHandler::endFrame() {
    double sleepTime = TARGET_FTIME - dTime;

    if (sleepTime > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds((int)sleepTime));
}