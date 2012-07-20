#ifndef __FPS_COUNTER_H
#define __FPS_COUNTER_H

#include <chrono>

namespace PTAMM {

class FPSCounter {
  public:
    FPSCounter()
    : mLastFpsUpdate(std::chrono::high_resolution_clock::now())
    , mNumFrames(0)
    , mFps(0)
    {
    }

    double Fps() const { return mFps; }

    /*
     * Called every frame to update the FPS counter.
     *
     * @Returns true if the FPS was updated
     */
    bool Update() {
      using namespace std::chrono;
      ++mNumFrames;
      auto now = high_resolution_clock::now();
      auto elapsed = now - mLastFpsUpdate;
      if (elapsed > seconds(1)) {
        typedef duration<double, std::ratio<1>> fseconds;
        double elapsedSeconds = duration_cast<fseconds>(elapsed).count();
        mFps = (double)mNumFrames / elapsedSeconds;
        mLastFpsUpdate = now;
        mNumFrames = 0;
        return true;
      }
      return false;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> mLastFpsUpdate;
    int mNumFrames;
    double mFps;
};

}

#endif
