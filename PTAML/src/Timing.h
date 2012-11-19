#ifndef __TIMER_H
#define __TIMER_H

#include <chrono>

namespace PTAMM {

class StopWatch {
  typedef std::chrono::high_resolution_clock Clock;
  typedef std::chrono::time_point<Clock> TimePoint;
  typedef std::chrono::duration<double> RealSeconds;

  public:
    StopWatch();

    void Start();
    void Stop();

    void Reset();

    double StoppedTime() const;
    double Elapsed() const;
    Clock::duration ElapsedDuration() const;

    bool Running() const { return mbRunning; }

  private:
    bool mbRunning;
    TimePoint mtpStart;
    TimePoint mtpStop;
    Clock::duration mdDuration;
};

class TimeoutTimer {
  typedef std::chrono::high_resolution_clock Clock;
  typedef std::chrono::time_point<Clock> TimePoint;
  typedef std::chrono::duration<double> RealSeconds;

  public:
    TimeoutTimer();
    TimeoutTimer(double seconds);

    bool HasTimedOut() const;
    void Reset();

  private:
    TimePoint mtpTimeout;
    Clock::duration mdDuration;
};


class TimingTimer {
  typedef std::chrono::high_resolution_clock Clock;
  typedef std::chrono::time_point<Clock> TimePoint;
  typedef std::chrono::duration<double> Duration;

  public:
    TimingTimer();

    void Start();
    void Stop();

    double Seconds() const;

  private:
    TimePoint mtpStart;
    Duration mdElapsed;
    size_t mnNumMeas;
    Duration mdAccum;
    double mdAverageMs;
};

class RateCounter {
  public:
    RateCounter()
      : mLastFpsUpdate(std::chrono::high_resolution_clock::now())
      , mNumFrames(0)
      , mFps(0)
    {
    }

    double GetRate() const { return mFps; }

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

class RateLimiter {
  public:
    RateLimiter();
    void Limit(double rate);
  private:
    typedef std::chrono::high_resolution_clock Clock;
    std::chrono::time_point<Clock> mtp;
};

void Tic();
void Toc();

}

#endif
