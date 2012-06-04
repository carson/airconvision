#ifndef __TIMER_H
#define __TIMER_H

#include <chrono>

namespace PTAMM {

class TimingTimer {
  typedef std::chrono::high_resolution_clock Clock;
  typedef std::chrono::time_point<Clock> TimePoint;
  typedef std::chrono::duration<double> Duration;

  public:
    TimingTimer();

    void Start();
    void Stop();

    double Milliseconds() const;

  private:
    TimePoint mtpStart;
    Duration mdElapsed;
    size_t mnNumMeas;
    Duration mdAccum;
    double mdAverageMs;
};

void Tic();
void Toc();

extern TimingTimer gFrameTimer;
extern TimingTimer gVideoSourceTimer;
extern TimingTimer gFeatureTimer;
extern TimingTimer gTrackTimer;
extern TimingTimer gSBIInitTimer;
extern TimingTimer gSBITimer;
extern TimingTimer gTrackingQualityTimer;
extern TimingTimer gDrawGridTimer;
extern TimingTimer gTrackFullTimer;
extern TimingTimer gPvsTimer;

}

#endif
