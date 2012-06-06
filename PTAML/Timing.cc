#include "Timing.h"

#include <iostream>

using namespace std;
using namespace std::chrono;

static time_point<high_resolution_clock> g_TicTime;

namespace PTAMM {

void Tic()
{
  g_TicTime = high_resolution_clock::now();
}

void Toc()
{
  duration<double> elapsed = high_resolution_clock::now() - g_TicTime;
  cout << "Time elapsed: " << elapsed.count() * 1000.0 << " ms" << endl;
}


TimeoutTimer::TimeoutTimer()
{
}

TimeoutTimer::TimeoutTimer(double seconds)
{
  mdDuration = duration_cast<Clock::duration>(RealSeconds(seconds));
  mtpTimeout = Clock::now() + mdDuration;
}

bool TimeoutTimer::HasTimedOut() const
{
  return Clock::now() >= mtpTimeout;
}

void TimeoutTimer::Reset()
{
  mtpTimeout = Clock::now() + mdDuration;
}


TimingTimer::TimingTimer()
  : mnNumMeas(0)
{
}

void TimingTimer::Start()
{
  mtpStart = Clock::now();
}

void TimingTimer::Stop()
{
  mdElapsed = Clock::now() - mtpStart;
  mdAccum += mdElapsed;
  ++mnNumMeas;
  if (mnNumMeas == 10) {
    mdAverageMs = mdAccum.count() * 1000.0 / 10.0;
    mdAccum = Duration(0);
    mnNumMeas = 0;
  }
}

double TimingTimer::Milliseconds() const
{
  return mdAverageMs;
  //return mdElapsed.count() * 1000.0;
}

}
