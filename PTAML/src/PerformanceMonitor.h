#ifndef __PERFORMANCE_MONITOR_H_
#define __PERFORMANCE_MONITOR_H_

#include "Timing.h"
#include <vector>
#include <map>
#include <string>

namespace PTAMM {

class PerformanceMonitor {
  public:
    void AddTimer(const std::string& sName);
    void StartTimer(const std::string& sName);
    void StopTimer(const std::string& sName);
    double GetTimerInSeconds(const std::string& sName) const;
    void QueryAllTimers(std::vector<std::pair<std::string, double>> &vOutput);

    void AddRateCounter(const std::string& sName);
    void UpdateRateCounter(const std::string& sName);
    double GetRateCounterInHz(const std::string& sName) const;
    void QueryAllRates(std::vector<std::pair<std::string, double>> &vOutput);

  private:
    std::map<std::string, TimingTimer> mvTimers;
    std::map<std::string, RateCounter> mvRateCounters;
};

}

#endif
