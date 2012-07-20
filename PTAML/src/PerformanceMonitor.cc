#include "PerformanceMonitor.h"

#include <cassert>
#include <stdexcept>

namespace PTAMM {

void PerformanceMonitor::AddTimer(const std::string& sName)
{
  if (mvTimers.find(sName) != mvTimers.end()) {
    throw std::runtime_error("Timer already exists");
  }
  mvTimers[sName] = TimingTimer();
}

void PerformanceMonitor::StartTimer(const std::string& sName)
{
  auto it = mvTimers.find(sName);
  if (it == mvTimers.end()) {
    throw std::runtime_error("Invalid timer");
  }
  it->second.Start();
}

void PerformanceMonitor::StopTimer(const std::string& sName)
{
  auto it = mvTimers.find(sName);
  if (it == mvTimers.end()) {
    throw std::runtime_error("Invalid timer");
  }
  it->second.Stop();
}

double PerformanceMonitor::GetTimerInSeconds(const std::string& sName) const
{
  auto it = mvTimers.find(sName);
  if (it == mvTimers.end()) {
    throw std::runtime_error("Invalid timer");
  }
  return it->second.Seconds();
}

void PerformanceMonitor::QueryAllTimers(std::vector<std::pair<std::string, double>> &vOutput)
{
  vOutput.clear();
  for (auto it = mvTimers.begin(); it != mvTimers.end(); ++it) {
    vOutput.emplace_back(it->first, it->second.Seconds());
  }
}

void PerformanceMonitor::AddRateCounter(const std::string& sName)
{
  if (mvRateCounters.find(sName) != mvRateCounters.end()) {
    throw std::runtime_error("Rate counter already exists");
  }
  mvRateCounters[sName] = RateCounter();
}

void PerformanceMonitor::UpdateRateCounter(const std::string& sName)
{
  auto it = mvRateCounters.find(sName);
  if (it == mvRateCounters.end()) {
    throw std::runtime_error("Invalid rate counter");
  }
  it->second.Update();
}

double PerformanceMonitor::GetRateCounterInHz(const std::string& sName) const
{
  auto it = mvRateCounters.find(sName);
  if (it == mvRateCounters.end()) {
    throw std::runtime_error("Invalid rate counter");
  }
  return it->second.GetRate();
}

void PerformanceMonitor::QueryAllRates(std::vector<std::pair<std::string, double>> &vOutput)
{
  vOutput.clear();
  for (auto it = mvRateCounters.begin(); it != mvRateCounters.end(); ++it) {
    vOutput.emplace_back(it->first, it->second.GetRate());
  }
}

}
