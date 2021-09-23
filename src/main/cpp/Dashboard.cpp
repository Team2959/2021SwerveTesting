#include <functional>
#include <vector>
#include <string>
#include <utility>
#include <thread>
#include <unordered_map>
#include <frc/smartdashboard/SmartDashboard.h>

#include <Dashboard.h>

void WatcherContext::Set(bool val)
{
    frc::SmartDashboard::PutBoolean(m_name, val);
}
void WatcherContext::Set(wpi::StringRef str)
{
    frc::SmartDashboard::PutString(m_name, str);
}
void WatcherContext::Set(double number)
{
    frc::SmartDashboard::PutNumber(m_name, number);
}
WatcherContext::WatcherContext(const std::string& name)
{
    m_name = name;
};



void Dashboard::AddWatcher(wpi::StringRef name, WatcherFunction func, Interval interval)
{
    switch(interval)
    {
    case Interval::Fast:
        m_fastWatchers.insert({name, func});
        break;
    case Interval::Slow:
        m_slowWatchers.insert({name, func});
        break;
    case Interval::ReallySlow:
        m_reallySlowWatchers.insert({name, func});
        break;
    default:
        m_reallySlowWatchers.insert({name, func});
    }
}
template<typename T>
void Dashboard::AddWatcher(wpi::StringRef name, T& ref, Interval interval)
{
    AddWatcher(name, [ref](const WatcherContext& ctx){
        ctx.Set(ref);
    }, interval);
}
void Dashboard::RunFast()
{
    while(true)
    {
        auto timestamp = std::chrono::high_resolution_clock::now();
        timestamp += std::chrono::milliseconds{Interval::Fast};
        for(auto watcher : m_fastWatchers)
        {
            watcher.second(WatcherContext(watcher.first));
        }
        std::this_thread::sleep_until(timestamp);
    }
}
void Dashboard::RunSlow()
{
    while(true)
    {
        auto timestamp = std::chrono::high_resolution_clock::now();
        timestamp += std::chrono::milliseconds{Interval::Slow};
        for(auto watcher : m_slowWatchers)
        {
            watcher.second(WatcherContext(watcher.first));
        }
        std::this_thread::sleep_until(timestamp);
    }
}
void Dashboard::RunReallySlow()
{
    while(true)
    {
        auto timestamp = std::chrono::high_resolution_clock::now();
        timestamp += std::chrono::milliseconds{Interval::ReallySlow};
        for(auto watcher : m_reallySlowWatchers)
        {
            watcher.second(WatcherContext(watcher.first));
        }
        std::this_thread::sleep_until(timestamp);
    }
}

std::unordered_map<std::string, Dashboard::WatcherFunction> Dashboard::m_fastWatchers;
std::unordered_map<std::string, Dashboard::WatcherFunction> Dashboard::m_slowWatchers;
std::unordered_map<std::string, Dashboard::WatcherFunction> Dashboard::m_reallySlowWatchers;

