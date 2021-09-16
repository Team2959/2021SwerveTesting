#pragma once

#include <functional>
#include <vector>
#include <string>
#include <utility>
#include <thread>
#include <unordered_map>
#include <frc/smartdashboard/SmartDashboard.h>

class WatcherContext
{
public:
    void Set(bool val)
    {
        frc::SmartDashboard::PutBoolean(m_name, val);
    }
    void Set(wpi::StringRef str)
    {
        frc::SmartDashboard::PutString(m_name, str);
    }
    void Set(double number)
    {
        frc::SmartDashboard::PutNumber(m_name, number);
    }
private:
    WatcherContext(const std::string& name);
    std::string m_name;
    friend class Dashboard;
};

class Dashboard 
{
public:
    using WatcherFunction = std::function<void(const WatcherContext&)>;
private:
    struct Watcher
    {
        WatcherFunction func;
        std::string name;
        int interval;
    };
public:
    enum Interval
    {
        Fast = 20,
        Slow = 60,
        ReallySlow = 500,
    };
    static void AddWatcher(wpi::StringRef name, WatcherFunction func, Interval interval = Interval::Slow)
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
            m_reallySlowWatcher.insert({name, func});
            break;
        default:
            m_reallySlowWatcher.insert({name, func});
        }
    }
    template<typename T>
    static void AddWatcher(wpi::StringRef name, T& ref, Interval interval = Interval::Slow)
    {
        AddWatcher(name, [](const WatcherContext& ctx){
            ctx.Set(ref);
        }, interval);
    }
    static void RunFast()
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
    static void RunSlow()
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
    static void RunReallySlow()
    {
        while(true)
        {
            auto timestamp = std::chrono::high_resolution_clock::now();
            timestamp += std::chrono::milliseconds{Interval::ReallySlow};
            for(auto watcher : m_reallySlowWatcher)
            {
                watcher.second(WatcherContext(watcher.first));
            }
            std::this_thread::sleep_until(timestamp);
        }
    }

private:
    static std::unordered_map<std::string, WatcherFunction> m_fastWatchers;
    static std::unordered_map<std::string, WatcherFunction> m_slowWatchers;
    static std::unordered_map<std::string, WatcherFunction> m_reallySlowWatcher;
};

