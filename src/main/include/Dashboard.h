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
    void Set(bool val);
    void Set(wpi::StringRef str);
    void Set(double number);
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
    static void AddWatcher(wpi::StringRef name, WatcherFunction func, Interval interval = Interval::Slow);
    template<typename T>
    static void AddWatcher(wpi::StringRef name, T& ref, Interval interval = Interval::Slow);
    static void RunFast();
    static void RunSlow();
    static void RunReallySlow();

public:
    static std::unordered_map<std::string, WatcherFunction> m_fastWatchers;
    static std::unordered_map<std::string, WatcherFunction> m_slowWatchers;
    static std::unordered_map<std::string, WatcherFunction> m_reallySlowWatchers;
};

