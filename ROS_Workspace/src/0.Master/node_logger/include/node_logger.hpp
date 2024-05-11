#include <string>
#include <iostream>
#include <set>
// #include <fstream>
#include <cstdio>
#include <cerrno>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

class enabled_logs
{
private:
    std::set<std::string> logs;
public:
    enabled_logs();
    bool check(const std::string &name);
};

class Logger
{
protected:
    FILE *file;
    int run_idx;
    std::string name;
    std::string base;
    std::string share_dir_;
    bool enabled;
    bool underscore_format;
    enabled_logs ENABLED_LOGS;
public:
    Logger();
    void init(const std::string &name);
    ~Logger();
    std::string check()const;
    void log(double timestamp, int type, int index);
};

class old_Logger: public Logger
{
public:
    old_Logger();
    void log(const std::string &data);
};