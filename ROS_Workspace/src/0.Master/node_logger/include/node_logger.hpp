#include <string>
#include <iostream>
// #include <fstream>
#include <cstdio>
#include <cerrno>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

class Logger
{
protected:
    FILE *file;
    int run_idx;
    std::string name;
    std::string base;
    std::string share_dir_;
    bool underscore_format;
public:
    Logger();
    void init(std::string name);
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