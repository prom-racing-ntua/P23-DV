#include "node_logger.hpp"

enabled_logs::enabled_logs(): logs({"slam_optim", "velocity", "perception", "mapLog", "slam_perception", "slam_odometry"}) {}

bool enabled_logs::check(const std::string &name)
{
    return logs.count(name)==1; 
}

Logger::Logger():name(""), file(nullptr), run_idx(-1), underscore_format(true), enabled(true), ENABLED_LOGS()
{
    share_dir_ = ament_index_cpp::get_package_share_directory("node_logger");
}

void Logger::init(const std::string &name)
{
    this->name = name;
    if(!ENABLED_LOGS.check(name))
    {
        enabled = false;
        return;
    }

    this->base = share_dir_+ "/../../../../timestamp_logs";
    std::filesystem::directory_iterator dirIter;
    try
    {
        dirIter = std::filesystem::directory_iterator(this->base);
    }
    catch(const std::exception& e)
    {
        file = nullptr;
        return;
    }
    this->run_idx = -1;

    for(auto& entry: dirIter) ++run_idx;

    std::string suffix = underscore_format ? "_log" : "Log";

    errno = 0;
    char f1[base.length() + name.length() + suffix.length() + 30];
    snprintf(f1, sizeof(f1), "%s/run_%d/%s%s.txt",this->base.c_str(), this->run_idx, name.c_str(), suffix.c_str());
    this->file = fopen(f1, "w");
}
Logger::~Logger()
{
    if(file!=nullptr)
    {
        fclose(file);
    }
}
std::string Logger::check()const
{
    if(!enabled)
    {
        return "Logger " + name + " is disabled.";
    }
    if(file==nullptr)
    {
        return "Couldn't open logger " + name + ".\n Reason: " + std::to_string(errno);
    }
    else
    {
        return "File " + name + " opened successfully";
    }
}
void Logger::log(double timestamp, int type, int index)
{
    if(file == nullptr || !enabled)return;
    
    try
    {
        fprintf(file, "%f\t%d\t%d\n", timestamp, type, index);
    }
    catch(const std::exception& e)
    {
        std::cout << "Error during writing:" << e.what() << '\n';
        std::cout << "Aborting writing. Plz fix!"<<std::endl;
        fprintf(file, "Error during writing: %s\n Aborting Writing. Plz Fiz!!!\n", e.what());
        file = nullptr;
    }
}

old_Logger::old_Logger(){this->base = share_dir_+ "/../../../../testingLogs"; underscore_format = false;}

void old_Logger::log(const std::string &data)
{
    if(file == nullptr)return;
    
    try
    {
        fprintf(file, "%s", data.c_str());
    }
    catch(const std::exception& e)
    {
        std::cout << "Error during writing:" << e.what() << '\n';
        std::cout << "Aborting writing. Plz fix!"<<std::endl;
        fprintf(file, "Error during writing: %s\n Aborting Writing. Plz Fiz!!!\n", e.what());
        file = nullptr;
    }
}