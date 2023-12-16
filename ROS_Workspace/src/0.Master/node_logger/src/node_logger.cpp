#include "node_logger.hpp"

Logger::Logger()
{
    this->name = "";
    this->file = nullptr;
    this->run_idx = -1;
}
void Logger::init(std::string name)
{
    this->name = name;
    std::filesystem::directory_iterator dirIter;
    try
    {
        dirIter = std::filesystem::directory_iterator("timestamp_logs");
    }
    catch(const std::exception& e)
    {
        file = nullptr;
        return;
    }
    
    this->run_idx = -1;

    for(auto& entry: dirIter) ++run_idx;
    
    char f1[30 + name.length()];
    snprintf(f1, sizeof(f1), "timestamp_logs/run_%d/%s_log.txt", this->run_idx, name.c_str());
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
    if(file==nullptr)
    {
        return "Couldn't open logger " + name;
    }
    else
    {
        return "File " + name + " opened successfully";
    }
}
void Logger::log(double timestamp, int type, int index)
{
    if(file == nullptr)return;
    
    try
    {
        fprintf(file, "%f\t%d\t%d\n", timestamp, type, index);
    }
    catch(const std::exception& e)
    {
        std::cout << "Error during writing:" << e.what() << '\n';
        std::cout << "Aborting writing. Plz fix!"<<std::endl;
        fprintf(file, "Error during writing: %s\n Aborting Writing. Plz Fiz!!!\n");
        file = nullptr;
    }
}