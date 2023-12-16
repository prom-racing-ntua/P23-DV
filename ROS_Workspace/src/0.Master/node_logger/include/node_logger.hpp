#include <string>
#include <iostream>
// #include <fstream>
#include <cstdio>
#include <filesystem>

class Logger
{
private:
    FILE *file;
    int run_idx;
    std::string name;
public:
    Logger();
    void init(std::string name);
    ~Logger();
    std::string check()const;
    void log(double timestamp, int type, int index);
};