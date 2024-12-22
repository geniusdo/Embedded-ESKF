#include <iostream>
#include <fstream>
#include <sstream>
#include "config.h"
#include "ConsistentOrientationFilter.hpp"

int main(int argc, char *argv[])
{
    std::string file_name = std::string(IMU_FILE_PATH) + "imu.txt";
    std::ifstream file(file_name);
    std::string line;

    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << file_name << std::endl;
        return false;
    }

    std::getline(file, line);

    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        double timestamp;
        double acce_x, acce_y, acce_z;
        double gyro_x, gyro_y, gyro_z;

        if (ss >> timestamp)
        {
            char comma; 
            ss >> comma >> acce_x >> comma >> acce_y >> comma >> acce_z >> comma >> gyro_x >> comma >> gyro_y >> comma >> gyro_z;

            std::cout << "Timestamp: " << timestamp << " acce_x: " << acce_x << " acce_y: " << acce_y << " acce_z: " << acce_z << " gyro_x: " << gyro_x << " gyro_y: " << gyro_y << " gyro_z: " << gyro_z << std::endl;
        }
        else
        {
            std::cerr << "Error parsing line: " << line << std::endl;
        }
    }

    file.close();
    return 0;
}