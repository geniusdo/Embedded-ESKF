#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include "config.h"
#include "ConsistentOrientationFilter.hpp"

using namespace EmbeddedMath;

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

    // ESKF initalization
    const double gyro_noise = 1e-6;
    const double gyro_random_walk = 1e-6;
    const double gravity_noise = 1e-3;
    auto consistent_filter = Filter::ConsistentOrientation::ConsistentOrientationFilter();
    consistent_filter.setImuParam(Vector3d(gyro_noise, gyro_noise, gyro_noise), 400);
    consistent_filter.setMeasurementParam(Vector3d(gravity_noise, gravity_noise, gravity_noise));
    consistent_filter.setInitialState(Quaterniond::Identity());

    int cnt = 0;

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

            Vector4d input = Vector4d(0.0025, gyro_x, gyro_y, gyro_z);

            auto consistent_prediction = consistent_filter.predict(input);

            Vector3d acce = Vector3d(acce_x, acce_y, acce_z);
            consistent_filter.update(acce.normalized());
            std::cout << "Quaternion " << consistent_prediction.w() << " " << consistent_prediction.x() << " " << consistent_prediction.y() << " " << consistent_prediction.z() << std::endl;
        }
        else
        {
            std::cerr << "Error parsing line: " << line << std::endl;
        }
        // usleep(1000);
        cnt++;
    }

    file.close();
    return 0;
}