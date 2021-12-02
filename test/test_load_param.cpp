//
// Created by znfs on 2021/11/28.
//

#include "load_param.h"
#include <iostream>

using namespace helti;
using namespace std;


int main(){
    auto test_param = LoadParam::HeltiParam();
    cout << test_param.os_lidar_->transform_.matrix() << endl;
    cout << test_param.imu_->transform_.matrix() << endl;
}