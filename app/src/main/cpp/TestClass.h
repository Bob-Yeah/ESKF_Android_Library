//
// Created by 63559 on 2025/7/11.
//

#ifndef ESKF_ANDROID_LIBRARY_TESTCLASS_H
#define ESKF_ANDROID_LIBRARY_TESTCLASS_H
#include "eigen/Core"
#include <iostream>
#include "eigen/Geometry"

class TestClass {
public:
    TestClass(Eigen::Vector3f val);
    ~TestClass() {};
    void SetValue(Eigen::Vector3f val);
    Eigen::Vector3f GetValue() const;
private:
    Eigen::Vector3f value;
};
#endif //ESKF_ANDROID_LIBRARY_TESTCLASS_H
