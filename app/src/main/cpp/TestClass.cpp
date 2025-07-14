//
// Created by 63559 on 2025/7/11.
//
#include "TestClass.h"
TestClass::TestClass(Eigen::Vector3f val) {
    value = val;
}

void TestClass::SetValue(Eigen::Vector3f val) {
    value = val;
}

Eigen::Vector3f TestClass::GetValue() const {
    return value;
}


