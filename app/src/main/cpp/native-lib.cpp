#include <jni.h>
#include <string>
#include "ESKF.h"
#include <android/log.h>

//#define EXPORT_API __declspec(dllexport)
#define EXPORT_API __attribute__((visibility("default")))
using namespace  Eigen;

#pragma pack(push,4)
struct InitData{
    float gravity;
    float initPosX;
    float initPosY;
    float initPosZ;
    float initVelX;
    float initVelY;
    float initVelZ;
    float initQuatX;
    float initQuatY;
    float initQuatZ;
    float initQuatW;
    float initAccBiasX;
    float initAccBiasY;
    float initAccBiasZ;
    float initGyroBiasX;
    float initGyroBiasY;
    float initGyroBiasZ;
    float sigma_init_pos;
    float sigma_init_vel;
    float sigma_init_dtheta;
    float sigma_init_accel_bias;
    float sigma_init_gyro_bias;
    float sigma_accel;
    float sigma_gyro;
    float sigma_accel_drift;
    float sigma_gyro_drift;
};

struct Vector3Data
{
    float x,y,z;
};

struct QuaternionData{
    float qx, qy, qz, qw;
};
#pragma pack(pop)

#define SQ(x) (x*x)
#define I_3 (Eigen::Matrix3f::Identity())

extern "C" JNIEXPORT jstring JNICALL
Java_com_uimh_eskf_1android_1library_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    __android_log_print(ANDROID_LOG_DEBUG, "ESKFLib", "Create IntPtr");

    return env->NewStringUTF(hello.c_str());
}

extern "C"
{
    // For test
    EXPORT_API float getTrace(){
        MatrixXf m = MatrixXf::Random(4,4);
        return m.trace();
    }

    EXPORT_API ESKF* CreateESKFClass(InitData* initData)
    {
        __android_log_print(ANDROID_LOG_DEBUG, "ESKFLib", "Create IntPtr");

        ESKF* eskf_obj = new ESKF(
                Vector3f(0, 0, -initData->gravity), // Acceleration due to gravity in global frame
                ESKF::makeState(
                        Vector3f(initData->initPosX, initData->initPosY, initData->initPosZ), // init pos
                        Vector3f(initData->initVelX, initData->initVelY, initData->initVelZ), // init vel
                        Quaternionf(initData->initQuatX,initData->initQuatY,initData->initQuatZ,initData->initQuatW), // init quaternion
                        Vector3f(initData->initAccBiasX,initData->initAccBiasY,initData->initAccBiasZ), // init accel bias
                        Vector3f(initData->initGyroBiasX,initData->initGyroBiasY,initData->initGyroBiasZ) // init gyro bias
                ),
                ESKF::makeP(
                        SQ(initData->sigma_init_pos) * I_3,
                        SQ(initData->sigma_init_vel) * I_3,
                        SQ(initData->sigma_init_dtheta) * I_3,
                        SQ(initData->sigma_init_accel_bias) * I_3,
                        SQ(initData->sigma_init_gyro_bias) * I_3
                ),
                SQ(initData->sigma_accel),
                SQ(initData->sigma_gyro),
                SQ(initData->sigma_accel_drift),
                SQ(initData->sigma_gyro_drift)
                );
        return eskf_obj;
    }

    EXPORT_API Vector3Data* GetPos(ESKF *eskf)
    {
        Eigen::Vector3f vec = eskf->getPos();
        std::cout << "get pos called" <<std::endl;
        __android_log_print(ANDROID_LOG_ERROR, "ESKFLib", "get pos called");
        Vector3Data* data = new Vector3Data{vec.x(),vec.y(),1.055};
        return data;
    }

    EXPORT_API QuaternionData* GetQuat(ESKF *eskf)
    {
        Eigen::Quaternionf quat = eskf->getQuat();
        QuaternionData* data = new QuaternionData{quat.x(),quat.y(),quat.z(),quat.w()};
        return data;
    }

    EXPORT_API Vector3Data* GetAccelBias(ESKF *eskf)
    {
        Eigen::Vector3f vec = eskf->getAccelBias();
        Vector3Data* data = new Vector3Data{vec.x(),vec.y(),vec.z()};
        return data;
    }

    EXPORT_API Vector3Data* GetGyroBias(ESKF *eskf)
    {
        Eigen::Vector3f vec = eskf->getGyroBias();
        Vector3Data* data = new Vector3Data{vec.x(),vec.y(),vec.z()};
        return data;
    }

    EXPORT_API void FreeVector(Vector3Data* ptr)
    {
        delete ptr;
    }

    EXPORT_API void FreeQuaternion(QuaternionData* ptr)
    {
        delete ptr;
    }

    EXPORT_API void PredictIMU(ESKF* eskf, Vector3Data a_m, Vector3Data omega_m, float dt)
    {
        Eigen::Vector3f imu_accel(a_m.x,a_m.y,a_m.z);
        Eigen::Vector3f imu_gyro(omega_m.x,omega_m.y,omega_m.z);
        std::cout << "received accel:" << imu_accel.x() << "," << imu_accel.y() << "," << imu_accel.z() <<std::endl;
        __android_log_print(ANDROID_LOG_DEBUG, "ESKFLib", "received accel:%.5f,%.5f,%.5f", imu_accel.x(),imu_accel.y(),imu_accel.z());
        eskf->predictIMU(imu_accel,imu_gyro,dt);
    }

    EXPORT_API void MeasurePos(ESKF* eskf, Vector3Data pos,float sigma_meas_pos)
    {
        Eigen::Vector3f pos_meas(pos.x,pos.y,pos.z);
        eskf->measurePos(pos_meas,SQ(sigma_meas_pos)*I_3);
    }

    EXPORT_API void MeasureQuat(ESKF* eskf, QuaternionData quat, float sigma_meas_quat)
    {
        Eigen::Quaternionf q_gb_meas(quat.qx,quat.qy,quat.qz,quat.qw);
        eskf->measureQuat(q_gb_meas,SQ(sigma_meas_quat)*I_3);
    }

    EXPORT_API Vector3Data TestDataInOut(Vector3Data in)
    {
        __android_log_print(ANDROID_LOG_DEBUG, "ESKFLib", "TestDataInOut");
        Vector3Data newData {in.x,in.y,0};
        return newData;
    }
}
