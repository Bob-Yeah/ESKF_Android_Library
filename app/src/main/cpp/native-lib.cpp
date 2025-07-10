#include <jni.h>
#include <string>
#include "ESKF.h"

//#define EXPORT_API __declspec(dllexport)
#define EXPORT_API __attribute__((visibility("default")))
using namespace  Eigen;

extern "C" JNIEXPORT jstring JNICALL
Java_com_uimh_eskf_1android_1library_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

extern "C"
{
    EXPORT_API float getTrace(){
        MatrixXf m = MatrixXf::Random(4,4);
        return m.trace();
    }

    EXPORT_API ESKF* CreateESKFClass()
    {
        return new ESKF();
    }

    EXPORT_API float GetPosX(ESKF *eskf)
    {
        return eskf->getPos().x();
    }


}
