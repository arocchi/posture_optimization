#ifndef __JNI_MBSYSPAD_H__
#define __JNI_MBSYSPAD_H__

#include <jni.h>

class JNI_MBSysPad {
    /* the JNI environment we need to load*/
    JNIEnv* env;

    jclass animFrameClass;
    jmethodID mainMethod;
    jmethodID updateJointMethod;
    jobject obj;

JNIEnv* create_vm();
public:
    JNI_MBSysPad();
    void plot(const int njoin, const double* q);
};

#endif
