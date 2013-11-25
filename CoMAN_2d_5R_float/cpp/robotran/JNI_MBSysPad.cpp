#include <jni.h>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include "JNI_MBSysPad.h"

namespace fs = boost::filesystem;

JNIEnv* JNI_MBSysPad::create_vm() {
	JavaVM* jvm;
	JNIEnv* env;
	JavaVMInitArgs args;
	JavaVMOption options[2];
    std::stringstream classPath;
    std::stringstream libraryPath;
    fs::path classPathBoost;

    fs::path full_path( fs::current_path() );
    classPathBoost = full_path / "mbsyspad" / "MBsysPad.jar";
    classPath << "-Djava.class.path=" << classPathBoost.c_str();
    libraryPath << "-Djava.library.path=" << JAVA_LIBRARY_PATH;


	// the following parameter should be adapted depending on the version of the JDK
	args.version = JNI_VERSION_1_6;
    // number of argument to pass to the jvm
    args.nOptions = 4;
    options[0].optionString = "-Djava.compile=NONE";
	// path to the jar file containing mbsyspad bynary code
    options[1].optionString = classPath.str().c_str();
	// path to the folder containing additional libraries, in particular the java3D native file (libj3dcore-ogl.so for linux)
    options[2].optionString = libraryPath.str().c_str();
    options[3].optionString = "-verbose:jni";
	args.options = options;
    args.ignoreUnrecognized = JNI_TRUE;
    std::cout << "Running jvm with class.path set to " << classPathBoost.c_str() << std::endl;
    int res = JNI_CreateJavaVM(&jvm, (void **)&env, &args);
    if(res != 0) {
        std::cerr << "Error while creating JavaVM" << std::endl;
        exit(1);
    }
	return env;
}


JNI_MBSysPad::JNI_MBSysPad() {
    // initialise jni variable for 3D visualisation
    jclass animFrameClass = NULL;
    jmethodID constructor = NULL;
    jmethodID loadMethod = NULL;
    jstring mbsFilename = NULL;
    jthrowable exception = NULL;
    fs::path mbsFilenameBoost;
    this->updateJointMethod = NULL;
    this->obj = NULL;
    this->env = NULL;

    fs::path full_path( fs::current_path() );
    mbsFilenameBoost = full_path/ "robot" / "robot.mbs";

    std::cout << "Creating JVM" << std::endl;
    this->env = this->create_vm();
    if(this->env == NULL) {
        std::cerr << "Error while creating JNI environment" << std::endl;
        exit(1);
    }

    // get the reference to the class which define the window for the 3D view
    animFrameClass = this->env->FindClass("be/robotran/test/LiveSimAnimFrame");
    exception = this->env->ExceptionOccurred();
    if(exception != NULL && animFrameClass != NULL) {
        std::cerr << "Exception while getting LiveSimAnimFrame" << std::endl;
        exit(1);
    }

    // get the reference to the constructor of this class
    constructor = this->env->GetMethodID(animFrameClass, "<init>", "()V");
    exception = this->env->ExceptionOccurred();
    if(exception != NULL && constructor != NULL) {
        std::cerr << "Exception while getting MethodId for constructor" << std::endl;
        exit(1);
    }

    // get the reference to the load method which load the *.mbs file
    loadMethod = this->env->GetMethodID(animFrameClass, "load", "(Ljava/lang/String;)V");
    exception = this->env->ExceptionOccurred();
    if(exception != NULL && loadMethod != NULL) {
        std::cerr << "Exception while getting MethodId load" << std::endl;
        exit(1);
    }

    // get the reference to the updateJoints method which update the 3D view during simulation
    this->updateJointMethod = this->env->GetMethodID(animFrameClass, "updateJoints", "([D)V");
    exception = this->env->ExceptionOccurred();
    if(exception != NULL && this->updateJointMethod != NULL) {
        std::cerr << "Exception while getting MethodId updateJoints" << std::endl;
        exit(1);
    }

    // create a jni string containing the path to the *.mbs file to load
    std::cout << "Opening" << mbsFilenameBoost.c_str() << std::endl;
    mbsFilename = this->env->NewStringUTF(mbsFilenameBoost.c_str());
    exception = this->env->ExceptionOccurred();
    if(exception != NULL && mbsFilename != NULL) {
        std::cerr << "Exception while creating new string mbsFileName" << std::endl;
        exit(1);
    }

    // create an instance of the 3D view
    this->obj = this->env->NewObject(animFrameClass, constructor);
    exception = this->env->ExceptionOccurred();
    if(exception != NULL && this->obj != NULL) {
        std::cerr << "Exception while creating object of type animFrameClass" << std::endl;
        exit(1);
    }
    std::cout << "created obj" << std::endl;
    sleep(5);
    // load the mbsfilename
    (this->env)->CallObjectMethod(obj, loadMethod, mbsFilename);
    exception = this->env->ExceptionOccurred();
    if(exception != NULL) {
        std::cerr << "Exception while loading mbs file" << std::endl;
        exit(1);
    }
    sleep(5);
    std::cout << "loaded file" << std::endl;

    jdoubleArray doubleArrayArg;
    jint n = (jint)8;
    double q[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    // instantiate a jni array of double
    doubleArrayArg = this->env->NewDoubleArray(n);
    exception = this->env->ExceptionOccurred();
    if(exception != NULL) {
        std::cerr << "Exception while creating array" << std::endl;
        exit(1);
    }

    // copy the current values of MBSdata->q to the jni array
    this->env->SetDoubleArrayRegion(doubleArrayArg, 0 , n, q);
    // update the 3D view
    this->env->CallObjectMethod(this->obj,
                                  this->updateJointMethod,
                                  doubleArrayArg);
}

void JNI_MBSysPad::plot(const int njoint, const double* q) {
    // checking JNI interface   //

    if(this->env != NULL &&
       this->obj != NULL) {
        jthrowable exception = NULL;
        jdoubleArray doubleArrayArg;
        jint n = (jint)njoint;

        // instantiate a jni array of double
        doubleArrayArg = this->env->NewDoubleArray(n);
        exception = this->env->ExceptionOccurred();
        if(exception != NULL) {
            std::cerr << "Exception while creating array" << std::endl;
            exit(1);
        }

        // copy the current values of MBSdata->q to the jni array
        this->env->SetDoubleArrayRegion(doubleArrayArg, 0 , n, q);
        // update the 3D view
        this->env->CallObjectMethod(this->obj,
                                      this->updateJointMethod,
                                      doubleArrayArg);
    }
}
