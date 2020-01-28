#include <jni.h>

#include <Eigen/Core>

#include <iostream>

#include <unsupported/Eigen/MatrixFunctions>

#include "drake/math/discrete_algebraic_riccati_equation.h"
#include "edu_wpi_first_wpilibj_math_DrakeJNI.h"
#include "wpi/jni_util.h"

#include "frc/StateSpaceUtil.h"

using namespace wpi::java;

extern "C" {

/*
 * Class:     edu_wpi_first_wpilibj_math_DrakeJNI
 * Method:    exp
 * Signature: ([DI[D)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_wpilibj_math_StateSpaceUtilsJNI_exp
  (JNIEnv* env, jclass, jdoubleArray src, jint rows, jdoubleArray dst)
{
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                   Eigen::RowMajor>> Amat{env->GetDoubleArrayElements(src, nullptr),
                                   rows, rows};
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Aexp = Amat.exp();
  env->SetDoubleArrayRegion(dst, 0, rows * rows, Aexp.data());

//  jboolean isCopy;
//
//  jdouble* Aelms = env->GetDoubleArrayElements(src, &isCopy);
//
//  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
//                                  Eigen::RowMajor>> Amat{Aelms,
//                                  rows, rows};
//
//  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Aexp = Amat.exp();
//  env->SetDoubleArrayRegion(dst, 0, rows * rows, Aexp.data());
//
//  if (isCopy == JNI_TRUE) {
//      env -> ReleaseDoubleArrayElements(src, Aelms, JNI_ABORT);
//  }
}

JNIEXPORT jboolean JNICALL
Java_edu_wpi_first_wpilibj_math_StateSpaceUtilsJNI_isStabilizable
 (JNIEnv* env, jclass, jint states, jint inputs, jdoubleArray aSrc, jdoubleArray bSrc) {

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                Eigen::RowMajor>> Amat{env->GetDoubleArrayElements(aSrc, nullptr),
                                states, states};

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                Eigen::RowMajor>> Bmat{env->GetDoubleArrayElements(bSrc, nullptr),
                                states, inputs};

    return true;// frc::IsStabilizable<>(Amat, Bmat);
 }

}  // extern "C"