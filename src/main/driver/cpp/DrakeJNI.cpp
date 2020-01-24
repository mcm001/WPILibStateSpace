/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <jni.h>

#include <Eigen/Core>

#include <iostream>

#include <unsupported/Eigen/MatrixFunctions>

#include "drake/math/discrete_algebraic_riccati_equation.h"
#include "edu_wpi_first_wpilibj_math_DrakeJNI.h"
#include "wpi/jni_util.h"

using namespace wpi::java;

extern "C" {

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
    // Check to ensure the JNI version is valid

    JNIEnv* env;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
        return JNI_ERR;

    // In here is also where you store things like class references
    // if they are ever needed

    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM* vm, void* reserved) {}

/*
 * Class:     edu_wpi_first_wpilibj_math_DrakeJNI
 * Method:    discreteAlgebraicRiccatiEquation
 * Signature: ([D[D[D[DII[D)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_wpilibj_math_DrakeJNI_discreteAlgebraicRiccatiEquation
  (JNIEnv* env, jclass, jdoubleArray A, jdoubleArray B, jdoubleArray Q,
   jdoubleArray R, jint states, jint inputs, jdoubleArray S)
{

  jboolean isCopyA;
  jboolean isCopyB;
  jboolean isCopyQ;
  jboolean isCopyR;

  jdouble* Aelms = env->GetDoubleArrayElements(A, &isCopyA);
  jdouble* Belms = env->GetDoubleArrayElements(A, &isCopyB);
  jdouble* Qelms = env->GetDoubleArrayElements(A, &isCopyQ);
  jdouble* Relms = env->GetDoubleArrayElements(A, &isCopyR);

  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 
                                  Eigen::RowMajor>> Amat{Aelms, 
                                  states, states};
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 
                                  Eigen::RowMajor>> Bmat{Belms,                                
                                  states, inputs};
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 
                                  Eigen::RowMajor>> Qmat{Qelms,
                                  states, states};
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 
                                  Eigen::RowMajor>> Rmat{Relms,
                                  inputs, inputs};

  Eigen::MatrixXd result =
      drake::math::DiscreteAlgebraicRiccatiEquation(Amat, Bmat, Qmat, Rmat);

  env->SetDoubleArrayRegion(S, 0, states * states, result.data());

  if (isCopyA == JNI_TRUE) {
      env -> ReleaseDoubleArrayElements(A, Aelms, JNI_ABORT);
  }
  if (isCopyB == JNI_TRUE) {
      env -> ReleaseDoubleArrayElements(B, Belms, JNI_ABORT);
  }
  if (isCopyQ == JNI_TRUE) {
      env -> ReleaseDoubleArrayElements(Q, Qelms, JNI_ABORT);
  }
  if (isCopyR == JNI_TRUE) {
      env -> ReleaseDoubleArrayElements(R, Relms, JNI_ABORT);
  }

}

/*
 * Class:     edu_wpi_first_wpilibj_math_DrakeJNI
 * Method:    exp
 * Signature: ([DI[D)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_wpilibj_math_DrakeJNI_exp
  (JNIEnv* env, jclass, jdoubleArray src, jint rows, jdoubleArray dst)
{

  jboolean isCopy;

  jdouble* Aelms = env->GetDoubleArrayElements(src, &isCopy);

  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                  Eigen::RowMajor>> Amat{Aelms,
                                  rows, rows};

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Aexp = Amat.exp();
  env->SetDoubleArrayRegion(dst, 0, rows * rows, Aexp.data());

  if (isCopy == JNI_TRUE) {
      env -> ReleaseDoubleArrayElements(src, Aelms, JNI_ABORT);
  }
}


}  // extern "C"
