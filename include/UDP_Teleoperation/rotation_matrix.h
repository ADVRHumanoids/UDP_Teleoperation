#ifndef ROTATION_MATRIX_H
#define ROTATION_MATRIX_H

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/SVD>

#define DEG2RAD(r) (r*M_PI/180)
#define RAD2DEG(r) (r*180/M_PI)

using namespace Eigen;
typedef Matrix<float, 10, 1> VectorQf;
typedef Matrix<float, 2, 1> VectorCpf;
typedef Matrix<float, 4, 1> VectorCpRL;
typedef Matrix<float, 11, 1> VectorJf;
typedef Matrix<float, 14, 1> VectorJ14f;
typedef Matrix<float, 5, 1> VectorJ5f;
typedef Matrix<float, 8, 1> VectorJ8f;
typedef Matrix<float, 3, 11> MatrixJf;
typedef Matrix<float, 3, 14> MatrixJ14f;
typedef Matrix<float, 3, 2> MatrixJHandf;
typedef Matrix<float, 3, 3> MatrixJLegf;
typedef Matrix<float, 31, 1> VectorSESC;
typedef Matrix<float, 2,2> Matrix2f;
typedef Matrix<float, 2,3> Matrix23f;
typedef Matrix<float, 6,2> Matrix62f;
typedef Matrix<int, 8, 1> VectorJ8i;

typedef Matrix<double, 66, 1> NNinput_m;
typedef Matrix<double, 62, 1> Bmat_d;
typedef Matrix<float, 62, 1> Bmat_f;
typedef Matrix<double, 4, 1> NNoutput_m;

static float s, c;
static Quaternionf q;
static Matrix3f tmp;

extern Matrix3f rotx (float rad);
extern Matrix3f roty (float rad);
extern Matrix3f rotz (float rad);
extern Matrix3f QuaternionRotation(float *vector);
extern Quaternionf RotationQuaternion(Matrix3f rotation_matrix);

struct QuaternionJoint{
    float *Q0;  // L5
    float *Q1;  // L-Thigh
    float *Q2;  // L-Shank
    float *Q3;  // L-Foot
    float *Q4;  // R-Thigh
    float *Q5;  // R-Shank
    float *Q6;  // R-Foot
    float *Q7;  // L-UpperArm
    float *Q8;  // L-LowerArm
    float *Q9;  // R-UpperArm
    float *Q10; // R-LowerArm
};

struct AngleJoint{
    float q0;   // Pelvis
    float q1;   // L-Hip
    float q2;   // L-Knee
    float q3;   // L-Ankle
    float q4;   // R-Hip
    float q5;   // R-Knee
    float q6;   // R-Ankle
    float q7;   // L-Shoulder
    float q8;   // L-Elbow
    float q9;  // R-Shoulder
    float q10;  // R-Elbow
};

struct Rotation3f {
    Matrix3f A0;
    Matrix3f A1;
    Matrix3f A2;
    Matrix3f A3;
    Matrix3f A4;
    Matrix3f A5;
    Matrix3f A6;
    Matrix3f A7;
    Matrix3f A8;
    Matrix3f A9;
    Matrix3f A10;
};

class RotationMatrix
{
private:
//    struct AngleJoint AngleJoint_;
//    struct Rotation3f rotationmatrix;
public:
    RotationMatrix() {}
//     virtual ~RotationMatrix();
    Rotation3f QuaternionMatrix(QuaternionJoint* QuaternionJoint_);
    struct Rotation3f AngleMatrix(struct AngleJoint) const;
};

#endif // ROTATION_MATRIX_H
