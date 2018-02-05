#include <rotation_matrix.h>

#include <iostream>
#include <fstream>

extern Matrix3f rotx (float rad) {
    s = sinf (rad);
    c = cosf (rad);
    tmp << 1., 0., 0.,
           0.,  c,  -s,
           0., s,  c;
    return tmp;
}

extern Matrix3f roty (float rad) {
    s = sinf (rad);
    c = cosf (rad);
    tmp <<  c, 0., s,
            0., 1., 0.,
            -s, 0., c;
    return tmp;
}

extern Matrix3f rotz (float rad) {
    s = sinf (rad);
    c = cosf (rad);
    tmp << c, -s, 0.,
           s, c, 0.,
           0., 0., 1.;
    return tmp;
}

extern Matrix3f QuaternionRotation(float *vector) {
  q.x() = vector[1];
  q.y() = vector[2];
  q.z() = vector[3];
  q.w() = vector[0];
  q.normalize();

  tmp = q.toRotationMatrix();
  return tmp;
}

extern Quaternionf RotationQuaternion(Matrix3f rotation_matrix) {
//   Quaternionf quaternion;
//   
//   quaternion = rotation_matrix;
  //std::cout << "rotation_matrix: "<< rotation_matrix << "\t" << std::endl;

  Quaternionf quaternion(rotation_matrix);
  quaternion.normalize();
  
  //std::cout << "PelvisQuat: "<< quaternion.w() << "\t" <<  quaternion.x() << "\t" << quaternion.y() << "\t" << quaternion.z() << "\t" << std::endl;

  return quaternion;
}

Rotation3f RotationMatrix::QuaternionMatrix(QuaternionJoint* QuaternionJoint_) {
    struct Rotation3f R;

    R.A0 = QuaternionRotation(QuaternionJoint_->Q0);
    R.A1 = QuaternionRotation(QuaternionJoint_->Q1);
    R.A2 = QuaternionRotation(QuaternionJoint_->Q2);
    R.A3 = QuaternionRotation(QuaternionJoint_->Q3);
    R.A4 = QuaternionRotation(QuaternionJoint_->Q4);
    R.A5 = QuaternionRotation(QuaternionJoint_->Q5);
    R.A6 = QuaternionRotation(QuaternionJoint_->Q6);
    R.A7 = QuaternionRotation(QuaternionJoint_->Q7)*rotx(M_PI/2); // Change the orientation with the following rules;
    R.A8 = QuaternionRotation(QuaternionJoint_->Q8)*rotx(M_PI/2);
    R.A9 = QuaternionRotation(QuaternionJoint_->Q9)*rotx(-M_PI/2);
    R.A10 = QuaternionRotation(QuaternionJoint_->Q10)*rotx(-M_PI/2);

    return R;
}

 Rotation3f  RotationMatrix::AngleMatrix(AngleJoint AngleJoint_)const{
     struct Rotation3f R;
     //Global to Base frame
     R.A0= roty(AngleJoint_.q0);
     // Base frame to left leg
     R.A1 = roty(AngleJoint_.q0)*roty(AngleJoint_.q1);
     R.A2 = roty(AngleJoint_.q0)*roty(AngleJoint_.q1)*roty(AngleJoint_.q2);
     R.A3 = roty(AngleJoint_.q0)*roty(AngleJoint_.q1)*roty(AngleJoint_.q2)*roty(AngleJoint_.q3);
     // Base frame to right leg
     R.A4 = roty(AngleJoint_.q0)*roty(AngleJoint_.q4);
     R.A5 = roty(AngleJoint_.q0)*roty(AngleJoint_.q4)*roty(AngleJoint_.q5);
     R.A6 = roty(AngleJoint_.q0)*roty(AngleJoint_.q4)*roty(AngleJoint_.q5)*roty(AngleJoint_.q6);
     // Base frame to left arm
     R.A7 = roty(AngleJoint_.q0)*roty(AngleJoint_.q7);
     R.A8 = roty(AngleJoint_.q0)*roty(AngleJoint_.q7)*roty(AngleJoint_.q8);
     // Base frame to right arm
     R.A9 = roty(AngleJoint_.q0)*roty(AngleJoint_.q9);;
     R.A10 = roty(AngleJoint_.q0)*roty(AngleJoint_.q9)*roty(AngleJoint_.q10);

     return R;
 }
