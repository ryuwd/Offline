
#ifndef RIGIDBODYDOCADERIV_H
#define RIGIDBODYDOCADERIV_H
#include <vector>
double rigidbodyalign_doca_deriv_a0(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_b0(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_a1(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_b1(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_dx(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_dy(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_dz(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_a(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_b(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_g(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double RigidBodyDOCADerivatives_DOCAfn(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz);

std::vector<float> RigidBodyDOCADerivatives_local(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

std::vector<float> RigidBodyDOCADerivatives_global(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

#endif

