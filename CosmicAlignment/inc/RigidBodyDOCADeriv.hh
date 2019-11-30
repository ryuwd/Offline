
#ifndef RIGIDBODYDOCADERIV_H
#define RIGIDBODYDOCADERIV_H

double rigidbodyalign_doca_deriv_a0(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_b0(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_a1(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_b1(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_dx(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_dy(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_dz(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_a(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_b(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

double rigidbodyalign_doca_deriv_g(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

std::vector<double> RigidBodyDOCADerivatives(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz));

#endif

