

#include "CosmicAlignment/inc/RigidBodyDOCADeriv.hh"
#include <math.h>
#include <vector>

double rigidbodyalign_doca_deriv_a0(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1.0/(1 - pow(R2, 2));
    double R4 = a0 - wx;
    double R5 = R4*a1;
    double R6 = b0 - wy;
    double R7 = R6*b1;
    double R8 = R4*wwx + R6*wwy - wwz*wz;
    double R9 = R3*(-R2*(R5 + R7) + R8);
    double R10 = -R9*wwz - wz;
    double R11 = R3*(R2*R8 - R5 - R7);
    double R12 = R11*a1 + R4 - R9*wwx;
    double R13 = R11*b1 + R6 - R9*wwy;
    double R14 = a1*(-R0 - R1) + wwx;
    double R15 = 2*R3;
    double R16 = R15*(R2*wwx - a1);
    double R17 = R14*R15;
    double R18 = (-R10*R14*R3*wwz + (1.0/2.0)*R12*(R16*a1 - R17*wwx + 2) + (1.0/2.0)*R13*(R16*b1 - R17*wwy))/sqrt(pow(R10, 2) + pow(R12, 2) + pow(R13, 2));
    double result = ((R9 > 0) ? (
   R18
)
: (
   -R18
));
    return result;
}


double rigidbodyalign_doca_deriv_b0(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1.0/(1 - pow(R2, 2));
    double R4 = a0 - wx;
    double R5 = R4*a1;
    double R6 = b0 - wy;
    double R7 = R6*b1;
    double R8 = R4*wwx + R6*wwy - wwz*wz;
    double R9 = R3*(-R2*(R5 + R7) + R8);
    double R10 = -R9*wwz - wz;
    double R11 = R3*(R2*R8 - R5 - R7);
    double R12 = R11*a1 + R4 - R9*wwx;
    double R13 = R11*b1 + R6 - R9*wwy;
    double R14 = b1*(-R0 - R1) + wwy;
    double R15 = 2*R3;
    double R16 = R15*(R2*wwy - b1);
    double R17 = R14*R15;
    double R18 = (-R10*R14*R3*wwz + (1.0/2.0)*R12*(R16*a1 - R17*wwx) + (1.0/2.0)*R13*(R16*b1 - R17*wwy + 2))/sqrt(pow(R10, 2) + pow(R12, 2) + pow(R13, 2));
    double result = ((R9 > 0) ? (
   R18
)
: (
   -R18
));
    return result;
}


double rigidbodyalign_doca_deriv_a1(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1 - pow(R2, 2);
    double R4 = 1.0/R3;
    double R5 = a0 - wx;
    double R6 = R5*a1;
    double R7 = b0 - wy;
    double R8 = R7*b1;
    double R9 = R6 + R8;
    double R10 = R5*wwx + R7*wwy - wwz*wz;
    double R11 = R10 - R2*R9;
    double R12 = R11*R4;
    double R13 = -R12*wwz - wz;
    double R14 = R10*R2 - R6 - R8;
    double R15 = R14*R4;
    double R16 = -R12*wwx + R15*a1 + R5;
    double R17 = -R12*wwy + R15*b1 + R7;
    double R18 = 2*R4;
    double R19 = R18*(R5*(-R0 - R1) - R9*wwx);
    double R20 = 4*R2/pow(R3, 2);
    double R21 = R11*R20;
    double R22 = R21*wwx;
    double R23 = R18*(R10*wwx - a0 + wx);
    double R24 = R14*R20;
    double R25 = ((1.0/2.0)*R13*(-R19*wwz - R22*wwz) + (1.0/2.0)*R16*(R0*R24 + 2*R15 - R19*wwx - R21*pow(wwx, 2) + R23*a1) + (1.0/2.0)*R17*(-R19*wwy - R22*wwy + R23*b1 + R24*b1*wwx))/sqrt(pow(R13, 2) + pow(R16, 2) + pow(R17, 2));
    double result = ((R12 > 0) ? (
   R25
)
: (
   -R25
));
    return result;
}


double rigidbodyalign_doca_deriv_b1(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1 - pow(R2, 2);
    double R4 = 1.0/R3;
    double R5 = a0 - wx;
    double R6 = R5*a1;
    double R7 = b0 - wy;
    double R8 = R7*b1;
    double R9 = R6 + R8;
    double R10 = R5*wwx + R7*wwy - wwz*wz;
    double R11 = R10 - R2*R9;
    double R12 = R11*R4;
    double R13 = -R12*wwz - wz;
    double R14 = R10*R2 - R6 - R8;
    double R15 = R14*R4;
    double R16 = -R12*wwx + R15*a1 + R5;
    double R17 = -R12*wwy + R15*b1 + R7;
    double R18 = 2*R4;
    double R19 = R18*(R7*(-R0 - R1) - R9*wwy);
    double R20 = 4*R2/pow(R3, 2);
    double R21 = R11*R20;
    double R22 = R21*wwy;
    double R23 = R18*(R10*wwy - b0 + wy);
    double R24 = R14*R20;
    double R25 = ((1.0/2.0)*R13*(-R19*wwz - R22*wwz) + (1.0/2.0)*R16*(-R19*wwx - R22*wwx + R23*a1 + R24*a1*wwy) + (1.0/2.0)*R17*(R1*R24 + 2*R15 - R19*wwy - R21*pow(wwy, 2) + R23*b1))/sqrt(pow(R13, 2) + pow(R16, 2) + pow(R17, 2));
    double result = ((R12 > 0) ? (
   R25
)
: (
   -R25
));
    return result;
}


double rigidbodyalign_doca_deriv_dx(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1.0/(1 - pow(R2, 2));
    double R4 = a0 - wx;
    double R5 = R4*a1;
    double R6 = b0 - wy;
    double R7 = R6*b1;
    double R8 = R4*wwx + R6*wwy - wwz*wz;
    double R9 = R3*(-R2*(R5 + R7) + R8);
    double R10 = -R9*wwz - wz;
    double R11 = R3*(R2*R8 - R5 - R7);
    double R12 = R11*a1 + R4 - R9*wwx;
    double R13 = R11*b1 + R6 - R9*wwy;
    double R14 = -a1*(-R0 - R1) - wwx;
    double R15 = 2*R3;
    double R16 = R15*(-R2*wwx + a1);
    double R17 = R14*R15;
    double R18 = (-R10*R14*R3*wwz + (1.0/2.0)*R12*(R16*a1 - R17*wwx - 2) + (1.0/2.0)*R13*(R16*b1 - R17*wwy))/sqrt(pow(R10, 2) + pow(R12, 2) + pow(R13, 2));
    double result = ((R9 > 0) ? (
   R18
)
: (
   -R18
));
    return result;
}


double rigidbodyalign_doca_deriv_dy(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1.0/(1 - pow(R2, 2));
    double R4 = a0 - wx;
    double R5 = R4*a1;
    double R6 = b0 - wy;
    double R7 = R6*b1;
    double R8 = R4*wwx + R6*wwy - wwz*wz;
    double R9 = R3*(-R2*(R5 + R7) + R8);
    double R10 = -R9*wwz - wz;
    double R11 = R3*(R2*R8 - R5 - R7);
    double R12 = R11*a1 + R4 - R9*wwx;
    double R13 = R11*b1 + R6 - R9*wwy;
    double R14 = -b1*(-R0 - R1) - wwy;
    double R15 = 2*R3;
    double R16 = R15*(-R2*wwy + b1);
    double R17 = R14*R15;
    double R18 = (-R10*R14*R3*wwz + (1.0/2.0)*R12*(R16*a1 - R17*wwx) + (1.0/2.0)*R13*(R16*b1 - R17*wwy - 2))/sqrt(pow(R10, 2) + pow(R12, 2) + pow(R13, 2));
    double result = ((R9 > 0) ? (
   R18
)
: (
   -R18
));
    return result;
}


double rigidbodyalign_doca_deriv_dz(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx + b1*wwy;
    double R1 = 1.0/(1 - pow(R0, 2));
    double R2 = a0 - wx;
    double R3 = R2*a1;
    double R4 = b0 - wy;
    double R5 = R4*b1;
    double R6 = R2*wwx + R4*wwy - wwz*wz;
    double R7 = R1*(-R0*(R3 + R5) + R6);
    double R8 = -R7*wwz - wz;
    double R9 = R1*(R0*R6 - R3 - R5);
    double R10 = R2 - R7*wwx + R9*a1;
    double R11 = R4 - R7*wwy + R9*b1;
    double R12 = 2*R1;
    double R13 = R12*wwz;
    double R14 = R0*R13;
    double R15 = ((1.0/2.0)*R10*(R13*wwx - R14*a1) + (1.0/2.0)*R11*(R13*wwy - R14*b1) + (1.0/2.0)*R8*(R12*pow(wwz, 2) - 2))/sqrt(pow(R10, 2) + pow(R11, 2) + pow(R8, 2));
    double result = ((R7 > 0) ? (
   R15
)
: (
   -R15
));
    return result;
}


double rigidbodyalign_doca_deriv_a(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = -wz;
    double R1 = a1*wwx;
    double R2 = b1*wwy;
    double R3 = R1 + R2;
    double R4 = 1 - pow(R3, 2);
    double R5 = 1.0/R4;
    double R6 = a0 - wx;
    double R7 = R6*a1;
    double R8 = b0 - wy;
    double R9 = R8*b1;
    double R10 = R7 + R9;
    double R11 = R6*wwx + R8*wwy - wwz*wz;
    double R12 = -R10*R3 + R11;
    double R13 = R12*R5;
    double R14 = R13*wwz;
    double R15 = R0 - R14;
    double R16 = R11*R3 - R7 - R9;
    double R17 = R16*R5;
    double R18 = -R13*wwx + R17*a1 + R6;
    double R19 = R13*wwy;
    double R20 = R17*b1 - R19 + R8;
    double R21 = 4*R3/pow(R4, 2);
    double R22 = R12*R21;
    double R23 = R0 + ppz;
    double R24 = R23*b1;
    double R25 = b1*wwz;
    double R26 = R23*wwy + R8*wwz + wwy*wz + wwz*(-ppy + wy);
    double R27 = 2*R5;
    double R28 = R27*(-R10*R25 + R24*(-R1 - R2) + R26);
    double R29 = R16*R21;
    double R30 = R27*(R11*R25 - R24 + R26*R3);
    double R31 = ((1.0/2.0)*R15*(2*R19 - R22*b1*pow(wwz, 2) - R28*wwz - 2*ppy + 2*wy) + (1.0/2.0)*R18*(-R22*R25*wwx + R25*R29*a1 - R28*wwx + R30*a1) + (1.0/2.0)*R20*(-2*R14 - R2*R22*wwz - R28*wwy + R29*pow(b1, 2)*wwz + R30*b1 + 2*ppz - 2*wz))/sqrt(pow(R15, 2) + pow(R18, 2) + pow(R20, 2));
    double result = ((R13 > 0) ? (
   R31
)
: (
   -R31
));
    return result;
}


double rigidbodyalign_doca_deriv_b(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = -wz;
    double R1 = a1*wwx;
    double R2 = b1*wwy;
    double R3 = R1 + R2;
    double R4 = 1 - pow(R3, 2);
    double R5 = 1.0/R4;
    double R6 = -wx;
    double R7 = R6 + a0;
    double R8 = R7*a1;
    double R9 = b0 - wy;
    double R10 = R9*b1;
    double R11 = R10 + R8;
    double R12 = R7*wwx + R9*wwy - wwz*wz;
    double R13 = -R11*R3 + R12;
    double R14 = R13*R5;
    double R15 = R14*wwz;
    double R16 = R0 - R15;
    double R17 = -R10 + R12*R3 - R8;
    double R18 = R17*R5;
    double R19 = R14*wwx;
    double R20 = R18*a1 - R19 + R7;
    double R21 = -R14*wwy + R18*b1 + R9;
    double R22 = 4*R3/pow(R4, 2);
    double R23 = R13*R22;
    double R24 = R0 + ppz;
    double R25 = R24*a1;
    double R26 = a1*wwz;
    double R27 = R24*wwx + R7*wwz - wwx*wz + wwz*(R6 + ppx);
    double R28 = 2*R5;
    double R29 = R28*(-R11*R26 + R25*(-R1 - R2) + R27);
    double R30 = R17*R22;
    double R31 = R28*(R12*R26 - R25 + R27*R3);
    double R32 = ((1.0/2.0)*R16*(-2*R19 - R23*a1*pow(wwz, 2) - R29*wwz + 2*ppx - 2*wx) + (1.0/2.0)*R20*(-R1*R23*wwz - 2*R15 - R29*wwx + R30*pow(a1, 2)*wwz + R31*a1 + 2*ppz - 2*wz) + (1.0/2.0)*R21*(-R23*R26*wwy + R26*R30*b1 - R29*wwy + R31*b1))/sqrt(pow(R16, 2) + pow(R20, 2) + pow(R21, 2));
    double result = ((R14 > 0) ? (
   R32
)
: (
   -R32
));
    return result;
}


double rigidbodyalign_doca_deriv_g(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1 - pow(R2, 2);
    double R4 = 1.0/R3;
    double R5 = a0 - wx;
    double R6 = R5*a1;
    double R7 = -wy;
    double R8 = R7 + b0;
    double R9 = R8*b1;
    double R10 = R6 + R9;
    double R11 = R5*wwx + R8*wwy - wwz*wz;
    double R12 = -R10*R2 + R11;
    double R13 = R12*R4;
    double R14 = -R13*wwz - wz;
    double R15 = R11*R2 - R6 - R9;
    double R16 = R15*R4;
    double R17 = R13*wwx;
    double R18 = R16*a1 - R17 + R5;
    double R19 = R13*wwy;
    double R20 = R16*b1 - R19 + R8;
    double R21 = a1*wwy;
    double R22 = b1*wwx;
    double R23 = 2*R2*(2*R21 - 2*R22)/pow(R3, 2);
    double R24 = R12*R23;
    double R25 = R7 + ppy;
    double R26 = R25*a1;
    double R27 = -ppx + wx;
    double R28 = R27*b1;
    double R29 = R25*wwx + R27*wwy + R5*wwy - R8*wwx;
    double R30 = 2*R4;
    double R31 = R30*(R10*(-R21 + R22) + R29 + (-R0 - R1)*(R26 + R28));
    double R32 = R15*R23;
    double R33 = R30*(R11*(R21 - R22) + R2*R29 - R26 - R28);
    double R34 = ((1.0/2.0)*R14*(-R24*wwz - R31*wwz) + (1.0/2.0)*R18*(-2*R19 - R24*wwx - R31*wwx + R32*a1 + R33*a1 + 2*ppy - 2*wy) + (1.0/2.0)*R20*(2*R17 - R24*wwy - R31*wwy + R32*b1 + R33*b1 - 2*ppx + 2*wx))/sqrt(pow(R14, 2) + pow(R18, 2) + pow(R20, 2));
    double result = ((R13 > 0) ? (
   R34
)
: (
   -R34
));
    return result;
}


double RigidBodyDOCADerivatives_DOCAfn(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz)
{
    double R0 = a1*wwx + b1*wwy;
    double R1 = 1.0/(1 - pow(R0, 2));
    double R2 = a0 - wx;
    double R3 = R2*a1;
    double R4 = b0 - wy;
    double R5 = R4*b1;
    double R6 = R2*wwx + R4*wwy - wwz*wz;
    double R7 = R1*(-R0*(R3 + R5) + R6);
    double R8 = R1*(R0*R6 - R3 - R5);
    double R9 = sqrt(pow(-R7*wwz - wz, 2) + pow(R2 - R7*wwx + R8*a1, 2) + pow(R4 - R7*wwy + R8*b1, 2));
    double result = ((R7 > 0) ? (
   R9
)
: (
   -R9
));
    return result;
}




std::vector<float> RigidBodyDOCADerivatives_local(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    std::vector<float> result;

result.push_back(rigidbodyalign_doca_deriv_a0(a0,b0,a1,b1,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_b0(a0,b0,a1,b1,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_a1(a0,b0,a1,b1,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_b1(a0,b0,a1,b1,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));

    return result;
}





std::vector<float> RigidBodyDOCADerivatives_global(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    std::vector<float> result;

result.push_back(rigidbodyalign_doca_deriv_dx(a0,b0,a1,b1,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_dy(a0,b0,a1,b1,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_dz(a0,b0,a1,b1,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_a(a0,b0,a1,b1,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_b(a0,b0,a1,b1,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_g(a0,b0,a1,b1,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));

    return result;
}



