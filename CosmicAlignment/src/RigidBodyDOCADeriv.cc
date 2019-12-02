

#include "CosmicAlignment/inc/RigidBodyDOCADeriv.hh"
#include <math.h>
#include <vector>

double rigidbodyalign_doca_deriv_a0(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1.0/(1 - pow(R2, 2));
    double R4 = a0 + ppx - wx;
    double R5 = R4*a1;
    double R6 = b0 + ppy - wy;
    double R7 = R6*b1;
    double R8 = ppz - wz;
    double R9 = R4*wwx + R6*wwy + R8*wwz;
    double R10 = R3*(-R2*(R5 + R7) + R9);
    double R11 = -R10*wwz + R8;
    double R12 = R3*(R2*R9 - R5 - R7);
    double R13 = -R10*wwx + R12*a1 + R4;
    double R14 = -R10*wwy + R12*b1 + R6;
    double R15 = a1*(-R0 - R1) + wwx;
    double R16 = 2*R3;
    double R17 = R16*(R2*wwx - a1);
    double R18 = R15*R16;
    double R19 = (-R11*R15*R3*wwz + (1.0/2.0)*R13*(R17*a1 - R18*wwx + 2) + (1.0/2.0)*R14*(R17*b1 - R18*wwy))/sqrt(pow(R11, 2) + pow(R13, 2) + pow(R14, 2));
    double result = ((R10 > 0) ? (
   R19
)
: (
   -R19
));
    return result;
}


double rigidbodyalign_doca_deriv_b0(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1.0/(1 - pow(R2, 2));
    double R4 = a0 + ppx - wx;
    double R5 = R4*a1;
    double R6 = b0 + ppy - wy;
    double R7 = R6*b1;
    double R8 = ppz - wz;
    double R9 = R4*wwx + R6*wwy + R8*wwz;
    double R10 = R3*(-R2*(R5 + R7) + R9);
    double R11 = -R10*wwz + R8;
    double R12 = R3*(R2*R9 - R5 - R7);
    double R13 = -R10*wwx + R12*a1 + R4;
    double R14 = -R10*wwy + R12*b1 + R6;
    double R15 = b1*(-R0 - R1) + wwy;
    double R16 = 2*R3;
    double R17 = R16*(R2*wwy - b1);
    double R18 = R15*R16;
    double R19 = (-R11*R15*R3*wwz + (1.0/2.0)*R13*(R17*a1 - R18*wwx) + (1.0/2.0)*R14*(R17*b1 - R18*wwy + 2))/sqrt(pow(R11, 2) + pow(R13, 2) + pow(R14, 2));
    double result = ((R10 > 0) ? (
   R19
)
: (
   -R19
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
    double R5 = a0 + ppx - wx;
    double R6 = R5*a1;
    double R7 = b0 + ppy - wy;
    double R8 = R7*b1;
    double R9 = R6 + R8;
    double R10 = ppz - wz;
    double R11 = R10*wwz + R5*wwx + R7*wwy;
    double R12 = R11 - R2*R9;
    double R13 = R12*R4;
    double R14 = R10 - R13*wwz;
    double R15 = R11*R2 - R6 - R8;
    double R16 = R15*R4;
    double R17 = -R13*wwx + R16*a1 + R5;
    double R18 = -R13*wwy + R16*b1 + R7;
    double R19 = 2*R4;
    double R20 = R19*(R5*(-R0 - R1) - R9*wwx);
    double R21 = 4*R2/pow(R3, 2);
    double R22 = R12*R21;
    double R23 = R22*wwx;
    double R24 = R19*(R11*wwx - a0 - ppx + wx);
    double R25 = R15*R21;
    double R26 = ((1.0/2.0)*R14*(-R20*wwz - R23*wwz) + (1.0/2.0)*R17*(R0*R25 + 2*R16 - R20*wwx - R22*pow(wwx, 2) + R24*a1) + (1.0/2.0)*R18*(-R20*wwy - R23*wwy + R24*b1 + R25*b1*wwx))/sqrt(pow(R14, 2) + pow(R17, 2) + pow(R18, 2));
    double result = ((R13 > 0) ? (
   R26
)
: (
   -R26
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
    double R5 = a0 + ppx - wx;
    double R6 = R5*a1;
    double R7 = b0 + ppy - wy;
    double R8 = R7*b1;
    double R9 = R6 + R8;
    double R10 = ppz - wz;
    double R11 = R10*wwz + R5*wwx + R7*wwy;
    double R12 = R11 - R2*R9;
    double R13 = R12*R4;
    double R14 = R10 - R13*wwz;
    double R15 = R11*R2 - R6 - R8;
    double R16 = R15*R4;
    double R17 = -R13*wwx + R16*a1 + R5;
    double R18 = -R13*wwy + R16*b1 + R7;
    double R19 = 2*R4;
    double R20 = R19*(R7*(-R0 - R1) - R9*wwy);
    double R21 = 4*R2/pow(R3, 2);
    double R22 = R12*R21;
    double R23 = R22*wwy;
    double R24 = R19*(R11*wwy - b0 - ppy + wy);
    double R25 = R15*R21;
    double R26 = ((1.0/2.0)*R14*(-R20*wwz - R23*wwz) + (1.0/2.0)*R17*(-R20*wwx - R23*wwx + R24*a1 + R25*a1*wwy) + (1.0/2.0)*R18*(R1*R25 + 2*R16 - R20*wwy - R22*pow(wwy, 2) + R24*b1))/sqrt(pow(R14, 2) + pow(R17, 2) + pow(R18, 2));
    double result = ((R13 > 0) ? (
   R26
)
: (
   -R26
));
    return result;
}


double rigidbodyalign_doca_deriv_dx(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1.0/(1 - pow(R2, 2));
    double R4 = a0 + ppx - wx;
    double R5 = R4*a1;
    double R6 = b0 + ppy - wy;
    double R7 = R6*b1;
    double R8 = ppz - wz;
    double R9 = R4*wwx + R6*wwy + R8*wwz;
    double R10 = R3*(-R2*(R5 + R7) + R9);
    double R11 = -R10*wwz + R8;
    double R12 = R3*(R2*R9 - R5 - R7);
    double R13 = -R10*wwx + R12*a1 + R4;
    double R14 = -R10*wwy + R12*b1 + R6;
    double R15 = -a1*(-R0 - R1) - wwx;
    double R16 = 2*R3;
    double R17 = R16*(-R2*wwx + a1);
    double R18 = R15*R16;
    double R19 = (-R11*R15*R3*wwz + (1.0/2.0)*R13*(R17*a1 - R18*wwx - 2) + (1.0/2.0)*R14*(R17*b1 - R18*wwy))/sqrt(pow(R11, 2) + pow(R13, 2) + pow(R14, 2));
    double result = ((R10 > 0) ? (
   R19
)
: (
   -R19
));
    return result;
}


double rigidbodyalign_doca_deriv_dy(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1.0/(1 - pow(R2, 2));
    double R4 = a0 + ppx - wx;
    double R5 = R4*a1;
    double R6 = b0 + ppy - wy;
    double R7 = R6*b1;
    double R8 = ppz - wz;
    double R9 = R4*wwx + R6*wwy + R8*wwz;
    double R10 = R3*(-R2*(R5 + R7) + R9);
    double R11 = -R10*wwz + R8;
    double R12 = R3*(R2*R9 - R5 - R7);
    double R13 = -R10*wwx + R12*a1 + R4;
    double R14 = -R10*wwy + R12*b1 + R6;
    double R15 = -b1*(-R0 - R1) - wwy;
    double R16 = 2*R3;
    double R17 = R16*(-R2*wwy + b1);
    double R18 = R15*R16;
    double R19 = (-R11*R15*R3*wwz + (1.0/2.0)*R13*(R17*a1 - R18*wwx) + (1.0/2.0)*R14*(R17*b1 - R18*wwy - 2))/sqrt(pow(R11, 2) + pow(R13, 2) + pow(R14, 2));
    double result = ((R10 > 0) ? (
   R19
)
: (
   -R19
));
    return result;
}


double rigidbodyalign_doca_deriv_dz(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx + b1*wwy;
    double R1 = 1.0/(1 - pow(R0, 2));
    double R2 = a0 + ppx - wx;
    double R3 = R2*a1;
    double R4 = b0 + ppy - wy;
    double R5 = R4*b1;
    double R6 = ppz - wz;
    double R7 = R2*wwx + R4*wwy + R6*wwz;
    double R8 = R1*(-R0*(R3 + R5) + R7);
    double R9 = R6 - R8*wwz;
    double R10 = R1*(R0*R7 - R3 - R5);
    double R11 = R10*a1 + R2 - R8*wwx;
    double R12 = R10*b1 + R4 - R8*wwy;
    double R13 = 2*R1;
    double R14 = R13*wwz;
    double R15 = R0*R14;
    double R16 = ((1.0/2.0)*R11*(R14*wwx - R15*a1) + (1.0/2.0)*R12*(R14*wwy - R15*b1) + (1.0/2.0)*R9*(R13*pow(wwz, 2) - 2))/sqrt(pow(R11, 2) + pow(R12, 2) + pow(R9, 2));
    double result = ((R8 > 0) ? (
   R16
)
: (
   -R16
));
    return result;
}


double rigidbodyalign_doca_deriv_a(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1 - pow(R2, 2);
    double R4 = 1.0/R3;
    double R5 = a0 + ppx - wx;
    double R6 = R5*a1;
    double R7 = b0 + ppy - wy;
    double R8 = R7*b1;
    double R9 = R6 + R8;
    double R10 = ppz - wz;
    double R11 = R10*wwz + R5*wwx + R7*wwy;
    double R12 = R11 - R2*R9;
    double R13 = R12*R4;
    double R14 = R13*wwz;
    double R15 = R10 - R14;
    double R16 = R11*R2 - R6 - R8;
    double R17 = R16*R4;
    double R18 = -R13*wwx + R17*a1 + R5;
    double R19 = R13*wwy;
    double R20 = R17*b1 - R19 + R7;
    double R21 = R10*b1;
    double R22 = b1*wwz;
    double R23 = R7*wwz + wwz*(-ppy + wy);
    double R24 = 2*R4;
    double R25 = R24*(R21*(-R0 - R1) - R22*R9 + R23);
    double R26 = 4*R2/pow(R3, 2);
    double R27 = R12*R26;
    double R28 = R24*(R11*R22 + R2*R23 - R21);
    double R29 = R16*R26;
    double R30 = ((1.0/2.0)*R15*(2*R19 - R25*wwz - R27*b1*pow(wwz, 2) - 2*ppy + 2*wy) + (1.0/2.0)*R18*(-R22*R27*wwx + R22*R29*a1 - R25*wwx + R28*a1) + (1.0/2.0)*R20*(-R1*R27*wwz - 2*R14 - R25*wwy + R28*b1 + R29*pow(b1, 2)*wwz + 2*ppz - 2*wz))/sqrt(pow(R15, 2) + pow(R18, 2) + pow(R20, 2));
    double result = ((R13 > 0) ? (
   R30
)
: (
   -R30
));
    return result;
}


double rigidbodyalign_doca_deriv_b(double a0, double b0, double a1, double b1, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = a1*wwx;
    double R1 = b1*wwy;
    double R2 = R0 + R1;
    double R3 = 1 - pow(R2, 2);
    double R4 = 1.0/R3;
    double R5 = ppx - wx;
    double R6 = R5 + a0;
    double R7 = R6*a1;
    double R8 = b0 + ppy - wy;
    double R9 = R8*b1;
    double R10 = R7 + R9;
    double R11 = ppz - wz;
    double R12 = R11*wwz + R6*wwx + R8*wwy;
    double R13 = -R10*R2 + R12;
    double R14 = R13*R4;
    double R15 = R14*wwz;
    double R16 = R11 - R15;
    double R17 = R12*R2 - R7 - R9;
    double R18 = R17*R4;
    double R19 = R14*wwx;
    double R20 = R18*a1 - R19 + R6;
    double R21 = -R14*wwy + R18*b1 + R8;
    double R22 = 4*R2/pow(R3, 2);
    double R23 = R13*R22;
    double R24 = R11*a1;
    double R25 = a1*wwz;
    double R26 = 2*wwx;
    double R27 = R11*R26 + R5*wwz + R6*wwz;
    double R28 = -R10*R25 + R24*(-R0 - R1) + R27;
    double R29 = 2*R4;
    double R30 = R28*R29;
    double R31 = R17*R22;
    double R32 = R29*(R12*R25 + R2*R27 - R24);
    double R33 = ((1.0/2.0)*R16*(-2*R19 - R23*a1*pow(wwz, 2) - R30*wwz + 2*ppx - 2*wx) + (1.0/2.0)*R20*(-R0*R23*wwz - 2*R15 - R26*R28*R4 + R31*pow(a1, 2)*wwz + R32*a1 + 2*ppz - 2*wz) + (1.0/2.0)*R21*(-R23*R25*wwy + R25*R31*b1 - R30*wwy + R32*b1))/sqrt(pow(R16, 2) + pow(R20, 2) + pow(R21, 2));
    double result = ((R14 > 0) ? (
   R33
)
: (
   -R33
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
    double R5 = a0 + ppx - wx;
    double R6 = R5*a1;
    double R7 = ppy - wy;
    double R8 = R7 + b0;
    double R9 = R8*b1;
    double R10 = R6 + R9;
    double R11 = ppz - wz;
    double R12 = R11*wwz + R5*wwx + R8*wwy;
    double R13 = -R10*R2 + R12;
    double R14 = R13*R4;
    double R15 = R11 - R14*wwz;
    double R16 = R12*R2 - R6 - R9;
    double R17 = R16*R4;
    double R18 = R14*wwx;
    double R19 = R17*a1 - R18 + R5;
    double R20 = R14*wwy;
    double R21 = R17*b1 - R20 + R8;
    double R22 = a1*wwy;
    double R23 = b1*wwx;
    double R24 = 2*R2*(2*R22 - 2*R23)/pow(R3, 2);
    double R25 = R13*R24;
    double R26 = R7*a1;
    double R27 = -ppx + wx;
    double R28 = R27*b1;
    double R29 = R27*wwy + R5*wwy + R7*wwx - R8*wwx;
    double R30 = 2*R4;
    double R31 = R30*(R10*(-R22 + R23) + R29 + (-R0 - R1)*(R26 + R28));
    double R32 = R16*R24;
    double R33 = R30*(R12*(R22 - R23) + R2*R29 - R26 - R28);
    double R34 = ((1.0/2.0)*R15*(-R25*wwz - R31*wwz) + (1.0/2.0)*R19*(-2*R20 - R25*wwx - R31*wwx + R32*a1 + R33*a1 + 2*ppy - 2*wy) + (1.0/2.0)*R21*(2*R18 - R25*wwy - R31*wwy + R32*b1 + R33*b1 - 2*ppx + 2*wx))/sqrt(pow(R15, 2) + pow(R19, 2) + pow(R21, 2));
    double result = ((R14 > 0) ? (
   R34
)
: (
   -R34
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



