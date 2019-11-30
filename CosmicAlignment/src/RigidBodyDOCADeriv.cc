

#include "CosmicAlignment/inc/RigidBodyDOCADeriv.hh"
#include <math.h>
#include <vector>

double rigidbodyalign_doca_deriv_a0(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = cos(a);
    double R1 = cos(b);
    double R2 = R1*wwz;
    double R3 = sin(a);
    double R4 = sin(g);
    double R5 = R3*R4;
    double R6 = sin(b);
    double R7 = cos(g);
    double R8 = R0*R7;
    double R9 = R5 + R6*R8;
    double R10 = R3*R7;
    double R11 = R0*R4;
    double R12 = -R10 + R11*R6;
    double R13 = R0*R2 + R12*wwy + R9*wwx;
    double R14 = R1*R7;
    double R15 = R1*R4;
    double R16 = R14*wwx + R15*wwy - R6*wwz;
    double R17 = R16*a1;
    double R18 = R5*R6 + R8;
    double R19 = R10*R6 - R11;
    double R20 = R18*wwy + R19*wwx + R2*R3;
    double R21 = R20*b1;
    double R22 = R17 + R21;
    double R23 = 1.0/(1 - pow(R22, 2));
    double R24 = -ppz + wz;
    double R25 = -ppx + wx;
    double R26 = -ppy + wy;
    double R27 = -R14*R25 - R15*R26 + R24*R6 + a0 - dx;
    double R28 = R27*a1;
    double R29 = R1*R24;
    double R30 = -R18*R26 - R19*R25 - R29*R3 + b0 - dy;
    double R31 = R30*b1;
    double R32 = -R0*R29 - R12*R26 - R25*R9 - dz;
    double R33 = R13*R32 + R16*R27 + R20*R30;
    double R34 = R23*(-R22*(R28 + R31) + R33);
    double R35 = -R13*R34 + R32;
    double R36 = R23*(R22*R33 - R28 - R31);
    double R37 = -R16*R34 + R27 + R36*a1;
    double R38 = -R20*R34 + R30 + R36*b1;
    double R39 = R16 + a1*(-R17 - R21);
    double R40 = 2*R23;
    double R41 = R40*(R16*R22 - a1);
    double R42 = R39*R40;
    double R43 = (-R13*R23*R35*R39 + (1.0/2.0)*R37*(-R16*R42 + R41*a1 + 2) + (1.0/2.0)*R38*(-R20*R42 + R41*b1))/sqrt(pow(R35, 2) + pow(R37, 2) + pow(R38, 2));
    double result = ((R34 > 0) ? (
   R43
)
: (
   -R43
));
    return result;
}


double rigidbodyalign_doca_deriv_b0(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = cos(a);
    double R1 = cos(b);
    double R2 = R1*wwz;
    double R3 = sin(a);
    double R4 = sin(g);
    double R5 = R3*R4;
    double R6 = sin(b);
    double R7 = cos(g);
    double R8 = R0*R7;
    double R9 = R5 + R6*R8;
    double R10 = R3*R7;
    double R11 = R0*R4;
    double R12 = -R10 + R11*R6;
    double R13 = R0*R2 + R12*wwy + R9*wwx;
    double R14 = R1*R7;
    double R15 = R1*R4;
    double R16 = R14*wwx + R15*wwy - R6*wwz;
    double R17 = R16*a1;
    double R18 = R5*R6 + R8;
    double R19 = R10*R6 - R11;
    double R20 = R18*wwy + R19*wwx + R2*R3;
    double R21 = R20*b1;
    double R22 = R17 + R21;
    double R23 = 1.0/(1 - pow(R22, 2));
    double R24 = -ppz + wz;
    double R25 = -ppx + wx;
    double R26 = -ppy + wy;
    double R27 = -R14*R25 - R15*R26 + R24*R6 + a0 - dx;
    double R28 = R27*a1;
    double R29 = R1*R24;
    double R30 = -R18*R26 - R19*R25 - R29*R3 + b0 - dy;
    double R31 = R30*b1;
    double R32 = -R0*R29 - R12*R26 - R25*R9 - dz;
    double R33 = R13*R32 + R16*R27 + R20*R30;
    double R34 = R23*(-R22*(R28 + R31) + R33);
    double R35 = -R13*R34 + R32;
    double R36 = R23*(R22*R33 - R28 - R31);
    double R37 = -R16*R34 + R27 + R36*a1;
    double R38 = -R20*R34 + R30 + R36*b1;
    double R39 = R20 + b1*(-R17 - R21);
    double R40 = 2*R23;
    double R41 = R40*(R20*R22 - b1);
    double R42 = R39*R40;
    double R43 = (-R13*R23*R35*R39 + (1.0/2.0)*R37*(-R16*R42 + R41*a1) + (1.0/2.0)*R38*(-R20*R42 + R41*b1 + 2))/sqrt(pow(R35, 2) + pow(R37, 2) + pow(R38, 2));
    double result = ((R34 > 0) ? (
   R43
)
: (
   -R43
));
    return result;
}


double rigidbodyalign_doca_deriv_a1(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = cos(a);
    double R1 = cos(b);
    double R2 = R1*wwz;
    double R3 = sin(a);
    double R4 = sin(g);
    double R5 = R3*R4;
    double R6 = sin(b);
    double R7 = cos(g);
    double R8 = R0*R7;
    double R9 = R5 + R6*R8;
    double R10 = R3*R7;
    double R11 = R0*R4;
    double R12 = -R10 + R11*R6;
    double R13 = R0*R2 + R12*wwy + R9*wwx;
    double R14 = R6*wwz;
    double R15 = R1*R7;
    double R16 = R15*wwx;
    double R17 = R1*R4;
    double R18 = R17*wwy;
    double R19 = -R14 + R16 + R18;
    double R20 = R19*a1;
    double R21 = R5*R6 + R8;
    double R22 = R10*R6 - R11;
    double R23 = R2*R3 + R21*wwy + R22*wwx;
    double R24 = R23*b1;
    double R25 = R20 + R24;
    double R26 = 1 - pow(R25, 2);
    double R27 = 1.0/R26;
    double R28 = -ppz + wz;
    double R29 = R28*R6;
    double R30 = -ppx + wx;
    double R31 = R15*R30;
    double R32 = -ppy + wy;
    double R33 = R17*R32;
    double R34 = R29 - R31 - R33 + a0 - dx;
    double R35 = R34*a1;
    double R36 = R1*R28;
    double R37 = -R21*R32 - R22*R30 - R3*R36 + b0 - dy;
    double R38 = R37*b1;
    double R39 = R35 + R38;
    double R40 = -R0*R36 - R12*R32 - R30*R9 - dz;
    double R41 = R13*R40 + R19*R34 + R23*R37;
    double R42 = -R25*R39 + R41;
    double R43 = R27*R42;
    double R44 = -R13*R43 + R40;
    double R45 = R25*R41 - R35 - R38;
    double R46 = R27*R45;
    double R47 = -R19*R43 + R34 + R46*a1;
    double R48 = -R23*R43 + R37 + R46*b1;
    double R49 = 2*R27;
    double R50 = R49*(R34*(-R20 - R24) + R39*(R14 - R16 - R18));
    double R51 = 2*R25*(-2*R14 + 2*R16 + 2*R18)/pow(R26, 2);
    double R52 = R42*R51;
    double R53 = R49*(R19*R41 - R29 + R31 + R33 - a0 + dx);
    double R54 = R45*R51;
    double R55 = ((1.0/2.0)*R44*(-R13*R50 - R13*R52) + (1.0/2.0)*R47*(-R19*R50 - R19*R52 + 2*R46 + R53*a1 + R54*a1) + (1.0/2.0)*R48*(-R23*R50 - R23*R52 + R53*b1 + R54*b1))/sqrt(pow(R44, 2) + pow(R47, 2) + pow(R48, 2));
    double result = ((R43 > 0) ? (
   R55
)
: (
   -R55
));
    return result;
}


double rigidbodyalign_doca_deriv_b1(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = cos(a);
    double R1 = cos(b);
    double R2 = R1*wwz;
    double R3 = sin(a);
    double R4 = sin(g);
    double R5 = R3*R4;
    double R6 = sin(b);
    double R7 = cos(g);
    double R8 = R0*R7;
    double R9 = R5 + R6*R8;
    double R10 = R3*R7;
    double R11 = R0*R4;
    double R12 = -R10 + R11*R6;
    double R13 = R0*R2 + R12*wwy + R9*wwx;
    double R14 = R1*R7;
    double R15 = R1*R4;
    double R16 = R14*wwx + R15*wwy - R6*wwz;
    double R17 = R16*a1;
    double R18 = R2*R3;
    double R19 = R5*R6 + R8;
    double R20 = R19*wwy;
    double R21 = R10*R6 - R11;
    double R22 = R21*wwx;
    double R23 = R18 + R20 + R22;
    double R24 = R23*b1;
    double R25 = R17 + R24;
    double R26 = 1 - pow(R25, 2);
    double R27 = 1.0/R26;
    double R28 = -ppz + wz;
    double R29 = -ppx + wx;
    double R30 = -ppy + wy;
    double R31 = -R14*R29 - R15*R30 + R28*R6 + a0 - dx;
    double R32 = R31*a1;
    double R33 = R1*R28;
    double R34 = R3*R33;
    double R35 = R19*R30;
    double R36 = R21*R29;
    double R37 = -R34 - R35 - R36 + b0 - dy;
    double R38 = R37*b1;
    double R39 = R32 + R38;
    double R40 = -R0*R33 - R12*R30 - R29*R9 - dz;
    double R41 = R13*R40 + R16*R31 + R23*R37;
    double R42 = -R25*R39 + R41;
    double R43 = R27*R42;
    double R44 = -R13*R43 + R40;
    double R45 = R25*R41 - R32 - R38;
    double R46 = R27*R45;
    double R47 = -R16*R43 + R31 + R46*a1;
    double R48 = -R23*R43 + R37 + R46*b1;
    double R49 = 2*R27;
    double R50 = R49*(R37*(-R17 - R24) + R39*(-R18 - R20 - R22));
    double R51 = 2*R25*(2*R18 + 2*R20 + 2*R22)/pow(R26, 2);
    double R52 = R42*R51;
    double R53 = R49*(R23*R41 + R34 + R35 + R36 - b0 + dy);
    double R54 = R45*R51;
    double R55 = ((1.0/2.0)*R44*(-R13*R50 - R13*R52) + (1.0/2.0)*R47*(-R16*R50 - R16*R52 + R53*a1 + R54*a1) + (1.0/2.0)*R48*(-R23*R50 - R23*R52 + 2*R46 + R53*b1 + R54*b1))/sqrt(pow(R44, 2) + pow(R47, 2) + pow(R48, 2));
    double result = ((R43 > 0) ? (
   R55
)
: (
   -R55
));
    return result;
}


double rigidbodyalign_doca_deriv_dx(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = cos(a);
    double R1 = cos(b);
    double R2 = R1*wwz;
    double R3 = sin(a);
    double R4 = sin(g);
    double R5 = R3*R4;
    double R6 = sin(b);
    double R7 = cos(g);
    double R8 = R0*R7;
    double R9 = R5 + R6*R8;
    double R10 = R3*R7;
    double R11 = R0*R4;
    double R12 = -R10 + R11*R6;
    double R13 = R0*R2 + R12*wwy + R9*wwx;
    double R14 = R6*wwz;
    double R15 = R1*R7;
    double R16 = R15*wwx;
    double R17 = R1*R4;
    double R18 = R17*wwy;
    double R19 = -R14 + R16 + R18;
    double R20 = R19*a1;
    double R21 = R5*R6 + R8;
    double R22 = R10*R6 - R11;
    double R23 = R2*R3 + R21*wwy + R22*wwx;
    double R24 = R23*b1;
    double R25 = R20 + R24;
    double R26 = 1.0/(1 - pow(R25, 2));
    double R27 = -ppz + wz;
    double R28 = -ppx + wx;
    double R29 = -ppy + wy;
    double R30 = -R15*R28 - R17*R29 + R27*R6 + a0 - dx;
    double R31 = R30*a1;
    double R32 = R1*R27;
    double R33 = -R21*R29 - R22*R28 - R3*R32 + b0 - dy;
    double R34 = R33*b1;
    double R35 = -R0*R32 - R12*R29 - R28*R9 - dz;
    double R36 = R13*R35 + R19*R30 + R23*R33;
    double R37 = R26*(-R25*(R31 + R34) + R36);
    double R38 = -R13*R37 + R35;
    double R39 = R26*(R25*R36 - R31 - R34);
    double R40 = -R19*R37 + R30 + R39*a1;
    double R41 = -R23*R37 + R33 + R39*b1;
    double R42 = R14 - R16 - R18;
    double R43 = R42 - a1*(-R20 - R24);
    double R44 = 2*R26;
    double R45 = R44*(R25*R42 + a1);
    double R46 = R43*R44;
    double R47 = (-R13*R26*R38*R43 + (1.0/2.0)*R40*(-R19*R46 + R45*a1 - 2) + (1.0/2.0)*R41*(-R23*R46 + R45*b1))/sqrt(pow(R38, 2) + pow(R40, 2) + pow(R41, 2));
    double result = ((R37 > 0) ? (
   R47
)
: (
   -R47
));
    return result;
}


double rigidbodyalign_doca_deriv_dy(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = cos(a);
    double R1 = cos(b);
    double R2 = R1*wwz;
    double R3 = sin(a);
    double R4 = sin(g);
    double R5 = R3*R4;
    double R6 = sin(b);
    double R7 = cos(g);
    double R8 = R0*R7;
    double R9 = R5 + R6*R8;
    double R10 = R3*R7;
    double R11 = R0*R4;
    double R12 = -R10 + R11*R6;
    double R13 = R0*R2 + R12*wwy + R9*wwx;
    double R14 = R1*R7;
    double R15 = R1*R4;
    double R16 = R14*wwx + R15*wwy - R6*wwz;
    double R17 = R16*a1;
    double R18 = R2*R3;
    double R19 = R5*R6 + R8;
    double R20 = R19*wwy;
    double R21 = R10*R6 - R11;
    double R22 = R21*wwx;
    double R23 = R18 + R20 + R22;
    double R24 = R23*b1;
    double R25 = R17 + R24;
    double R26 = 1.0/(1 - pow(R25, 2));
    double R27 = -ppz + wz;
    double R28 = -ppx + wx;
    double R29 = -ppy + wy;
    double R30 = -R14*R28 - R15*R29 + R27*R6 + a0 - dx;
    double R31 = R30*a1;
    double R32 = R1*R27;
    double R33 = -R19*R29 - R21*R28 - R3*R32 + b0 - dy;
    double R34 = R33*b1;
    double R35 = -R0*R32 - R12*R29 - R28*R9 - dz;
    double R36 = R13*R35 + R16*R30 + R23*R33;
    double R37 = R26*(-R25*(R31 + R34) + R36);
    double R38 = -R13*R37 + R35;
    double R39 = R26*(R25*R36 - R31 - R34);
    double R40 = -R16*R37 + R30 + R39*a1;
    double R41 = -R23*R37 + R33 + R39*b1;
    double R42 = -R18 - R20 - R22;
    double R43 = R42 - b1*(-R17 - R24);
    double R44 = 2*R26;
    double R45 = R44*(R25*R42 + b1);
    double R46 = R43*R44;
    double R47 = (-R13*R26*R38*R43 + (1.0/2.0)*R40*(-R16*R46 + R45*a1) + (1.0/2.0)*R41*(-R23*R46 + R45*b1 - 2))/sqrt(pow(R38, 2) + pow(R40, 2) + pow(R41, 2));
    double result = ((R37 > 0) ? (
   R47
)
: (
   -R47
));
    return result;
}


double rigidbodyalign_doca_deriv_dz(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = cos(a);
    double R1 = cos(b);
    double R2 = R1*wwz;
    double R3 = R0*R2;
    double R4 = sin(a);
    double R5 = sin(g);
    double R6 = R4*R5;
    double R7 = sin(b);
    double R8 = cos(g);
    double R9 = R0*R8;
    double R10 = R6 + R7*R9;
    double R11 = R10*wwx;
    double R12 = R4*R8;
    double R13 = R0*R5;
    double R14 = -R12 + R13*R7;
    double R15 = R14*wwy;
    double R16 = R11 + R15 + R3;
    double R17 = R1*R8;
    double R18 = R1*R5;
    double R19 = R17*wwx + R18*wwy - R7*wwz;
    double R20 = R6*R7 + R9;
    double R21 = R12*R7 - R13;
    double R22 = R2*R4 + R20*wwy + R21*wwx;
    double R23 = R19*a1 + R22*b1;
    double R24 = 1.0/(1 - pow(R23, 2));
    double R25 = -ppz + wz;
    double R26 = -ppx + wx;
    double R27 = -ppy + wy;
    double R28 = -R17*R26 - R18*R27 + R25*R7 + a0 - dx;
    double R29 = R28*a1;
    double R30 = R1*R25;
    double R31 = -R20*R27 - R21*R26 - R30*R4 + b0 - dy;
    double R32 = R31*b1;
    double R33 = -R0*R30 - R10*R26 - R14*R27 - dz;
    double R34 = R16*R33 + R19*R28 + R22*R31;
    double R35 = R24*(-R23*(R29 + R32) + R34);
    double R36 = -R16*R35 + R33;
    double R37 = R24*(R23*R34 - R29 - R32);
    double R38 = -R19*R35 + R28 + R37*a1;
    double R39 = -R22*R35 + R31 + R37*b1;
    double R40 = 2*R24*(-R11 - R15 - R3);
    double R41 = R23*R40;
    double R42 = ((1.0/2.0)*R36*(-R16*R40 - 2) + (1.0/2.0)*R38*(-R19*R40 + R41*a1) + (1.0/2.0)*R39*(-R22*R40 + R41*b1))/sqrt(pow(R36, 2) + pow(R38, 2) + pow(R39, 2));
    double result = ((R35 > 0) ? (
   R42
)
: (
   -R42
));
    return result;
}


double rigidbodyalign_doca_deriv_a(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = cos(a);
    double R1 = cos(b);
    double R2 = R1*wwz;
    double R3 = sin(a);
    double R4 = sin(g);
    double R5 = R3*R4;
    double R6 = sin(b);
    double R7 = cos(g);
    double R8 = R0*R7;
    double R9 = R5 + R6*R8;
    double R10 = R3*R7;
    double R11 = R0*R4;
    double R12 = -R10 + R11*R6;
    double R13 = R0*R2 + R12*wwy + R9*wwx;
    double R14 = R1*R7;
    double R15 = R1*R4;
    double R16 = R14*wwx + R15*wwy - R6*wwz;
    double R17 = R16*a1;
    double R18 = R2*R3;
    double R19 = R5*R6;
    double R20 = R19 + R8;
    double R21 = R10*R6;
    double R22 = -R11 + R21;
    double R23 = R18 + R20*wwy + R22*wwx;
    double R24 = R23*b1;
    double R25 = R17 + R24;
    double R26 = 1 - pow(R25, 2);
    double R27 = 1.0/R26;
    double R28 = -ppz + wz;
    double R29 = -ppx + wx;
    double R30 = -ppy + wy;
    double R31 = -R14*R29 - R15*R30 + R28*R6 + a0 - dx;
    double R32 = R31*a1;
    double R33 = R1*R28;
    double R34 = -R20*R30 - R22*R29 - R3*R33 + b0 - dy;
    double R35 = R34*b1;
    double R36 = R32 + R35;
    double R37 = -R0*R33 - R12*R30 - R29*R9 - dz;
    double R38 = R13*R37 + R16*R31 + R23*R34;
    double R39 = -R25*R36 + R38;
    double R40 = R27*R39;
    double R41 = R13*R40;
    double R42 = R37 - R41;
    double R43 = R25*R38 - R32 - R35;
    double R44 = R27*R43;
    double R45 = -R16*R40 + R31 + R44*a1;
    double R46 = -R23*R40 + R34 + R44*b1;
    double R47 = R1*(ppz - wz);
    double R48 = R3*R47;
    double R49 = ppx - wx;
    double R50 = R11 - R21;
    double R51 = R49*R50;
    double R52 = ppy - wy;
    double R53 = -R19 - R8;
    double R54 = R52*R53;
    double R55 = -R18 + R50*wwx + R53*wwy;
    double R56 = 4*R25/pow(R26, 2);
    double R57 = R39*R56;
    double R58 = R0*R47;
    double R59 = R49*R9;
    double R60 = R12*R52;
    double R61 = R58 + R59 + R60;
    double R62 = R61*b1;
    double R63 = R13*b1;
    double R64 = R13*R34 + R13*(-R48 + R51 + R54) + R23*R61 + R37*R55;
    double R65 = 2*R27;
    double R66 = R65*(-R36*R63 + R62*(-R17 - R24) + R64);
    double R67 = R43*R56;
    double R68 = R65*(R25*R64 + R38*R63 - R62);
    double R69 = ((1.0/2.0)*R42*(-pow(R13, 2)*R57*b1 - R13*R66 - 2*R40*R55 - 2*R48 + 2*R51 + 2*R54) + (1.0/2.0)*R45*(-R16*R57*R63 - R16*R66 + R63*R67*a1 + R68*a1) + (1.0/2.0)*R46*(-R13*R24*R57 + R13*R67*pow(b1, 2) - R23*R66 - 2*R41 + 2*R58 + 2*R59 + 2*R60 + R68*b1))/sqrt(pow(R42, 2) + pow(R45, 2) + pow(R46, 2));
    double result = ((R40 > 0) ? (
   R69
)
: (
   -R69
));
    return result;
}


double rigidbodyalign_doca_deriv_b(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = cos(a);
    double R1 = cos(b);
    double R2 = R1*wwz;
    double R3 = sin(a);
    double R4 = sin(g);
    double R5 = R3*R4;
    double R6 = sin(b);
    double R7 = cos(g);
    double R8 = R0*R7;
    double R9 = R5 + R6*R8;
    double R10 = R3*R7;
    double R11 = R0*R4;
    double R12 = -R10 + R11*R6;
    double R13 = R0*R2 + R12*wwy + R9*wwx;
    double R14 = R6*wwz;
    double R15 = R1*R7;
    double R16 = R1*R4;
    double R17 = -R14 + R15*wwx + R16*wwy;
    double R18 = R17*a1;
    double R19 = R5*R6 + R8;
    double R20 = R10*R6 - R11;
    double R21 = R19*wwy + R2*R3 + R20*wwx;
    double R22 = R21*b1;
    double R23 = R18 + R22;
    double R24 = 1 - pow(R23, 2);
    double R25 = 1.0/R24;
    double R26 = -ppz + wz;
    double R27 = -ppx + wx;
    double R28 = -ppy + wy;
    double R29 = -R15*R27 - R16*R28 + R26*R6 + a0 - dx;
    double R30 = R29*a1;
    double R31 = R1*R26;
    double R32 = -R19*R28 - R20*R27 - R3*R31 + b0 - dy;
    double R33 = R32*b1;
    double R34 = R30 + R33;
    double R35 = -R0*R31 - R12*R28 - R27*R9 - dz;
    double R36 = R13*R35 + R17*R29 + R21*R32;
    double R37 = -R23*R34 + R36;
    double R38 = R25*R37;
    double R39 = -R13*R38 + R35;
    double R40 = R23*R36 - R30 - R33;
    double R41 = R25*R40;
    double R42 = -R17*R38 + R29 + R41*a1;
    double R43 = -R21*R38 + R32 + R41*b1;
    double R44 = R6*(ppz - wz);
    double R45 = R0*R44;
    double R46 = ppx - wx;
    double R47 = R1*R8;
    double R48 = R46*R47;
    double R49 = ppy - wy;
    double R50 = R1*R11;
    double R51 = R49*R50;
    double R52 = -R0*R14 + R47*wwx + R50*wwy;
    double R53 = 2*R38;
    double R54 = R6*R7;
    double R55 = R4*R6;
    double R56 = -R2 - R54*wwx - R55*wwy;
    double R57 = R56*a1;
    double R58 = R1*R10;
    double R59 = R1*R5;
    double R60 = -R14*R3 + R58*wwx + R59*wwy;
    double R61 = R60*b1;
    double R62 = 2*R23*(2*R57 + 2*R61)/pow(R24, 2);
    double R63 = R37*R62;
    double R64 = R46*R54;
    double R65 = R49*R55;
    double R66 = R31 - R64 - R65;
    double R67 = R66*a1;
    double R68 = R3*R44;
    double R69 = R46*R58;
    double R70 = R49*R59;
    double R71 = -R68 + R69 + R70;
    double R72 = R71*b1;
    double R73 = R13*(-R45 + R48 + R51) + R17*R66 + R21*R71 + R29*R56 + R32*R60 + R35*R52;
    double R74 = 2*R25;
    double R75 = R74*(R34*(-R57 - R61) + R73 + (-R18 - R22)*(R67 + R72));
    double R76 = R40*R62;
    double R77 = R74*(R23*R73 + R36*(R57 + R61) - R67 - R72);
    double R78 = ((1.0/2.0)*R39*(-R13*R63 - R13*R75 - 2*R45 + 2*R48 + 2*R51 - R52*R53) + (1.0/2.0)*R42*(-R17*R63 - R17*R75 + 2*R31 - R53*R56 - 2*R64 - 2*R65 + R76*a1 + R77*a1) + (1.0/2.0)*R43*(-R21*R63 - R21*R75 - R53*R60 - 2*R68 + 2*R69 + 2*R70 + R76*b1 + R77*b1))/sqrt(pow(R39, 2) + pow(R42, 2) + pow(R43, 2));
    double result = ((R38 > 0) ? (
   R78
)
: (
   -R78
));
    return result;
}


double rigidbodyalign_doca_deriv_g(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = cos(a);
    double R1 = cos(b);
    double R2 = R1*wwz;
    double R3 = sin(a);
    double R4 = sin(g);
    double R5 = R3*R4;
    double R6 = sin(b);
    double R7 = cos(g);
    double R8 = R0*R7;
    double R9 = R5 + R6*R8;
    double R10 = R3*R7;
    double R11 = R0*R4;
    double R12 = R11*R6;
    double R13 = -R10 + R12;
    double R14 = R0*R2 + R13*wwy + R9*wwx;
    double R15 = R1*R7;
    double R16 = R1*R4;
    double R17 = R15*wwx + R16*wwy - R6*wwz;
    double R18 = R17*a1;
    double R19 = R5*R6;
    double R20 = R19 + R8;
    double R21 = R10*R6 - R11;
    double R22 = R2*R3 + R20*wwy + R21*wwx;
    double R23 = R22*b1;
    double R24 = R18 + R23;
    double R25 = 1 - pow(R24, 2);
    double R26 = 1.0/R25;
    double R27 = -ppz + wz;
    double R28 = -ppx + wx;
    double R29 = -ppy + wy;
    double R30 = -R15*R28 - R16*R29 + R27*R6 + a0 - dx;
    double R31 = R30*a1;
    double R32 = R1*R27;
    double R33 = -R20*R29 - R21*R28 - R3*R32 + b0 - dy;
    double R34 = R33*b1;
    double R35 = R31 + R34;
    double R36 = -R0*R32 - R13*R29 - R28*R9 - dz;
    double R37 = R14*R36 + R17*R30 + R22*R33;
    double R38 = -R24*R35 + R37;
    double R39 = R26*R38;
    double R40 = -R14*R39 + R36;
    double R41 = R24*R37 - R31 - R34;
    double R42 = R26*R41;
    double R43 = -R17*R39 + R30 + R42*a1;
    double R44 = -R22*R39 + R33 + R42*b1;
    double R45 = ppy - wy;
    double R46 = R45*R9;
    double R47 = ppx - wx;
    double R48 = R10 - R12;
    double R49 = R47*R48;
    double R50 = R48*wwx + R9*wwy;
    double R51 = 2*R39;
    double R52 = R15*wwy - R16*wwx;
    double R53 = R52*a1;
    double R54 = -R19 - R8;
    double R55 = R21*wwy + R54*wwx;
    double R56 = R55*b1;
    double R57 = 2*R24*(2*R53 + 2*R56)/pow(R25, 2);
    double R58 = R38*R57;
    double R59 = R15*R45;
    double R60 = R16*R47;
    double R61 = R59 - R60;
    double R62 = R61*a1;
    double R63 = R21*R45;
    double R64 = R47*R54;
    double R65 = R63 + R64;
    double R66 = R65*b1;
    double R67 = R14*(R46 + R49) + R17*R61 + R22*R65 + R30*R52 + R33*R55 + R36*R50;
    double R68 = 2*R26;
    double R69 = R68*(R35*(-R53 - R56) + R67 + (-R18 - R23)*(R62 + R66));
    double R70 = R41*R57;
    double R71 = R68*(R24*R67 + R37*(R53 + R56) - R62 - R66);
    double R72 = ((1.0/2.0)*R40*(-R14*R58 - R14*R69 + 2*R46 + 2*R49 - R50*R51) + (1.0/2.0)*R43*(-R17*R58 - R17*R69 - R51*R52 + 2*R59 - 2*R60 + R70*a1 + R71*a1) + (1.0/2.0)*R44*(-R22*R58 - R22*R69 - R51*R55 + 2*R63 + 2*R64 + R70*b1 + R71*b1))/sqrt(pow(R40, 2) + pow(R43, 2) + pow(R44, 2));
    double result = ((R39 > 0) ? (
   R72
)
: (
   -R72
));
    return result;
}


std::vector<double> RigidBodyDOCADerivatives(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz));
{
    std::vector<double> result;

    result.push_back(rigidbodyalign_doca_deriv_a0(a0,b0,a1,b1,dx,dy,dz,a,b,g,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_b0(a0,b0,a1,b1,dx,dy,dz,a,b,g,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_a1(a0,b0,a1,b1,dx,dy,dz,a,b,g,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_b1(a0,b0,a1,b1,dx,dy,dz,a,b,g,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_dx(a0,b0,a1,b1,dx,dy,dz,a,b,g,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_dy(a0,b0,a1,b1,dx,dy,dz,a,b,g,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_dz(a0,b0,a1,b1,dx,dy,dz,a,b,g,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_a(a0,b0,a1,b1,dx,dy,dz,a,b,g,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_b(a0,b0,a1,b1,dx,dy,dz,a,b,g,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));
result.push_back(rigidbodyalign_doca_deriv_g(a0,b0,a1,b1,dx,dy,dz,a,b,g,wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz));

    return result;
}
