

#include "CosmicAlignment/inc/RigidBodyDOCADeriv.hh"
#include <math.h>
#include <vector>
double rigidbodyalign_doca_deriv_a0(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = -ppz + wz;
    double R1 = cos(a);
    double R2 = cos(b);
    double R3 = R1*R2;
    double R4 = -ppx + wx;
    double R5 = sin(a);
    double R6 = sin(g);
    double R7 = R5*R6;
    double R8 = sin(b);
    double R9 = cos(g);
    double R10 = R1*R9;
    double R11 = R10*R8 + R7;
    double R12 = -ppy + wy;
    double R13 = R5*R9;
    double R14 = R1*R6;
    double R15 = -R13 + R14*R8;
    double R16 = R11*wwx + R15*wwy + R3*wwz;
    double R17 = R2*R9;
    double R18 = R2*R6;
    double R19 = R17*wwx + R18*wwy - R8*wwz;
    double R20 = R19*a1;
    double R21 = R2*R5;
    double R22 = R10 + R7*R8;
    double R23 = R13*R8 - R14;
    double R24 = R21*wwz + R22*wwy + R23*wwx;
    double R25 = R24*b1;
    double R26 = R20 + R25;
    double R27 = 1.0/(1 - pow(R26, 2));
    double R28 = R0*R8 - R12*R18 - R17*R4 + a0 - dx;
    double R29 = R28*a1;
    double R30 = -R0*R21 - R12*R22 - R23*R4 + b0 - dy;
    double R31 = R30*b1;
    double R32 = R29 + R31;
    double R33 = R26*R32;
    double R34 = R27*(R32 - R33);
    double R35 = -R0*R3 - R11*R4 - R12*R15 - R16*R34 - dz;
    double R36 = R27*(-R29 - R31 + R33);
    double R37 = -R19*R34 + R28 + R36*a1;
    double R38 = -R24*R34 + R30 + R36*b1;
    double R39 = a1*(-R20 - R25) + a1;
    double R40 = 2*R27;
    double R41 = R40*(R26*a1 - a1);
    double R42 = R39*R40;
    double R43 = (-R16*R27*R35*R39 + (1.0/2.0)*R37*(-R19*R42 + R41*a1 + 2) + (1.0/2.0)*R38*(-R24*R42 + R41*b1))/sqrt(pow(R35, 2) + pow(R37, 2) + pow(R38, 2));
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
    double R0 = -ppz + wz;
    double R1 = cos(a);
    double R2 = cos(b);
    double R3 = R1*R2;
    double R4 = -ppx + wx;
    double R5 = sin(a);
    double R6 = sin(g);
    double R7 = R5*R6;
    double R8 = sin(b);
    double R9 = cos(g);
    double R10 = R1*R9;
    double R11 = R10*R8 + R7;
    double R12 = -ppy + wy;
    double R13 = R5*R9;
    double R14 = R1*R6;
    double R15 = -R13 + R14*R8;
    double R16 = R11*wwx + R15*wwy + R3*wwz;
    double R17 = R2*R9;
    double R18 = R2*R6;
    double R19 = R17*wwx + R18*wwy - R8*wwz;
    double R20 = R19*a1;
    double R21 = R2*R5;
    double R22 = R10 + R7*R8;
    double R23 = R13*R8 - R14;
    double R24 = R21*wwz + R22*wwy + R23*wwx;
    double R25 = R24*b1;
    double R26 = R20 + R25;
    double R27 = 1.0/(1 - pow(R26, 2));
    double R28 = R0*R8 - R12*R18 - R17*R4 + a0 - dx;
    double R29 = R28*a1;
    double R30 = -R0*R21 - R12*R22 - R23*R4 + b0 - dy;
    double R31 = R30*b1;
    double R32 = R29 + R31;
    double R33 = R26*R32;
    double R34 = R27*(R32 - R33);
    double R35 = -R0*R3 - R11*R4 - R12*R15 - R16*R34 - dz;
    double R36 = R27*(-R29 - R31 + R33);
    double R37 = -R19*R34 + R28 + R36*a1;
    double R38 = -R24*R34 + R30 + R36*b1;
    double R39 = b1*(-R20 - R25) + b1;
    double R40 = 2*R27;
    double R41 = R40*(R26*b1 - b1);
    double R42 = R39*R40;
    double R43 = (-R16*R27*R35*R39 + (1.0/2.0)*R37*(-R19*R42 + R41*a1) + (1.0/2.0)*R38*(-R24*R42 + R41*b1 + 2))/sqrt(pow(R35, 2) + pow(R37, 2) + pow(R38, 2));
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
    double R0 = -ppz + wz;
    double R1 = cos(a);
    double R2 = cos(b);
    double R3 = R1*R2;
    double R4 = -ppx + wx;
    double R5 = sin(a);
    double R6 = sin(g);
    double R7 = R5*R6;
    double R8 = sin(b);
    double R9 = cos(g);
    double R10 = R1*R9;
    double R11 = R10*R8 + R7;
    double R12 = -ppy + wy;
    double R13 = R5*R9;
    double R14 = R1*R6;
    double R15 = -R13 + R14*R8;
    double R16 = R11*wwx + R15*wwy + R3*wwz;
    double R17 = R8*wwz;
    double R18 = R2*R9;
    double R19 = R18*wwx;
    double R20 = R2*R6;
    double R21 = R20*wwy;
    double R22 = -R17 + R19 + R21;
    double R23 = R22*a1;
    double R24 = R2*R5;
    double R25 = R10 + R7*R8;
    double R26 = R13*R8 - R14;
    double R27 = R24*wwz + R25*wwy + R26*wwx;
    double R28 = R27*b1;
    double R29 = R23 + R28;
    double R30 = 1 - pow(R29, 2);
    double R31 = 1.0/R30;
    double R32 = R0*R8;
    double R33 = R18*R4;
    double R34 = R12*R20;
    double R35 = R32 - R33 - R34 + a0 - dx;
    double R36 = R35*a1;
    double R37 = -R0*R24 - R12*R25 - R26*R4 + b0 - dy;
    double R38 = R37*b1;
    double R39 = R36 + R38;
    double R40 = R29*R39;
    double R41 = R39 - R40;
    double R42 = R31*R41;
    double R43 = -R0*R3 - R11*R4 - R12*R15 - R16*R42 - dz;
    double R44 = -R36 - R38 + R40;
    double R45 = R31*R44;
    double R46 = -R22*R42 + R35 + R45*a1;
    double R47 = -R27*R42 + R37 + R45*b1;
    double R48 = 2*R31;
    double R49 = R48*(R35*(-R23 - R28) + R35 + R39*(R17 - R19 - R21));
    double R50 = 2*R29*(-2*R17 + 2*R19 + 2*R21)/pow(R30, 2);
    double R51 = R41*R50;
    double R52 = R48*(R22*R39 + R29*R35 - R32 + R33 + R34 - a0 + dx);
    double R53 = R44*R50;
    double R54 = ((1.0/2.0)*R43*(-R16*R49 - R16*R51) + (1.0/2.0)*R46*(-R22*R49 - R22*R51 + 2*R45 + R52*a1 + R53*a1) + (1.0/2.0)*R47*(-R27*R49 - R27*R51 + R52*b1 + R53*b1))/sqrt(pow(R43, 2) + pow(R46, 2) + pow(R47, 2));
    double result = ((R42 > 0) ? (
   R54
)
: (
   -R54
));
    return result;
}


double rigidbodyalign_doca_deriv_b1(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = -ppz + wz;
    double R1 = cos(a);
    double R2 = cos(b);
    double R3 = R1*R2;
    double R4 = -ppx + wx;
    double R5 = sin(a);
    double R6 = sin(g);
    double R7 = R5*R6;
    double R8 = sin(b);
    double R9 = cos(g);
    double R10 = R1*R9;
    double R11 = R10*R8 + R7;
    double R12 = -ppy + wy;
    double R13 = R5*R9;
    double R14 = R1*R6;
    double R15 = -R13 + R14*R8;
    double R16 = R11*wwx + R15*wwy + R3*wwz;
    double R17 = R2*R9;
    double R18 = R2*R6;
    double R19 = R17*wwx + R18*wwy - R8*wwz;
    double R20 = R19*a1;
    double R21 = R2*R5;
    double R22 = R21*wwz;
    double R23 = R10 + R7*R8;
    double R24 = R23*wwy;
    double R25 = R13*R8 - R14;
    double R26 = R25*wwx;
    double R27 = R22 + R24 + R26;
    double R28 = R27*b1;
    double R29 = R20 + R28;
    double R30 = 1 - pow(R29, 2);
    double R31 = 1.0/R30;
    double R32 = R0*R8 - R12*R18 - R17*R4 + a0 - dx;
    double R33 = R32*a1;
    double R34 = R0*R21;
    double R35 = R12*R23;
    double R36 = R25*R4;
    double R37 = -R34 - R35 - R36 + b0 - dy;
    double R38 = R37*b1;
    double R39 = R33 + R38;
    double R40 = R29*R39;
    double R41 = R39 - R40;
    double R42 = R31*R41;
    double R43 = -R0*R3 - R11*R4 - R12*R15 - R16*R42 - dz;
    double R44 = -R33 - R38 + R40;
    double R45 = R31*R44;
    double R46 = -R19*R42 + R32 + R45*a1;
    double R47 = -R27*R42 + R37 + R45*b1;
    double R48 = 2*R31;
    double R49 = R48*(R37*(-R20 - R28) + R37 + R39*(-R22 - R24 - R26));
    double R50 = 2*R29*(2*R22 + 2*R24 + 2*R26)/pow(R30, 2);
    double R51 = R41*R50;
    double R52 = R48*(R27*R39 + R29*R37 + R34 + R35 + R36 - b0 + dy);
    double R53 = R44*R50;
    double R54 = ((1.0/2.0)*R43*(-R16*R49 - R16*R51) + (1.0/2.0)*R46*(-R19*R49 - R19*R51 + R52*a1 + R53*a1) + (1.0/2.0)*R47*(-R27*R49 - R27*R51 + 2*R45 + R52*b1 + R53*b1))/sqrt(pow(R43, 2) + pow(R46, 2) + pow(R47, 2));
    double result = ((R42 > 0) ? (
   R54
)
: (
   -R54
));
    return result;
}


double rigidbodyalign_doca_deriv_dx(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = -ppz + wz;
    double R1 = cos(a);
    double R2 = cos(b);
    double R3 = R1*R2;
    double R4 = -ppx + wx;
    double R5 = sin(a);
    double R6 = sin(g);
    double R7 = R5*R6;
    double R8 = sin(b);
    double R9 = cos(g);
    double R10 = R1*R9;
    double R11 = R10*R8 + R7;
    double R12 = -ppy + wy;
    double R13 = R5*R9;
    double R14 = R1*R6;
    double R15 = -R13 + R14*R8;
    double R16 = R11*wwx + R15*wwy + R3*wwz;
    double R17 = R2*R9;
    double R18 = R2*R6;
    double R19 = R17*wwx + R18*wwy - R8*wwz;
    double R20 = R19*a1;
    double R21 = R2*R5;
    double R22 = R10 + R7*R8;
    double R23 = R13*R8 - R14;
    double R24 = R21*wwz + R22*wwy + R23*wwx;
    double R25 = R24*b1;
    double R26 = R20 + R25;
    double R27 = 1.0/(1 - pow(R26, 2));
    double R28 = R0*R8 - R12*R18 - R17*R4 + a0 - dx;
    double R29 = R28*a1;
    double R30 = -R0*R21 - R12*R22 - R23*R4 + b0 - dy;
    double R31 = R30*b1;
    double R32 = R29 + R31;
    double R33 = R26*R32;
    double R34 = R27*(R32 - R33);
    double R35 = -R0*R3 - R11*R4 - R12*R15 - R16*R34 - dz;
    double R36 = R27*(-R29 - R31 + R33);
    double R37 = -R19*R34 + R28 + R36*a1;
    double R38 = -R24*R34 + R30 + R36*b1;
    double R39 = -a1*(-R20 - R25) - a1;
    double R40 = 2*R27;
    double R41 = R40*(-R26*a1 + a1);
    double R42 = R39*R40;
    double R43 = (-R16*R27*R35*R39 + (1.0/2.0)*R37*(-R19*R42 + R41*a1 - 2) + (1.0/2.0)*R38*(-R24*R42 + R41*b1))/sqrt(pow(R35, 2) + pow(R37, 2) + pow(R38, 2));
    double result = ((R34 > 0) ? (
   R43
)
: (
   -R43
));
    return result;
}


double rigidbodyalign_doca_deriv_dy(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = -ppz + wz;
    double R1 = cos(a);
    double R2 = cos(b);
    double R3 = R1*R2;
    double R4 = -ppx + wx;
    double R5 = sin(a);
    double R6 = sin(g);
    double R7 = R5*R6;
    double R8 = sin(b);
    double R9 = cos(g);
    double R10 = R1*R9;
    double R11 = R10*R8 + R7;
    double R12 = -ppy + wy;
    double R13 = R5*R9;
    double R14 = R1*R6;
    double R15 = -R13 + R14*R8;
    double R16 = R11*wwx + R15*wwy + R3*wwz;
    double R17 = R2*R9;
    double R18 = R2*R6;
    double R19 = R17*wwx + R18*wwy - R8*wwz;
    double R20 = R19*a1;
    double R21 = R2*R5;
    double R22 = R10 + R7*R8;
    double R23 = R13*R8 - R14;
    double R24 = R21*wwz + R22*wwy + R23*wwx;
    double R25 = R24*b1;
    double R26 = R20 + R25;
    double R27 = 1.0/(1 - pow(R26, 2));
    double R28 = R0*R8 - R12*R18 - R17*R4 + a0 - dx;
    double R29 = R28*a1;
    double R30 = -R0*R21 - R12*R22 - R23*R4 + b0 - dy;
    double R31 = R30*b1;
    double R32 = R29 + R31;
    double R33 = R26*R32;
    double R34 = R27*(R32 - R33);
    double R35 = -R0*R3 - R11*R4 - R12*R15 - R16*R34 - dz;
    double R36 = R27*(-R29 - R31 + R33);
    double R37 = -R19*R34 + R28 + R36*a1;
    double R38 = -R24*R34 + R30 + R36*b1;
    double R39 = -b1*(-R20 - R25) - b1;
    double R40 = 2*R27;
    double R41 = R40*(-R26*b1 + b1);
    double R42 = R39*R40;
    double R43 = (-R16*R27*R35*R39 + (1.0/2.0)*R37*(-R19*R42 + R41*a1) + (1.0/2.0)*R38*(-R24*R42 + R41*b1 - 2))/sqrt(pow(R35, 2) + pow(R37, 2) + pow(R38, 2));
    double result = ((R34 > 0) ? (
   R43
)
: (
   -R43
));
    return result;
}


double rigidbodyalign_doca_deriv_dz(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = -ppz + wz;
    double R1 = cos(a);
    double R2 = cos(b);
    double R3 = R1*R2;
    double R4 = R0*R3;
    double R5 = -ppx + wx;
    double R6 = sin(a);
    double R7 = sin(g);
    double R8 = R6*R7;
    double R9 = sin(b);
    double R10 = cos(g);
    double R11 = R1*R10;
    double R12 = R11*R9 + R8;
    double R13 = R12*R5;
    double R14 = -ppy + wy;
    double R15 = R10*R6;
    double R16 = R1*R7;
    double R17 = -R15 + R16*R9;
    double R18 = R14*R17;
    double R19 = R10*R2;
    double R20 = R2*R7;
    double R21 = R19*wwx + R20*wwy - R9*wwz;
    double R22 = R2*R6;
    double R23 = R11 + R8*R9;
    double R24 = R15*R9 - R16;
    double R25 = R22*wwz + R23*wwy + R24*wwx;
    double R26 = R21*a1 + R25*b1;
    double R27 = 1.0/(1 - pow(R26, 2));
    double R28 = R0*R9 - R14*R20 - R19*R5 + a0 - dx;
    double R29 = R28*a1;
    double R30 = -R0*R22 - R14*R23 - R24*R5 + b0 - dy;
    double R31 = R30*b1;
    double R32 = R29 + R31;
    double R33 = R26*R32;
    double R34 = R27*(R32 - R33);
    double R35 = R34*(R12*wwx + R17*wwy + R3*wwz);
    double R36 = R27*(-R29 - R31 + R33);
    double R37 = (R13 + R18 + R35 + R4 + dz)/sqrt(pow(-R21*R34 + R28 + R36*a1, 2) + pow(-R25*R34 + R30 + R36*b1, 2) + pow(-R13 - R18 - R35 - R4 - dz, 2));
    double result = ((R34 > 0) ? (
   R37
)
: (
   -R37
));
    return result;
}


double rigidbodyalign_doca_deriv_a(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = -ppz + wz;
    double R1 = cos(a);
    double R2 = cos(b);
    double R3 = R1*R2;
    double R4 = -ppx + wx;
    double R5 = sin(a);
    double R6 = sin(g);
    double R7 = R5*R6;
    double R8 = sin(b);
    double R9 = cos(g);
    double R10 = R1*R9;
    double R11 = R10*R8 + R7;
    double R12 = -ppy + wy;
    double R13 = R5*R9;
    double R14 = R1*R6;
    double R15 = -R13 + R14*R8;
    double R16 = R11*wwx + R15*wwy + R3*wwz;
    double R17 = R2*R9;
    double R18 = R2*R6;
    double R19 = R17*wwx + R18*wwy - R8*wwz;
    double R20 = R19*a1;
    double R21 = R2*R5;
    double R22 = R21*wwz;
    double R23 = R7*R8;
    double R24 = R10 + R23;
    double R25 = R13*R8;
    double R26 = -R14 + R25;
    double R27 = R22 + R24*wwy + R26*wwx;
    double R28 = R27*b1;
    double R29 = R20 + R28;
    double R30 = 1 - pow(R29, 2);
    double R31 = 1.0/R30;
    double R32 = R0*R8 - R12*R18 - R17*R4 + a0 - dx;
    double R33 = R32*a1;
    double R34 = -R0*R21 - R12*R24 - R26*R4 + b0 - dy;
    double R35 = R34*b1;
    double R36 = R33 + R35;
    double R37 = R29*R36;
    double R38 = R36 - R37;
    double R39 = R31*R38;
    double R40 = R16*R39;
    double R41 = -R0*R3 - R11*R4 - R12*R15 - R40 - dz;
    double R42 = -R33 - R35 + R37;
    double R43 = R31*R42;
    double R44 = -R19*R39 + R32 + R43*a1;
    double R45 = -R27*R39 + R34 + R43*b1;
    double R46 = ppz - wz;
    double R47 = ppx - wx;
    double R48 = R14 - R25;
    double R49 = ppy - wy;
    double R50 = -R10 - R23;
    double R51 = R3*R46;
    double R52 = R11*R47;
    double R53 = R15*R49;
    double R54 = b1*(R51 + R52 + R53);
    double R55 = R16*b1;
    double R56 = R36*R55;
    double R57 = 2*R31;
    double R58 = R57*(R54*(-R20 - R28) + R54 - R56);
    double R59 = 4*R29/pow(R30, 2);
    double R60 = R38*R59;
    double R61 = R57*(R29*R54 - R54 + R56);
    double R62 = R42*R59;
    double R63 = ((1.0/2.0)*R41*(-pow(R16, 2)*R60*b1 - R16*R58 - 2*R21*R46 - 2*R39*(-R22 + R48*wwx + R50*wwy) + 2*R47*R48 + 2*R49*R50) + (1.0/2.0)*R44*(-R19*R55*R60 - R19*R58 + R55*R62*a1 + R61*a1) + (1.0/2.0)*R45*(-R16*R28*R60 + R16*R62*pow(b1, 2) - R27*R58 - 2*R40 + 2*R51 + 2*R52 + 2*R53 + R61*b1))/sqrt(pow(R41, 2) + pow(R44, 2) + pow(R45, 2));
    double result = ((R39 > 0) ? (
   R63
)
: (
   -R63
));
    return result;
}


double rigidbodyalign_doca_deriv_b(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = cos(a);
    double R1 = cos(b);
    double R2 = -ppz + wz;
    double R3 = R1*R2;
    double R4 = -ppx + wx;
    double R5 = sin(a);
    double R6 = sin(g);
    double R7 = R5*R6;
    double R8 = sin(b);
    double R9 = cos(g);
    double R10 = R0*R9;
    double R11 = R10*R8 + R7;
    double R12 = -ppy + wy;
    double R13 = R5*R9;
    double R14 = R0*R6;
    double R15 = -R13 + R14*R8;
    double R16 = R1*wwz;
    double R17 = R0*R16 + R11*wwx + R15*wwy;
    double R18 = R8*wwz;
    double R19 = R1*R9;
    double R20 = R1*R6;
    double R21 = -R18 + R19*wwx + R20*wwy;
    double R22 = R21*a1;
    double R23 = R10 + R7*R8;
    double R24 = R13*R8 - R14;
    double R25 = R16*R5 + R23*wwy + R24*wwx;
    double R26 = R25*b1;
    double R27 = R22 + R26;
    double R28 = 1 - pow(R27, 2);
    double R29 = 1.0/R28;
    double R30 = -R12*R20 - R19*R4 + R2*R8 + a0 - dx;
    double R31 = R30*a1;
    double R32 = -R12*R23 - R24*R4 - R3*R5 + b0 - dy;
    double R33 = R32*b1;
    double R34 = R31 + R33;
    double R35 = R27*R34;
    double R36 = R34 - R35;
    double R37 = R29*R36;
    double R38 = -R0*R3 - R11*R4 - R12*R15 - R17*R37 - dz;
    double R39 = -R31 - R33 + R35;
    double R40 = R29*R39;
    double R41 = -R21*R37 + R30 + R40*a1;
    double R42 = -R25*R37 + R32 + R40*b1;
    double R43 = R8*(ppz - wz);
    double R44 = ppx - wx;
    double R45 = R1*R44;
    double R46 = ppy - wy;
    double R47 = R1*R46;
    double R48 = R1*wwx;
    double R49 = R1*wwy;
    double R50 = 2*R37;
    double R51 = R8*R9;
    double R52 = R44*R51;
    double R53 = R6*R8;
    double R54 = R46*R53;
    double R55 = a1*(R3 - R52 - R54);
    double R56 = R43*R5;
    double R57 = R13*R45;
    double R58 = R47*R7;
    double R59 = b1*(-R56 + R57 + R58);
    double R60 = R55 + R59;
    double R61 = -R16 - R51*wwx - R53*wwy;
    double R62 = R61*a1;
    double R63 = R13*R48 - R18*R5 + R49*R7;
    double R64 = R63*b1;
    double R65 = 2*R29;
    double R66 = R65*(R34*(-R62 - R64) + R60*(-R22 - R26) + R60);
    double R67 = 2*R27*(2*R62 + 2*R64)/pow(R28, 2);
    double R68 = R36*R67;
    double R69 = R65*(R27*R60 + R34*(R62 + R64) - R55 - R59);
    double R70 = R39*R67;
    double R71 = ((1.0/2.0)*R38*(-2*R0*R43 + 2*R10*R45 + 2*R14*R47 - R17*R66 - R17*R68 - R50*(-R0*R18 + R10*R48 + R14*R49)) + (1.0/2.0)*R41*(-R21*R66 - R21*R68 + 2*R3 - R50*R61 - 2*R52 - 2*R54 + R69*a1 + R70*a1) + (1.0/2.0)*R42*(-R25*R66 - R25*R68 - R50*R63 - 2*R56 + 2*R57 + 2*R58 + R69*b1 + R70*b1))/sqrt(pow(R38, 2) + pow(R41, 2) + pow(R42, 2));
    double result = ((R37 > 0) ? (
   R71
)
: (
   -R71
));
    return result;
}


double rigidbodyalign_doca_deriv_g(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    double R0 = -ppz + wz;
    double R1 = cos(a);
    double R2 = cos(b);
    double R3 = R1*R2;
    double R4 = -ppx + wx;
    double R5 = sin(a);
    double R6 = sin(g);
    double R7 = R5*R6;
    double R8 = sin(b);
    double R9 = cos(g);
    double R10 = R1*R9;
    double R11 = R10*R8 + R7;
    double R12 = -ppy + wy;
    double R13 = R5*R9;
    double R14 = R1*R6;
    double R15 = R14*R8;
    double R16 = -R13 + R15;
    double R17 = R11*wwx + R16*wwy + R3*wwz;
    double R18 = R2*R9;
    double R19 = R2*R6;
    double R20 = R18*wwx + R19*wwy - R8*wwz;
    double R21 = R20*a1;
    double R22 = R2*R5;
    double R23 = R7*R8;
    double R24 = R10 + R23;
    double R25 = R13*R8 - R14;
    double R26 = R22*wwz + R24*wwy + R25*wwx;
    double R27 = R26*b1;
    double R28 = R21 + R27;
    double R29 = 1 - pow(R28, 2);
    double R30 = 1.0/R29;
    double R31 = R0*R8 - R12*R19 - R18*R4 + a0 - dx;
    double R32 = R31*a1;
    double R33 = -R0*R22 - R12*R24 - R25*R4 + b0 - dy;
    double R34 = R33*b1;
    double R35 = R32 + R34;
    double R36 = R28*R35;
    double R37 = R35 - R36;
    double R38 = R30*R37;
    double R39 = -R0*R3 - R11*R4 - R12*R16 - R17*R38 - dz;
    double R40 = -R32 - R34 + R36;
    double R41 = R30*R40;
    double R42 = -R20*R38 + R31 + R41*a1;
    double R43 = -R26*R38 + R33 + R41*b1;
    double R44 = ppy - wy;
    double R45 = 2*R44;
    double R46 = R13 - R15;
    double R47 = ppx - wx;
    double R48 = 2*R47;
    double R49 = 2*R38;
    double R50 = a1*(R18*R44 - R19*R47);
    double R51 = R25*R44;
    double R52 = -R10 - R23;
    double R53 = R47*R52;
    double R54 = b1*(R51 + R53);
    double R55 = R50 + R54;
    double R56 = R18*wwy - R19*wwx;
    double R57 = R56*a1;
    double R58 = R25*wwy + R52*wwx;
    double R59 = R58*b1;
    double R60 = 2*R30;
    double R61 = R60*(R35*(-R57 - R59) + R55*(-R21 - R27) + R55);
    double R62 = 2*R28*(2*R57 + 2*R59)/pow(R29, 2);
    double R63 = R37*R62;
    double R64 = R60*(R28*R55 + R35*(R57 + R59) - R50 - R54);
    double R65 = R40*R62;
    double R66 = ((1.0/2.0)*R39*(R11*R45 - R17*R61 - R17*R63 + R46*R48 - R49*(R11*wwy + R46*wwx)) + (1.0/2.0)*R42*(R18*R45 - R19*R48 - R20*R61 - R20*R63 - R49*R56 + R64*a1 + R65*a1) + (1.0/2.0)*R43*(-R26*R61 - R26*R63 - R49*R58 + 2*R51 + 2*R53 + R64*b1 + R65*b1))/sqrt(pow(R39, 2) + pow(R42, 2) + pow(R43, 2));
    double result = ((R38 > 0) ? (
   R66
)
: (
   -R66
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
