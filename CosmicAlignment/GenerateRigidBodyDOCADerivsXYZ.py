# Ryunosuke O'Neil, 2019
# Symbolic derivation of DOCA partial derivatives with respect to alignment and track params

import sympy
from sympy import Symbol, Matrix, diff,sqrt, Abs
from sympy.vector import CoordSys3D, matrix_to_vector
from sympy.functions import sign

from sympy.vector.orienters import AxisOrienter
from sympy.simplify.cse_main import cse
from sympy.utilities.codegen import codegen, Routine
from sympy.printing import ccode
from sympy.utilities.iterables import numbered_symbols

def DOCA(track_pos, track_dir, wire_pos, wire_dir):
    alpha = track_dir.dot(wire_dir)
    beta = 1 - alpha*alpha
    delta = track_pos - wire_pos

    ddotT1 = delta.dot(track_dir)
    ddotT2 = delta.dot(wire_dir)

    s1 = (ddotT2 * alpha - ddotT1)/beta
    s2 = -(ddotT1 * alpha - ddotT2)/beta

    pca1 = track_pos + track_dir * s1
    pca2 = wire_pos + wire_dir * s2

    diff = pca1-pca2

    doca = sqrt(diff.dot(diff))

    return sympy.Piecewise((doca, s2 > 0), (-doca, True))

def alignment(X, wire_pos, wire_dir, body_origin, translation, a, b, g):
    R_a = AxisOrienter(a, X.i).rotation_matrix(X)
    R_b = AxisOrienter(b, X.j).rotation_matrix(X)
    R_g = AxisOrienter(g, X.k).rotation_matrix(X)

    aligned_wire_pos = matrix_to_vector((R_a * R_b * R_g * (wire_pos - body_origin).to_matrix(X)) + translation.to_matrix(X),X)
    aligned_wire_dir = matrix_to_vector((R_a * R_b * R_g * (wire_dir).to_matrix(X)),X) # direction unaffected by translation of plane/panel

    return aligned_wire_pos, aligned_wire_dir


c_template = """

#include "CosmicAlignment/inc/RigidBodyDOCADeriv.hh"
#include <math.h>
#include <vector>

%s

std::vector<double> RigidBodyDOCADerivatives(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz)
{
    std::vector<double> result;

    %s

    return result;
}
"""

h_template = """
#ifndef RIGIDBODYDOCADERIV_H
#define RIGIDBODYDOCADERIV_H

%s

std::vector<double> RigidBodyDOCADerivatives(double a0, double b0, double a1, double b1, double dx, double dy, double dz, double a, double b, double g, double wx, double wy, double wz, double wwx, double wwy, double wwz, double ppx, double ppy, double ppz);

#endif

"""

fn_template = "double %s(%s);"

def cseexpr_to_ccode( symname, symfunc, symbolslist ):
    tmpsyms = numbered_symbols("R")
    symbols, simple = cse(symfunc, symbols=tmpsyms)

    code = "double %s(%s)\n" % (str(symname), ", ".join("double %s" % x for x in symbolslist))
    code +=  "{\n"
    for s in symbols:
        code +=  "    double %s = %s;\n" % (ccode(s[0]), ccode(s[1]))
    code +=  "    double result = %s;\n" % ccode(simple[0])
    code +=  "    return result;\n"
    code += "}\n"

    return code

def main():
    # define symbols for alignment and track params
    X = CoordSys3D('X') # coordinate system

    # rotation angles
    a = Symbol('a', real=True)
    b = Symbol('b', real=True)
    g = Symbol('g', real=True)

    # translation vector
    dx = Symbol('dx', real=True)
    dy = Symbol('dy', real=True)
    dz = Symbol('dz', real=True)
    trl = dx * X.i + dy * X.j + dz * X.k

    # track parametrisation
    a0 = Symbol('a0', real=True)
    a1 = Symbol('a1', real=True)
    b0 = Symbol('b0', real=True)
    b1 = Symbol('b1', real=True)
    track_pos = a0 * X.i + b0 * X.j
    track_dir = a1 * X.i + b1 * X.j
    # t0 = Symbol('t0')

    # wire position (midpoint) and direction
    wx = Symbol('wx', real=True)
    wy = Symbol('wy', real=True)
    wz = Symbol('wz', real=True)
    wire_pos = wx * X.i + wy * X.j + wz * X.k

    wwx = Symbol('wwx', real=True)
    wwy = Symbol('wwy', real=True)
    wwz = Symbol('wwz', real=True)
    wire_dir = wwx * X.i + wwy * X.j + wwz * X.k

    # origin of the rigid body being aligned
    ppx = Symbol('ppx', real=True)
    ppy = Symbol('ppy', real=True)
    ppz = Symbol('ppz', real=True)
    body_origin = ppx * X.i + ppy * X.j + ppz * X.k

    # recalculate wire position and rotation according to alignment parameters
    aligned_wpos, aligned_wdir = alignment(X, wire_pos, wire_dir, body_origin, trl, a, b, g)

    aligned_doca = DOCA(track_pos, track_dir, aligned_wpos, aligned_wdir)

    # now generate optimised C code to calculate each deriv

    local_params = [a0,b0,a1,b1]
    global_params = [dx,dy,dz,a,b,g]

    all_params = local_params + global_params + [wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz]

    code = []
    functions=[]
    functioncalls=[]
    for parameter in local_params + global_params:
        pdev = diff(aligned_doca, parameter)

        fnname = 'rigidbodyalign_doca_deriv_%s' % parameter.name
        args = 'double ' + (', double '.join([p.name for p in all_params]))
        code.append(cseexpr_to_ccode(fnname, pdev, all_params))
        functions.append(fn_template % (fnname, args))
        functioncalls.append('result.push_back(%s(%s));' % (fnname, ','.join([p.name for p in all_params])))

    c_code = c_template % ('\n\n'.join(code), '\n'.join(functioncalls))
    c_header = h_template % '\n\n'.join(functions)

    with open('src/RigidBodyDOCADeriv.cc', 'w') as f:
        f.write(c_code)

    with open('inc/RigidBodyDOCADeriv.hh', 'w') as f:
        f.write(c_header)

if __name__ == "__main__":
    main()