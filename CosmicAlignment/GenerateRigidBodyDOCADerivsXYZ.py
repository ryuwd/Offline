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

def exact_alignment(X, wire_pos, wire_dir, body_origin, translation, a, b, g):
    R_a = AxisOrienter(a, X.i).rotation_matrix(X)
    R_b = AxisOrienter(b, X.j).rotation_matrix(X)
    R_g = AxisOrienter(g, X.k).rotation_matrix(X)

    aligned_wire_pos = body_origin + matrix_to_vector((R_a * R_b * R_g * (wire_pos - body_origin).to_matrix(X)),X) + translation
    aligned_wire_dir = matrix_to_vector((R_a * R_b * R_g * (wire_dir).to_matrix(X)),X) # direction unaffected by translation of plane/panel

    return aligned_wire_pos, aligned_wire_dir

def small_alignment_approximation(X, wire_pos, wire_dir, body_origin, translation, a, b, g):
    # for small a, b, g (corrections to nominal rotation transforms)
    # we can approximate (according to Millepede documentation)

    # the overall rotation matrix to:
    # 1   g  -b
    # -g  1   a
    #  b -a   1
    # Thanks to: https://www.desy.de/~kleinwrt/MP2/doc/html/draftman_page.html (See: Linear Transformations in 3D)

    R_abg_approx = Matrix([[ 1, g, b],
                           [-g, 1, a],
                           [ b,-a, 1]])

    aligned_wire_pos = body_origin + matrix_to_vector((R_abg_approx * (wire_pos - body_origin).to_matrix(X)),X) + translation
    aligned_wire_dir = matrix_to_vector((R_abg_approx * (wire_dir).to_matrix(X)),X)

    return aligned_wire_pos, aligned_wire_dir

c_template = """

#include "CosmicAlignment/inc/RigidBodyDOCADeriv.hh"
#include <math.h>
#include <vector>

%s

"""

h_template = """
#ifndef RIGIDBODYDOCADERIV_H
#define RIGIDBODYDOCADERIV_H
#include <vector>
%s

#endif

"""

deriv_calc_fn_template = """

std::vector<float> RigidBodyDOCADerivatives_%s(%s)
{
    std::vector<float> result;

%s

    return result;
}

"""

fn_template = "%s %s(%s);"

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

def main(approximate=False, remove_globalparam_dependence = True):
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

    alignment_func = exact_alignment
    if approximate:
        alignment_func = small_alignment_approximation

    # recalculate wire position and rotation according to alignment parameters
    aligned_wpos, aligned_wdir = alignment_func(X, wire_pos, wire_dir, body_origin, trl, a, b, g)

    aligned_doca = DOCA(track_pos, track_dir, aligned_wpos, aligned_wdir)

    # now generate optimised C code to calculate each deriv

    local_params = [a0,b0,a1,b1]
    global_params = [dx,dy,dz,a,b,g]

    all_params = local_params + global_params + [wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz]
    if remove_globalparam_dependence:
        all_params = local_params + [wx,wy,wz,wwx,wwy,wwz,ppx,ppy,ppz]

    code = []
    functions=[]
    args = 'double ' + (', double '.join([p.name for p in all_params]))

    for parameter in local_params + global_params:
        pdev = diff(aligned_doca, parameter)

        if remove_globalparam_dependence:
            # since these derivatives are intended for use BEFORE
            # any corrections are applied, we substitute a,b,g,dx,dy,dz to equal zero
            # in our final expressions
            pdev = pdev.subs({
                dx:0,
                dy:0,
                dz:0,
                a:0,
                b:0,
                g:0
            })


        fnname = 'rigidbodyalign_doca_deriv_%s' % parameter.name
        code.append(cseexpr_to_ccode(fnname, pdev, all_params))
        functions.append(fn_template % ('double', fnname, args))

    # generate helper functions
    functions.append(fn_template %('std::vector<float>', "RigidBodyDOCADerivatives_local", args))
    functioncalls=[]

    for parameter in local_params:
        functioncalls.append('result.push_back(%s(%s));' % ('rigidbodyalign_doca_deriv_%s' % parameter.name, ','.join([p.name for p in all_params])))
    code.append(deriv_calc_fn_template % ('local', args, '\n'.join(functioncalls)))

    functions.append(fn_template %('std::vector<float>', "RigidBodyDOCADerivatives_global", args))
    functioncalls=[]
    for parameter in global_params:
        functioncalls.append('result.push_back(%s(%s));' % ('rigidbodyalign_doca_deriv_%s' % parameter.name, ','.join([p.name for p in all_params])))
    code.append(deriv_calc_fn_template % ('global', args, '\n'.join(functioncalls)))

    # pull everything together and write to file

    c_code = c_template % ('\n\n'.join(code))
    c_header = h_template % '\n\n'.join(functions)

    with open('src/RigidBodyDOCADeriv.cc', 'w') as f:
        f.write(c_code)

    with open('inc/RigidBodyDOCADeriv.hh', 'w') as f:
        f.write(c_header)

if __name__ == "__main__":
    main(approximate=True)