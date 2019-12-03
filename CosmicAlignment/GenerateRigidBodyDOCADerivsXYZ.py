# Ryunosuke O'Neil, 2019
# Symbolic derivation of DOCA partial derivatives with respect to alignment and track params

import sympy
from sympy import Symbol, Matrix, diff, sqrt
from sympy.physics.vector import ReferenceFrame
from sympy.vector import matrix_to_vector, CoordSys3D
from sympy.functions import sign

from sympy.vector.orienters import AxisOrienter
from sympy.simplify.cse_main import cse
from sympy.utilities.codegen import codegen, Routine
from sympy.printing import ccode
from sympy.utilities.iterables import numbered_symbols
from sympy.functions import Abs
from sympy.matrices.dense import matrix_multiply_elementwise

def unit_vector(v):
    tot2 = v.dot(v)
    return v/sqrt(tot2) # sympy.Piecewise((1 / sqrt(tot2), tot2 > 0), (1, True)) * v

# Based on TwoLinePCAXYZ.
def DOCA(p1, t1, p2, t2):
    t1 = unit_vector(t1)
    #t2 = unit_vector(t2) t2 should already be a unit vector

    c = t2.dot(t1)

    sinsq = 1 - c*c
    _delta = p1 - p2
    ddotT1 = _delta.dot(t1)
    ddotT2 = _delta.dot(t2)

    _s1 =  (ddotT2*c-ddotT1)/sinsq
    _s2 = -(ddotT1*c-ddotT2)/sinsq

    _pca1 = p1 + t1 * _s1
    _pca2 = p2 + t2 * _s2

    ___diff = _pca1 - _pca2

    dca = sqrt(___diff.dot(___diff))

    return sympy.Piecewise((dca, _s2 > 0), (-dca, True))


def exact_alignment(wire_pos, wire_dir, body_origin, translation, a, b, g):
    X = CoordSys3D('X')
    R_a = AxisOrienter(a, X.i).rotation_matrix(X)
    R_b = AxisOrienter(b, X.j).rotation_matrix(X)
    R_g = AxisOrienter(g, X.k).rotation_matrix(X)

    aligned_wire_pos = body_origin + (R_a * R_b * R_g * (wire_pos - body_origin)) + translation
    aligned_wire_dir = (R_a * R_b * R_g * wire_dir) # direction unaffected by translation of plane/panel

    return aligned_wire_pos, aligned_wire_dir

def small_alignment_approximation(wire_pos, wire_dir, body_origin, translation, a, b, g):
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

    aligned_wire_pos = (body_origin + (R_abg_approx * (wire_pos - body_origin))) + translation
    aligned_wire_dir = R_abg_approx * wire_dir

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
    # define symbols for alignment and track params as column vectors

    # rotation angles
    a = Symbol('a', real=True)
    b = Symbol('b', real=True)
    g = Symbol('g', real=True)

    # translation vector
    dx = Symbol('dx', real=True)
    dy = Symbol('dy', real=True)
    dz = Symbol('dz', real=True)
    trl = Matrix([dx, dy, dz]) # dx * X.x + dy * X.y + dz * X.z

    # track parametrisation
    a0 = Symbol('a0', real=True)
    b0 = Symbol('b0', real=True)
    track_pos = Matrix([a0,b0,0]) # a0 * X.x + b0 * X.y + 0 * X.z

    a1 = Symbol('a1', real=True)
    b1 = Symbol('b1', real=True)
    track_dir = Matrix([a1,b1,1]) # a1 * X.x + b1 * X.y + 1 * X.z
    # t0 = Symbol('t0')

    # wire position (midpoint) and direction
    wx = Symbol('wx', real=True)
    wy = Symbol('wy', real=True)
    wz = Symbol('wz', real=True)
    wire_pos = Matrix([wx,wy,wz]) # wx * X.x + wy * X.y + wz * X.z

    wwx = Symbol('wwx', real=True)
    wwy = Symbol('wwy', real=True)
    wwz = Symbol('wwz', real=True)
    wire_dir = Matrix([wwx,wwy,wwz]) # wwx * X.x + wwy * X.y + wwz * X.z

    # origin of the rigid body being aligned
    ppx = Symbol('ppx', real=True)
    ppy = Symbol('ppy', real=True)
    ppz = Symbol('ppz', real=True)
    body_origin = Matrix([ppx,ppy,ppz]) # ppx * X.x + ppy * X.y + ppz * X.z


    local_params = [a0,b0,a1,b1]
    global_params = [dx,dy,dz,a,b,g]
    wire_params = [wx,wy,wz,wwx,wwy,wwz]
    plane_params = [ppx,ppy,ppz]
    all_params = local_params + global_params + wire_params + plane_params

    # choose method to align vectors with
    alignment_func = exact_alignment
    if approximate:
        alignment_func = small_alignment_approximation

    # recalculate wire position and rotation according to alignment parameters
    aligned_wpos, aligned_wdir = alignment_func(wire_pos, wire_dir, body_origin, trl, a, b, g)
    aligned_doca = DOCA(track_pos, track_dir, aligned_wpos, aligned_wdir)

    # now generate optimised C code to calculate each deriv
    if remove_globalparam_dependence:
        all_params = local_params + wire_params + plane_params

    code = []
    functions = []
    args = 'double ' + (', double '.join([p.name for p in all_params]))

    for parameter in local_params + global_params:
        # calculate derivative symbolically for each local and global parameter then generate code for the function
        pdev = diff(aligned_doca, parameter)

        if remove_globalparam_dependence:
            # since these derivatives are intended for use BEFORE
            # any corrections are applied, we substitute zero for
            # a, b, g, dx, dy, dz in our final expressions
            pdev = pdev.subs({
                dx:0,dy:0,dz:0,
                a:0,b:0,g:0})

        fnname = 'rigidbodyalign_doca_deriv_%s' % parameter.name
        code.append(cseexpr_to_ccode(fnname, pdev, all_params))
        functions.append(fn_template % ('double', fnname, args))

    # generate helper function (DOCA)
    argsd = 'double ' + (', double '.join([p.name for p in local_params+[wx,wy,wz,wwx,wwy,wwz]]))
    code.append(cseexpr_to_ccode('RigidBodyDOCADerivatives_DOCAfn', DOCA(track_pos, track_dir, wire_pos, wire_dir), local_params+[wx,wy,wz,wwx,wwy,wwz]))
    functions.append(fn_template %('double', 'RigidBodyDOCADerivatives_DOCAfn', argsd))

    # function to build arrays of calculated derivatives (local only)
    functions.append(fn_template %('std::vector<float>', "RigidBodyDOCADerivatives_local", args))
    functioncalls = []
    for parameter in local_params:
        functioncalls.append('result.push_back(%s(%s));' % ('rigidbodyalign_doca_deriv_%s' % parameter.name, ','.join([p.name for p in all_params])))
    code.append(deriv_calc_fn_template % ('local', args, '\n'.join(functioncalls)))

    # (global derivatives)
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