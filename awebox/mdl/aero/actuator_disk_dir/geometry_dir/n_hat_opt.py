#
#    This file is part of awebox.
#
#    awebox -- A modeling and optimization framework for multi-kite AWE systems.
#    Copyright (C) 2017-2019 Jochem De Schutter, Rachel Leuthold, Moritz Diehl,
#                            ALU Freiburg.
#    Copyright (C) 2018-2019 Thilo Bronnenmeyer, Kiteswarms Ltd.
#    Copyright (C) 2016      Elena Malz, Sebastien Gros, Chalmers UT.
#
#    awebox is free software; you can redistribute it and/or
#    modify it under the terms of the GNU Lesser General Public
#    License as published by the Free Software Foundation; either
#    version 3 of the License, or (at your option) any later version.
#
#    awebox is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#    Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public
#    License along with awebox; if not, write to the Free Software Foundation,
#    Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
#
#
'''
actuator_disk model of awebox aerodynamics
sets up the axial-induction actuator disk equation
currently for untilted rotor with no tcf.
_python-3.5 / casadi-3.4.5
- author: rachel leuthold, alu-fr 2017-19
- edit: jochem de schutter, alu-fr 2019
'''

import numpy as np
import casadi as cas
from awebox.logger.logger import Logger as awelogger

import awebox.tools.vector_operations as vect_op
import pdb


def get_n_vec(model_options, parent, variables, parameters, architecture):

    model = model_options['aero']['actuator']['normal_vector_model']
    children = architecture.kites_map[parent]
    number_children = float(len(children))

    if model == 'least_squares' and number_children == 3:
        n_vec = get_plane_fit_n_vec(parent, variables, parameters, architecture)

    elif model == 'least_squares' and number_children > 3:
        n_vec = get_least_squares_n_vec(parent, variables, parameters, architecture)

    elif model == 'binormal':
        n_vec = get_binormal_n_vec(parent, variables, parameters, architecture)

    elif model == 'tether_parallel' and number_children > 1:
        n_vec = get_tether_parallel_multi_n_vec(parent, variables, parameters, architecture)

    elif model == 'tether_parallel' and number_children == 1:
        n_vec = get_tether_parallel_single_n_vec(parent, variables, parameters, architecture)

    elif model == 'default' and number_children > 1:
        n_vec = get_tether_parallel_multi_n_vec(parent, variables, parameters, architecture)

    elif model == 'default' and number_children == 1:
        n_vec = get_tether_parallel_single_n_vec(parent, variables, parameters, architecture)

    else:
        awelogger.logger.warning('normal-vector model-type (for actuator disk) not supported. Consider checking the number of kites per layer.')
        n_vec = vect_op.xhat_np()

    return n_vec


def get_n_vec_default(model_options, parent, variables, parameters, architecture):

    model = model_options['aero']['actuator']['normal_vector_model']
    children = architecture.kites_map[parent]
    number_children = float(len(children))

    if number_children > 1:
        n_vec = get_tether_parallel_multi_n_vec(parent, variables, parameters, architecture)

    elif number_children == 1:
        n_vec = get_tether_parallel_single_n_vec(parent, variables, parameters, architecture)

    else:
        awelogger.logger.warning('normal-vector model-type (for actuator disk) not supported. Consider checking the number of kites per layer.')
        n_vec = vect_op.xhat_np()

    return n_vec



def get_n_hat(model_options, parent, variables, parameters, architecture):

    n_vec = get_n_vec(model_options, parent, variables, parameters, architecture)
    n_hat = vect_op.normalize(n_vec)

    return n_hat

def get_n_vec_length_var(variables, parent):
    scale = get_n_vec_length_ref(variables, parent)
    len_var = scale * variables['xl']['n_vec_length' + str(parent)]
    return len_var

def get_n_vec_length_ref(variables, parent):
    return 1.



def get_least_squares_n_vec(parent, variables, parameters, architecture):

    children = architecture.kites_map[parent]

    matrix = []
    rhs = []
    for kite in children:
        qk = variables['xd']['q' + str(kite) + str(parent)]
        newline = cas.horzcat(qk[[1]], qk[[2]], 1)
        matrix = cas.vertcat(matrix, newline)

        rhs = cas.vertcat(rhs, -qk[[0]])

    coords = cas.mtimes(cas.pinv(matrix), rhs)

    n_vec = cas.vertcat(1, coords[[0]], coords[[1]])

    return n_vec


def get_plane_fit_n_vec(parent, variables, parameters, architecture):

    children = sorted(architecture.kites_map[parent])

    kite0 = children[0]
    kite1 = children[1]
    kite2 = children[2]

    # there is a potential failure here if the order of the kite nodes is not increaseing from kite0 -> kite1 -> kite2,
    # where the direction of the cross-product flips, based on initialization where the lower number kites have a
    # smaller azimuthal angle. presumably, this should be resulved by the sorting above, but, just in case!
    if (kite0 > kite1) or (kite1 > kite2):
        awelogger.logger.warning('based on assignment order of kites, normal vector (by cross product) may point in reverse.')

    qkite0 = variables['xd']['q' + str(kite0) + str(parent)]
    qkite1 = variables['xd']['q' + str(kite1) + str(parent)]
    qkite2 = variables['xd']['q' + str(kite2) + str(parent)]

    x1 = qkite0[0]
    x2 = qkite1[0]
    x3 = qkite2[0]

    y1 = qkite0[1]
    y2 = qkite1[1]
    y3 = qkite2[1]

    z1 = qkite0[2]
    z2 = qkite1[2]
    z3 = qkite2[2]

    aa = (-y2 * z1 + y3 * z1 + y1 * z2 - y3 * z2 - y1 * z3 + y2 * z3)
    bb = ( x2 * z1 - x3 * z1 - x1 * z2 + x3 * z2 + x1 * z3 - x2 * z3)
    cc = (-x2 * y1 + x3 * y1 + x1 * y2 - x3 * y2 - x1 * y3 + x2 * y3)

    n_vec_unscale = cas.vertcat(aa, bb, cc)

    scale = 1.e3
    #
    # b_ref = parameters['theta0', 'geometry', 'b_ref']
    # varrho_temp = 7.
    #
    # scale = b_ref**2. * varrho_temp**2.
    n_vec = n_vec_unscale / scale

    return n_vec

def get_tether_parallel_multi_n_vec(parent, variables, parameters, architecture):

    grandparent = architecture.parent_map[parent]

    if grandparent == 0:
        n_vec = variables['xd']['q' + str(parent) + str(grandparent)]

        n_vec = n_vec * 1.e-3
    else:
        great_grandparent = architecture.parent_map[grandparent]
        q_parent = variables['xd']['q' + str(parent) + str(grandparent)]
        q_grandparent = variables['xd']['q' + str(grandparent) + str(great_grandparent)]
        n_vec = q_parent - q_grandparent

        n_vec = n_vec * 1.e-1

    return n_vec

def get_tether_parallel_single_n_vec(parent, variables, parameters, architecture):

    kite = architecture.children_map[parent][0]
    n_vec = variables['xd']['q' + str(kite) + str(parent)]

    n_vec = n_vec * 1.e-3

    return n_vec

def get_binormal_n_vec(parent, variables, parameters, architecture):

    children = architecture.kites_map[parent]

    n_vec = np.zeros((3,1))
    for kite in children:
        dqk = variables['xddot']['dq' + str(kite) + str(parent)]
        ddqk = variables['xddot']['ddq' + str(kite) + str(parent)]

        binormal_dim = vect_op.cross(dqk, ddqk)

        n_vec = n_vec + binormal_dim

    return n_vec
