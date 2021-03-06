#
#    This file is part of awebox.
#
#    awebox -- A modeling and optimization framework for multi-kite AWE systems.
#    Copyright (C) 2017-2020 Jochem De Schutter, Rachel Leuthold, Moritz Diehl,
#                            ALU Freiburg.
#    Copyright (C) 2018-2020 Thilo Bronnenmeyer, Kiteswarms Ltd.
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
constraints code of the awebox
takes model inequalities, generates constraint structures, and defines the nlp constraints
python-3.5 / casadi-3.4.5
- refactored from awebox code (elena malz, chalmers; jochem de schutter, alu-fr; rachel leuthold, alu-fr), 2018
- edited: jochem de schutter, rachel leuthold, alu-fr 2018 - 2019
'''

import casadi.tools as cas
import numpy as np
import awebox.mdl.aero.induction_dir.vortex_dir.fixing as vortex_fix
import awebox.mdl.aero.induction_dir.vortex_dir.strength as vortex_strength

import awebox.ocp.operation as operation
import awebox.ocp.ocp_constraint as ocp_constraint

import awebox.tools.print_operations as print_op
import awebox.tools.struct_operations as struct_op
import awebox.tools.constraint_operations as cstr_op
import awebox.tools.performance_operations as perf_op

from awebox.logger.logger import Logger as awelogger

import copy

def get_constraints(nlp_options, V, P, Xdot, model, dae, formulation, Integral_constraint_list, Collocation, Multiple_shooting, ms_z0, ms_xf, ms_vars, ms_params, Outputs):

    awelogger.logger.info('generate constraints...')

    direct_collocation = (nlp_options['discretization'] == 'direct_collocation')
    multiple_shooting = (nlp_options['discretization'] == 'multiple_shooting')

    ocp_cstr_list = ocp_constraint.OcpConstraintList()
    ocp_cstr_entry_list = []

    # model constraints structs
    mdl_path_constraints = model.constraints_dict['inequality']
    mdl_dyn_constraints = model.constraints_dict['equality']
    
    # get discretization information
    nk = nlp_options['n_k']
    
    # size of algebraic variables on interval nodes
    nz = model.variables['xa'].shape[0]
    if 'xl' in list(model.variables.keys()):
        nz += model.variables['xl'].shape[0]

    # add initial constraints
    var_initial = struct_op.get_variables_at_time(nlp_options, V, Xdot, model.variables, 0)
    var_ref_initial = struct_op.get_var_ref_at_time(nlp_options, P, V, Xdot, model, 0)
    init_cstr = operation.get_initial_constraints(nlp_options, var_initial, var_ref_initial, model, formulation.xi_dict)
    ocp_cstr_list.append(init_cstr)
    if len(init_cstr.eq_list) != 0:
        ocp_cstr_entry_list.append(cas.entry('initial', shape = init_cstr.get_expression_list('all').shape))

    # entry tuple for nested constraints
    entry_tuple = ()
    entry_tuple += (cas.entry('dynamics', repeat = [nk], struct = model.variables_dict['xd']),)
    entry_tuple += (cas.entry('algebraic', repeat = [nk], shape = (nz,1)),)

    # add the path constraints.
    if multiple_shooting:
        ms_cstr = expand_with_multiple_shooting(nlp_options, V, model, dae, Multiple_shooting, ms_z0, ms_xf, ms_vars, ms_params)
        ocp_cstr_list.append(ms_cstr)
        entry_tuple += (cas.entry('path', repeat = [nk], struct = mdl_path_constraints),)

    elif direct_collocation:
        coll_cstr = expand_with_collocation(nlp_options, P, V, Xdot, model, Collocation)
        ocp_cstr_list.append(coll_cstr)
        d = nlp_options['collocation']['d']
        entry_tuple += (
            cas.entry('path',        repeat = [nk],    struct = mdl_path_constraints),
            cas.entry('collocation', repeat = [nk, d], struct = mdl_dyn_constraints),
            )

    else:
        message = 'unexpected ocp discretization method selected: ' + nlp_options['discretization']
        awelogger.logger.error(message)
        raise Exception(message)

    # continuity constraints
    entry_tuple += (
        cas.entry('continuity', repeat = [nk], struct = model.variables_dict['xd']),
    )

    # add stage and continuity constraints to list
    ocp_cstr_entry_list.append(entry_tuple)

    # add terminal constraints
    var_terminal = struct_op.get_variables_at_final_time(nlp_options, V, Xdot, model)
    var_ref_terminal = struct_op.get_var_ref_at_final_time(nlp_options, P, V, Xdot, model)
    terminal_cstr = operation.get_terminal_constraints(nlp_options, var_terminal, var_ref_terminal, model, formulation.xi_dict)
    ocp_cstr_list.append(terminal_cstr)
    if len(terminal_cstr.eq_list) != 0:
        ocp_cstr_entry_list.append(cas.entry('terminal', shape =  terminal_cstr.get_expression_list('all').shape))

    # add periodic constraints
    periodic_cstr = operation.get_periodic_constraints(nlp_options, var_initial, var_terminal)
    ocp_cstr_list.append(periodic_cstr)
    if len(periodic_cstr.eq_list) != 0:
        ocp_cstr_entry_list.append(cas.entry('periodic', shape =  periodic_cstr.get_expression_list('all').shape))

    vortex_fixing_cstr = vortex_fix.get_fixing_constraint(nlp_options, V, Outputs, model)
    ocp_cstr_list.append(vortex_fixing_cstr)
    if len(vortex_fixing_cstr.eq_list) != 0:
        ocp_cstr_entry_list.append(cas.entry('vortex_fix', shape =  vortex_fixing_cstr.get_expression_list('all').shape))

    vortex_strength_cstr = vortex_strength.get_strength_constraint(nlp_options, V, Outputs, model)
    ocp_cstr_list.append(vortex_strength_cstr)
    if len(vortex_strength_cstr.eq_list) != 0:
        ocp_cstr_entry_list.append(cas.entry('vortex_strength', shape =  vortex_strength_cstr.get_expression_list('all').shape))

    if direct_collocation:
        integral_cstr = get_integral_constraints(Integral_constraint_list, formulation.integral_constants)
        ocp_cstr_list.append(integral_cstr)
        if len(integral_cstr.eq_list) != 0:
            ocp_cstr_entry_list.append(cas.entry('integral', shape=integral_cstr.get_expression_list('all').shape))

    # Constraints structure
    ocp_cstr_struct = cas.struct_symSX(ocp_cstr_entry_list)(ocp_cstr_list.get_expression_list('all'))

    return ocp_cstr_list, ocp_cstr_struct


def expand_with_collocation(nlp_options, P, V, Xdot, model, Collocation):

    cstr_list = ocp_constraint.OcpConstraintList()

    model_variables = model.variables
    model_parameters = model.parameters
    model_constraints_list = model.constraints_list

    n_k = nlp_options['n_k']
    d = nlp_options['collocation']['d']

    parallellization = nlp_options['parallelization']['type']

    # collect shooting variables
    shooting_nodes = struct_op.count_shooting_nodes(nlp_options)
    shooting_vars = struct_op.get_shooting_vars(nlp_options, V, P, Xdot, model)
    shooting_params = struct_op.get_shooting_params(nlp_options, V, P, model)

    # collect collocation variables
    coll_vars = struct_op.get_coll_vars(nlp_options, V, P, Xdot, model)
    coll_params = struct_op.get_coll_params(nlp_options, V, P, model)

    # create maps of relevant functions
    mdl_ineq_fun = model_constraints_list.get_function(nlp_options, model_variables, model_parameters, 'ineq')
    mdl_ineq_map = mdl_ineq_fun.map('mdl_ineq_map', parallellization, shooting_nodes, [], [])

    mdl_eq_fun = model_constraints_list.get_function(nlp_options, model_variables, model_parameters, 'eq')
    mdl_eq_map = mdl_eq_fun.map('mdl_eq_map', parallellization, n_k * d, [], [])
    mdl_eq_map_shooting = mdl_eq_fun.map('mdl_eq_map', parallellization, shooting_nodes, [], [])

    # evaluate constraint functions
    ocp_ineqs_expr = mdl_ineq_map(shooting_vars, shooting_params)
    ocp_eqs_expr = mdl_eq_map(coll_vars, coll_params)
    ocp_eqs_shooting_expr = mdl_eq_map_shooting(shooting_vars, shooting_params)

    # sort constraints to obtain desired sparsity structure
    for kdx in range(n_k):

        # algebraic constraints on shooting nodes
        cstr_list.append(cstr_op.Constraint(
            expr = ocp_eqs_shooting_expr[:,kdx],
            name = 'algebraic_{}'.format(kdx),
            cstr_type = 'eq'
            )
        )

        # path constraints on shooting nodes
        cstr_list.append(cstr_op.Constraint(
            expr= ocp_ineqs_expr[:,kdx],
            name='path_{}'.format(kdx),
            cstr_type='ineq'
            )
        )

        # collocation constraints
        for jdx in range(d):
            cstr_list.append(cstr_op.Constraint(
                expr = ocp_eqs_expr[:,kdx*d+jdx],
                name = 'collocation_{}_{}'.format(kdx,jdx),
                cstr_type = 'eq'
                )
            )

        # continuity constraints
        cstr_list.append(Collocation.get_continuity_constraint(V, kdx))

    return cstr_list

def expand_with_multiple_shooting(nlp_options, V, model, dae, Multiple_shooting, ms_z0, ms_xf, ms_vars, ms_params):

    cstr_list = ocp_constraint.OcpConstraintList()

    model_variables = model.variables
    model_parameters = model.parameters
    model_constraints_list = model.constraints_list

    n_k = nlp_options['n_k']

    parallellization = nlp_options['parallelization']['type']

    # algebraic constraints
    z_from_V = []
    for kdx in range(n_k):
        local_z_from_V = cas.vertcat(V['xddot',kdx], V['xa', kdx])
        if 'xl' in V.keys():
            local_z_from_V = cas.vertcat(local_z_from_V, V['xl', kdx])
        z_from_V = cas.horzcat(z_from_V, local_z_from_V)

    z_at_time_sym = cas.MX.sym('z_at_time_sym',dae.z.shape)
    z_from_V_sym =  cas.MX.sym('z_from_V_sym',dae.z.shape)
    alg_fun = cas.Function('alg_fun', [z_at_time_sym, z_from_V_sym], [z_at_time_sym - z_from_V_sym])
    alg_map = alg_fun.map('alg_map', parallellization, n_k, [], [])

    # inequality constraints
    mdl_ineq_fun = model_constraints_list.get_function(nlp_options, model_variables, model_parameters, 'ineq')
    mdl_ineq_map = mdl_ineq_fun.map('mdl_ineq_map', parallellization, n_k, [], [])

    # evaluate mapped constraint functions
    alg_expr = alg_map(ms_z0, z_from_V)
    ocp_ineqs_expr = mdl_ineq_map(ms_vars, ms_params)

    for kdx in range(n_k):

        # algebraic constraints
        cstr_list.append(cstr_op.Constraint(
            expr = alg_expr[:,kdx],
            name = 'algebraic_{}'.format(kdx),
            cstr_type = 'eq'
            )
        )

        # path constraints
        cstr_list.append(cstr_op.Constraint(
            expr = ocp_ineqs_expr[:,kdx],
            name = 'path_{}'.format(kdx),
            cstr_type = 'ineq'
            )
        )

        # continuity constraints
        cstr_list.append(Multiple_shooting.get_continuity_constraint(ms_xf, V, kdx))

    return cstr_list

def get_integral_constraints(integral_list, integral_constants):

    cstr_list = ocp_constraint.OcpConstraintList()

    # nu = V['phi','nu']
    integral_sum = {}

    for key_name in list(integral_constants.keys()):
        integral_t0 = integral_constants[key_name]
        integral_sum[key_name] = 0.
        for i in range(len(integral_list)):
            integral_sum[key_name] += integral_list[i][key_name]

        expr = (- integral_t0 - integral_sum[key_name]) / integral_t0
        cstr_type = translate_cstr_type(key_name)

        g_cstr = cstr_op.Constraint(expr=expr,
                                    name='integral_' + key_name,
                                    cstr_type=cstr_type)
        cstr_list.append(g_cstr)

    return cstr_list


def translate_cstr_type(constraint_type):

    # convention h(w) <= 0
    if constraint_type == 'inequality':
        return 'ineq'
    elif constraint_type == 'equality':
        return 'eq'
    else:
        raise ValueError('Wrong constraint type chosen. Possible values: "inequality" / "equality" ')

    return None

