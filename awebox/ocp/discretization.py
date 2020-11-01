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
discretization code (direct collocation or multiple shooting)
creates n_l_p variables
creates n_l_p constraints
creates n_l_p outputs
python-3.5 / casadi-3.4.5
- authors: elena malz 2016
           rachel leuthold, jochem de schutter alu-fr 2017-18
'''

import casadi.tools as cas

import awebox.ocp.ocp_constraint as ocp_constraint
import awebox.ocp.operation as operation

from . import constraints
from . import collocation
from . import multiple_shooting
from . import performance
from . import var_struct

import awebox.tools.struct_operations as struct_op

import pdb

import awebox.tools.print_operations as print_op

def construct_time_grids(nlp_options):

    time_grids = {}
    nk = nlp_options['n_k']
    if nlp_options['discretization'] == 'direct_collocation':
        direct_collocation = True
        ms = False
        d = nlp_options['collocation']['d']
        scheme = nlp_options['collocation']['scheme']
        tau_root = cas.vertcat(cas.collocation_points(d, scheme))
        tcoll = []

    elif nlp_options['discretization'] == 'multiple_shooting':
        direct_collocation = False
        ms = True
        tcoll = None

    # make symbolic time constants
    if nlp_options['phase_fix']:
        tfsym = cas.SX.sym('tfsym',2)
        nk_reelout = round(nk * nlp_options['phase_fix_reelout'])

        t_switch = tfsym[0] * nk_reelout / nk
        time_grids['t_switch'] = cas.Function('tgrid_tswitch', [tfsym], [t_switch])

    else:
        tfsym = cas.SX.sym('tfsym',1)

    # initialize
    tx = []
    tu = []

    for k in range(nk+1):

        # extract correct time constant in case of phase fix
        if nlp_options['phase_fix']:
            if k < nk_reelout:
                tf = tfsym[0]
                k0 = 0.0
                kcount = k
            else:
                tf = tfsym[1]
                k0 = nk_reelout * tfsym[0]/ tf
                kcount = k - nk_reelout
        else:
            k0 = 0.0
            kcount = k
            tf = tfsym

        # add interval timings
        tx = cas.vertcat(tx, (k0 + kcount) * tf / float(nk))
        if k < nk:
            tu = cas.vertcat(tu, (k0 + kcount) * tf / float(nk))

        # add collocation timings
        if direct_collocation and (k < nk):
            for j in range(d):
                tcoll = cas.vertcat(tcoll,(k0 + kcount + tau_root[j]) * tf / float(nk))

    if direct_collocation:
        # reshape tcoll
        tcoll = tcoll.reshape((d,nk)).T
        tx_coll = cas.vertcat(cas.horzcat(tu, tcoll).T.reshape((nk*(d+1),1)),tx[-1])

        # write out collocation grids
        time_grids['coll'] = cas.Function('tgrid_coll',[tfsym],[tcoll])
        time_grids['x_coll'] = cas.Function('tgrid_x_coll',[tfsym],[tx_coll])

    # write out interval grid
    time_grids['x'] = cas.Function('tgrid_x',[tfsym],[tx])
    time_grids['u'] = cas.Function('tgrid_u',[tfsym],[tu])


    return time_grids

def setup_nlp_cost():

    cost = cas.struct_symSX([(
        cas.entry('tracking'),
        cas.entry('u_regularisation'),
        cas.entry('ddq_regularisation'),
        cas.entry('gamma'),
        cas.entry('iota'),
        cas.entry('psi'),
        cas.entry('tau'),
        cas.entry('eta'),
        cas.entry('nu'),
        cas.entry('upsilon'),
        cas.entry('fictitious'),
        cas.entry('power'),
        cas.entry('t_f'),
        cas.entry('theta_regularisation'),
        cas.entry('nominal_landing'),
        cas.entry('compromised_battery'),
        cas.entry('transition'),
        cas.entry('slack')
    )])

    return cost


def setup_nlp_p_fix(V, model):

    # fixed system parameters
    p_fix = cas.struct_symSX([(
        cas.entry('ref', struct=V),     # tracking reference for cost function
        cas.entry('weights', struct=model.variables)  # weights for cost function
    )])

    return p_fix

def setup_nlp_p(V, model):

    cost = setup_nlp_cost()
    p_fix = setup_nlp_p_fix(V, model)

    use_vortex_linearization = 'lin' in model.parameters_dict.keys()
    if use_vortex_linearization:
        P = cas.struct_symMX([
            cas.entry('p', struct=p_fix),
            cas.entry('cost', struct=cost),
            cas.entry('theta0', struct=model.parameters_dict['theta0']),
            cas.entry('lin', struct=V)
        ])
    else:
        P = cas.struct_symMX([
            cas.entry('p',      struct = p_fix),
            cas.entry('cost',   struct = cost),
            cas.entry('theta0', struct = model.parameters_dict['theta0'])
        ])

    return P

def setup_integral_output_structure(nlp_options, integral_outputs):

    nk = nlp_options['n_k']

    entry_tuple = ()

    # interval outputs
    entry_tuple += (
        cas.entry('int_out', repeat = [nk+1], struct = integral_outputs),
    )

    if nlp_options['discretization'] == 'direct_collocation':
        d  = nlp_options['collocation']['d']
        entry_tuple += (
            cas.entry('coll_int_out', repeat = [nk,d], struct = integral_outputs),
        )

    Integral_outputs_struct = cas.struct_symMX([entry_tuple])

    return Integral_outputs_struct

def setup_output_structure(nlp_options, model_outputs, form_outputs):
    # create outputs
    # n_o_t_e: !!! outputs are defined at nodes where both state and algebraic variables
    # are defined. In the radau case, algebraic variables are not defined on the interval
    # nodes, note however that algebraic variables might be discontinuous so the same
    # holds for the outputs!!!
    nk = nlp_options['n_k']

    entry_tuple =  ()
    if nlp_options['discretization'] == 'direct_collocation':

        # extract collocation parameters
        d  = nlp_options['collocation']['d']
        scheme = nlp_options['collocation']['scheme']

        if scheme != 'radau':

            # define outputs on interval and collocation nodes
            entry_tuple += (
                cas.entry('outputs',      repeat = [nk],   struct = model_outputs),
                cas.entry('coll_outputs', repeat = [nk,d], struct = model_outputs),
            )

        else:

            # define outputs on collocation nodes
            entry_tuple += (
                cas.entry('coll_outputs', repeat = [nk,d], struct = model_outputs),
            )

    elif nlp_options['discretization'] == 'multiple_shooting':

        # define outputs on interval nodes
        entry_tuple += (
            cas.entry('outputs', repeat = [nk], struct = model_outputs),
        )

    Outputs = cas.struct_symSX([entry_tuple]
                           + [cas.entry('final', struct=form_outputs)])

    return Outputs

def discretize(nlp_options, model, formulation):

    # -----------------------------------------------------------------------------
    # discretization setup
    # -----------------------------------------------------------------------------
    nk = nlp_options['n_k']

    if nlp_options['discretization'] == 'direct_collocation':
        direct_collocation = True
        ms = False
        d = nlp_options['collocation']['d']
        scheme = nlp_options['collocation']['scheme']
        Collocation = collocation.Collocation(nk, d, scheme)
        Multiple_shooting = None

    elif nlp_options['discretization'] == 'multiple_shooting':
        direct_collocation = False
        ms = True
        Collocation = None
        dae = model.get_dae()
        Multiple_shooting = multiple_shooting.Multiple_shooting(nk, dae, nlp_options['integrator'])
        slacks = None

    #---------------------------------------
    # prepare constraints structure
    #---------------------------------------
    form_constraints = formulation.constraints
    constraints_fun = formulation.constraints_fun # initial, terminal and periodicity constraints

    #-------------------------------------------
    # DISCRETIZE VARIABLES, CREATE NLP PARAMETERS
    #-------------------------------------------
    V = var_struct.setup_nlp_v(nlp_options, model, Collocation)
    P = setup_nlp_p(V, model)
    if direct_collocation:
        Xdot = Collocation.get_xdot(nlp_options, V, model)

    # construct time grids for this nlp
    time_grids = construct_time_grids(nlp_options)

    # ---------------------------------------
    # prepare outputs structure
    # ---------------------------------------
    outputs = model.outputs

    [form_outputs, _] = performance.collect_performance_outputs(nlp_options, model, V)
    form_outputs_fun = cas.Function('form_outputs_fun', [V, P], [form_outputs.cat])

    Outputs_struct = setup_output_structure(nlp_options, outputs, form_outputs)

    #-------------------------------------------
    # COLLOCATE CONSTRAINTS, OUTPUTS
    #-------------------------------------------

    # prepare listing of outputs and constraints
    Outputs_list = []

    xi = V['xi']

    ocp_cstr_list = ocp_constraint.OcpConstraintList()

    # add initial constraints
    var_initial = struct_op.get_variables_at_time(nlp_options, V, Xdot, model.variables, 0)
    var_ref_initial = struct_op.get_var_ref_at_time(nlp_options, P, V, Xdot, model, 0)
    init_cstr = operation.get_initial_constraints(nlp_options, var_initial, var_ref_initial, xi, model, formulation.xi_dict)
    ocp_cstr_list.append(init_cstr)

    # parallellize constraints on collocation nodes
    if direct_collocation:
        [coll_outputs,
        Integral_outputs_list,
        Integral_constraint_list] = Collocation.collocate_constraints(nlp_options, model, formulation, V, P, Xdot)

    # parallellize constraints on interval nodes
    if ms:
        [ms_xf,
        ms_z0,
        Xdot,
        ms_constraints,
        ms_outputs,
        Integral_outputs_list,
        Integral_constraint_list] = Multiple_shooting.discretize_constraints(nlp_options, model, formulation, V, P)


    # Construct list of constraints (+ bounds) and outputs
    for kdx in range(nk):
        if (ms) or (direct_collocation and scheme != 'radau'):

            path_constraints = model.constraints

            # at each interval node, algebraic constraints should be satisfied
            [g_list, g_bounds] = constraints.append_algebraic_constraints(
                            g_list, g_bounds, dae.z(ms_z0[:,kdx]), V, kdx)

            # at each interval node, path constraints should be satisfied
            if 'us' in list(V.keys()): # slack path constraints
                slacks = V['us',kdx]

            [g_list, g_bounds] = constraints.append_path_constraints(
                                    g_list, g_bounds, path_constraints, ms_constraints[:, kdx],slacks)

            # compute outputs for this time interval
            Outputs_list.append(ms_outputs[:,kdx])

        if direct_collocation:
            # add outputs on collocation nodes
            for ddx in range(d):
                # compute outputs for this time interval
                Outputs_list.append(coll_outputs[:,kdx*d+ddx])

        elif ms:
            # endpoint should match next start point
            [g_list, g_bounds] = Multiple_shooting.append_continuity_constraint(g_list, g_bounds, ms_xf, V, kdx)

    if direct_collocation and (scheme == 'radau'):
        ocp_cstr_list.expand_with_radau_collocation(nlp_options, P, V, Xdot, model, Collocation)

    # add terminal constraints
    var_terminal = struct_op.get_variables_at_final_time(nlp_options, V, Xdot, model)
    var_ref_terminal = struct_op.get_var_ref_at_final_time(nlp_options, P, V, Xdot, model)
    terminal_cstr = constraints.get_terminal_constraints(constraints_fun, var_terminal, var_ref_terminal, xi)
    ocp_cstr_list.append(terminal_cstr)

    # add periodic constraints
    periodic_cstr = operation.get_periodic_constraints(nlp_options, var_initial, var_terminal)
    ocp_cstr_list.append(periodic_cstr)

    if direct_collocation:
        integral_cstr = constraints.get_integral_constraints(Integral_constraint_list, formulation.integral_constants)
        ocp_cstr_list.append(integral_cstr)

    Outputs_list.append(form_outputs_fun(V, P))

    # Create Outputs struct and function
    Outputs = Outputs_struct(cas.vertcat(*Outputs_list))
    Outputs_fun = cas.Function('Outputs_fun', [V, P], [Outputs.cat])

    print_op.warn_about_temporary_funcationality_removal('discret.wake')
    # [g_list, g_bounds] = constraints.append_wake_fix_constraints(nlp_options, g_list, g_bounds, V, Outputs, model)
    # [g_list, g_bounds] = constraints.append_vortex_strength_constraints(nlp_options, g_list, g_bounds, V, Outputs, model)

    # Create Integral outputs struct and function
    Integral_outputs_struct = setup_integral_output_structure(nlp_options, model.integral_outputs)
    Integral_outputs = Integral_outputs_struct(cas.vertcat(*Integral_outputs_list))
    Integral_outputs_fun = cas.Function('Integral_outputs_fun', [V, P], [Integral_outputs.cat])

    Xdot_struct = struct_op.construct_Xdot_struct(nlp_options, model.variables_dict)
    Xdot_fun = cas.Function('Xdot_fun',[V],[Xdot])

    return V, P, Xdot_struct, Xdot_fun, ocp_cstr_list, Outputs_struct, Outputs_fun, Integral_outputs_struct, Integral_outputs_fun, time_grids, Collocation, Multiple_shooting


