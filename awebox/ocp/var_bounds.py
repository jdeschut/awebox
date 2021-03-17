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
var_bounds code of the awebox
takes variable struct and options to and model inequalities, generates constraint structures, and defines the nlp constraints
python-3.5 / casadi-3.4.5
- refactored from awebox code (elena malz, chalmers; jochem de schutter, alu-fr; rachel leuthold, alu-fr), 2018
- edited: rachel leuthold, jochem de schutter alu-fr 2020
'''

import casadi.tools as cas

import awebox.tools.struct_operations as struct_op
import awebox.tools.performance_operations as perf_op
import awebox.tools.print_operations as print_op

from awebox.logger.logger import Logger as awelogger

def get_scaled_variable_bounds(nlp_options, V, model):

    # initialize
    vars_lb = V(-cas.inf)
    vars_ub = V(cas.inf)

    distinct_variables = struct_op.get_distinct_V_indices(V)

    n_k = nlp_options['n_k']
    d = nlp_options['collocation']['d']

    periodic = perf_op.determine_if_periodic(nlp_options)

    # fill in bounds
    for canonical in distinct_variables:

        [var_is_coll_var, var_type, kdx, ddx, name, dim] = struct_op.get_V_index(canonical)
        use_depending_on_periodicity = ((periodic and (not kdx is None) and (kdx < n_k)) or (not periodic))

        if (var_type == 'xd'):

            if nlp_options['collocation']['u_param'] == 'poly' and var_is_coll_var:
                vars_lb['coll_var', kdx, ddx, var_type, name] = model.variable_bounds[var_type][name]['lb']
                vars_ub['coll_var', kdx, ddx, var_type, name] = model.variable_bounds[var_type][name]['ub']
            
            elif nlp_options['collocation']['u_param'] == 'zoh' and use_depending_on_periodicity  and (not var_is_coll_var):
                vars_lb[var_type, kdx, name] = model.variable_bounds[var_type][name]['lb']
                vars_ub[var_type, kdx, name] = model.variable_bounds[var_type][name]['ub']

            [vars_lb, vars_ub] = assign_phase_fix_bounds(nlp_options, model, vars_lb, vars_ub, var_is_coll_var,
                                                            var_type, kdx, ddx, name)

        elif (var_type in {'xl', 'xa', 'u'}):
            if (var_type in V.keys()) and (not var_is_coll_var):

                vars_lb[var_type, kdx, name] = model.variable_bounds[var_type][name]['lb']
                vars_ub[var_type, kdx, name] = model.variable_bounds[var_type][name]['ub']

            elif nlp_options['collocation']['u_param'] == 'poly' and var_is_coll_var:
                vars_lb['coll_var', kdx, ddx, var_type, name] = model.variable_bounds[var_type][name]['lb']
                vars_ub['coll_var', kdx, ddx, var_type, name] = model.variable_bounds[var_type][name]['ub']

        elif (var_type == 'theta'):
            vars_lb[var_type, name] = model.variable_bounds[var_type][name]['lb']
            vars_ub[var_type, name] = model.variable_bounds[var_type][name]['ub']

        elif (var_type == 'phi'):
            vars_lb[var_type, name] = model.parameter_bounds[name]['lb']
            vars_ub[var_type, name] = model.parameter_bounds[name]['ub']

    return [vars_lb, vars_ub]

def assign_phase_fix_bounds(nlp_options, model, vars_lb, vars_ub, coll_flag, var_type, kdx, ddx, name):

    # drag-mode phase fixing: fix y-speed of first system node
    if (kdx == 0) and (not coll_flag) and (name == 'dq10') and (nlp_options['system_type'] == 'drag_mode'):
        vars_lb[var_type, 0, name, 1] = 0.0
        vars_ub[var_type, 0, name, 1] = 0.0

    # lift-mode phase fixing
    switch_kdx = round(nlp_options['n_k'] * nlp_options['phase_fix_reelout'])
    in_out_phase = (kdx < switch_kdx)

    n_k = nlp_options['n_k']

    if name == 'dl_t':

        if nlp_options['phase_fix'] == 'single_reelout':

            if (not coll_flag) and nlp_options['collocation']['u_param'] == 'zoh':

                if kdx == (n_k):
                    vars_lb[var_type, kdx, name] = 0.0
                    vars_ub[var_type, kdx, name] = 0.0

                elif in_out_phase:
                    vars_lb[var_type, kdx, name] = 0.0
                    vars_ub[var_type, kdx, name] = model.variable_bounds[var_type][name]['ub']
                else:
                    vars_lb[var_type, kdx, name] = model.variable_bounds[var_type][name]['lb']
                    vars_ub[var_type, kdx, name] = 0.0

            elif nlp_options['collocation']['u_param'] == 'poly':

                if kdx in [n_k, switch_kdx] and (not coll_flag):
                    vars_lb[var_type, kdx, name] = 0.0
                    vars_ub[var_type, kdx, name] = 0.0

                if in_out_phase and coll_flag:
                    vars_lb['coll_var', kdx, ddx, var_type, name] = 0.0
                    vars_ub['coll_var', kdx, ddx, var_type, name] = model.variable_bounds[var_type][name]['ub']
                elif not in_out_phase and coll_flag:
                    vars_lb['coll_var', kdx, ddx, var_type, name] = model.variable_bounds[var_type][name]['lb']
                    vars_ub['coll_var', kdx, ddx, var_type, name] = 0.0

        elif nlp_options['phase_fix'] == 'simple' and (kdx == 0) and (not coll_flag):

            vars_lb[var_type, kdx, name] = 0.0
            vars_ub[var_type, kdx, name] = 0.0

    pumping_range = nlp_options['pumping_range']
    if name == 'l_t':

        if kdx == 0 and (not coll_flag) and nlp_options['pumping_range'][0]:
            vars_lb[var_type, 0, name] = nlp_options['pumping_range'][0]
            vars_ub[var_type, 0, name] = nlp_options['pumping_range'][0]
        if kdx == switch_kdx and (not coll_flag) and nlp_options['pumping_range'][1]:
            vars_lb[var_type, kdx, name] = nlp_options['pumping_range'][1]
            vars_ub[var_type, kdx, name] = nlp_options['pumping_range'][1]

    return vars_lb, vars_ub
