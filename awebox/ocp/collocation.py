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
collocation code
python-3.5 / casadi-3.4.5
- authors: elena malz, chalmers 2016
           rachel leuthold, jochem de schutter alu-fr 2017-18
- edited:  thilo bronnenmeyer 2018
'''

import casadi.tools as cas
import logging
import numpy as np
import awebox.tools.struct_operations as struct_op
from collections import OrderedDict
from . import constraints

class Collocation(object):
    """Collocation class with methods for optimal control
    """

    def __init__(self, n_k, d=4, scheme='radau'):
        """Constructor

        @param n_k number of collocation intervals.
        @param d order of collocation polynomial.
        @param scheme collocation scheme.
        """

        # save discretization info
        self.__n_k = n_k
        self.__d = d
        self.__scheme = scheme

        # get polynomial coefficients and quadrature weights
        self.__poly_coeffs()
        self.__quadrature_weights()

        return None

    def __poly_coeffs(self):
        """Compute coefficients of interpolating polynomials and their derivatives
        """

        # discretization info
        nk = self.__n_k
        d = self.__d

        # choose collocation points
        tau_root = cas.vertcat(0.0, cas.collocation_points(d, self.__scheme))

        # coefficients of the collocation equation
        coeff_collocation = np.zeros((d + 1, d + 1))

        # coefficients of the continuity equation
        coeff_continuity = np.zeros(d + 1)

        # dimensionless time inside one control interval
        tau = cas.SX.sym('tau')

        # all collocation time points
        t = np.zeros((nk, d + 1))
        for k in range(nk):
            for j in range(d + 1):
                t[k, j] = (k + tau_root[j])

        # for all collocation points
        ls = []
        ls_u = []
        for j in range(d + 1):
            # construct lagrange polynomials to get the polynomial basis at the
            # collocation point
            l = 1
            for r in range(d + 1):
                if r != j:
                    l *= (tau - tau_root[r]) / (tau_root[j] - tau_root[r])
            lfcn = cas.Function('lfcn', [tau], [l])
            ls = cas.vertcat(ls, l)

            # evaluate the polynomial at the final time to get the coefficients of
            # the continuity equation
            coeff_continuity[j] = lfcn([1.0])

            # evaluate the time derivative of the polynomial at all collocation
            # points to get the coefficients of the continuity equation
            tfcn = cas.Function('lfcntan',[tau],[cas.jacobian(l,tau)])
            for r in range(d + 1):
                coeff_collocation[j][r] = tfcn(tau_root[r])

            # construct lagrange polynomials to get the polynomial basis
            # for the controls and algebraic variables
            if j > 0:
                l = 1
                for r in range(1,d+1):
                    if r != j:
                        l *= (tau - tau_root[r]) / (tau_root[j] - tau_root[r])
                ls_u = cas.vertcat(ls_u, l)

        # interpolating function for all polynomials
        lfcns = cas.Function('lfcns',[tau],[ls])
        lfcns_u = cas.Function('lfcns_u',[tau],[ls_u])

        self.__coeff_continuity = coeff_continuity
        self.__coeff_collocation = coeff_collocation
        self.__coeff_fun = lfcns
        self.__coeff_fun_u = lfcns_u

        return None

    def build_interpolator(self, nlp_params, V, integral_outputs = None):
        """Build interpolating function over the interval
        using lagrange polynomials

        @param nlp_params nlp discretization info
        @param V decision variables struct containing coll_vars
        @return interpolation function
        """

        def coll_interpolator(time_grid, name, dim, var_type):
            """Interpolating function

            @param time_grid list with time points
            @param name xd variable name
            @param dim xd variable dimension index
            """

            vals = []
            for t in time_grid:
                kdx, tau = struct_op.calculate_kdx(nlp_params, V, t)
                if var_type == 'xd':
                    poly_vars = cas.vertcat(V['xd',kdx, name, dim], *V['coll_var',kdx, :,'xd', name, dim])
                    vals = cas.vertcat(vals, cas.mtimes(poly_vars.T, self.__coeff_fun(tau)))
                elif var_type in ['u', 'xa', 'xl']:
                    poly_vars = cas.vertcat(*V['coll_var',kdx, :,var_type, name, dim])
                    vals = cas.vertcat(vals, cas.mtimes(poly_vars.T, self.__coeff_fun_u(tau)))
                elif var_type in ['int_out']:
                    poly_vars = cas.vertcat(integral_outputs['int_out',kdx, name, dim], *integral_outputs['coll_int_out',kdx, :, name, dim])
                    vals = cas.vertcat(vals, cas.mtimes(poly_vars.T, self.__coeff_fun(tau)))

            return vals

        return coll_interpolator

    def __quadrature_weights(self):
        """ Compute quadrature weights for integration
        """

        coeff_collocation = self.__coeff_collocation
        coeff_continuity = self.__coeff_continuity

        # compute quadrature weights
        Lambda = np.linalg.solve(coeff_collocation[1:,1:], np.eye(len(coeff_collocation)-1))
        quad_weights_1 = np.matmul(Lambda,coeff_continuity[1:])
        quad_weights = np.linalg.solve(coeff_collocation[1:,1:], coeff_continuity[1:])

        self.__quad_weights = quad_weights
        self.__Lambda = Lambda

        return None

    def get_xdot(self, nlp_numerics_options, V, model):
        """ Get state derivates on all collocation nodes based on polynomials
        """

        scheme = nlp_numerics_options['collocation']['scheme']

        Vdot = struct_op.construct_Xdot_struct(nlp_numerics_options, model)

        # size of the finite elements
        h = 1. / self.__n_k

        store_derivatives = []

        # collect the derivatives
        for k in range(self.__n_k):

            tf = struct_op.calculate_tf(nlp_numerics_options, V, k)

            # For all collocation points
            for j in range(self.__d+1):
                # get an expression for the state derivative at the collocation point
                xp_jk = self.__calculate_collocation_deriv(V, k, j)

                xdot = xp_jk / h / tf
                store_derivatives = cas.vertcat(store_derivatives, xdot)

        Xdot = Vdot(store_derivatives)

        return Xdot


    def __calculate_collocation_deriv(self, V, k, j):
        """ Compute derivative of polynomial at specific node
        """

        xp_jk = 0
        for r in range(self.__d + 1):
            if r == 0:
                xp_jk += self.__coeff_collocation[r, j] * V['xd', k]
            else:
                xp_jk += self.__coeff_collocation[r, j] * V['coll_var', k, r-1,'xd']

        return xp_jk

    def get_collocation_variables_struct(self, model, nlp_options):

        entry_list = [
            cas.entry('xd', struct = model.variables_dict['xd']),
            cas.entry('xa', struct = model.variables_dict['xa'])
        ]

        if 'xl' in list(model.variables_dict.keys()):
            entry_list += [cas.entry('xl', struct = model.variables_dict['xl'])]

        if nlp_options['collocation']['u_param'] == 'poly':
            entry_list += [cas.entry('u', struct = model.variables_dict['u'])]

        if nlp_options['slack_constraints']:
            entry_list += [cas.entry('us', struct = model.constraints_dict['inequality'])]

        return cas.struct_symMX(entry_list)

    def __integrate_integral_outputs(self, Integral_outputs_list, integral_outputs_deriv, model, tf):

        # number of integral outputs
        ni = model.integral_outputs.cat.shape[0]

        if ni > 0:

            # constant term
            i0 =  model.integral_outputs(cas.vertcat(*Integral_outputs_list)[-ni:])

            # evaluate derivative functions
            derivative_list = []
            for i in range(self.__d):
                derivative_list += [model.integral_outputs(integral_outputs_deriv[:,i])]

            integral_output = OrderedDict()
            # integrate using collocation
            for name in list(model.integral_outputs.keys()):

                # get derivatives
                derivatives = []
                for i in range(len(derivative_list)):
                    derivatives.append(derivative_list[i][name])

                # compute state values at collocation nodes
                integral_output[name] = tf/self.__n_k*cas.mtimes(self.__Lambda.T, cas.vertcat(*derivatives))

                # compute state value at end of collocation interval
                integral_output_continuity = 0.0

                for j in range(self.__d):
                    integral_output_continuity += self.__coeff_continuity[j+1] * integral_output[name][j]

                integral_output[name] = cas.vertcat(integral_output[name],integral_output_continuity)

                # add constant term
                integral_output[name] += i0[name]

            # build Integral_outputs_list
            for i in range(integral_output[list(integral_output.keys())[0]].shape[0]):
                for name in list(model.integral_outputs.keys()):
                    Integral_outputs_list.append(integral_output[name][i])

        else:

            32.0

        return Integral_outputs_list


    def append_continuity_constraint(self, g_list, g_bounds, V, kdx):

        # get an expression for the state at the end of the finite element
        xf_k = 0
        for ddx in range(self.__d + 1):
            if ddx == 0:
                xf_k += self.__coeff_continuity[ddx] * V['xd', kdx]
            else:
                xf_k += self.__coeff_continuity[ddx] * V['coll_var', kdx, ddx-1, 'xd']


        # add continuity equation to nlp
        g_continuity = V['xd', kdx + 1] - xf_k
        g_list.append(g_continuity)
        # add constraint bounds
        g_bounds = constraints.append_constraint_bounds(g_bounds, 'equality', g_continuity.size()[0])

        return [g_list, g_bounds]

    def __integrate_integral_constraints(self, integral_constraints, kdx, t_f):

        integral_over_interval = {}
        for cstr_type in list(integral_constraints.keys()):
            integral_over_interval[cstr_type] = 0.
            for ddx in range(self.__d):
                integral_over_interval[cstr_type] += self.__quad_weights[ddx]*integral_constraints[cstr_type][:,kdx*self.__d+ddx]
            integral_over_interval[cstr_type] *= t_f/self.__n_k

        return integral_over_interval

    def collocate_constraints(self, options, model, formulation, V, P, Xdot):
        """ Generate collocation and path constraints on all nodes, provide integral outputs and
            integral constraints on all nodes
        """
        # extract discretization information
        N_coll = self.__n_k*self.__d # collocation points

        # extract model information
        variables = model.variables
        parameters = model.parameters

        # construct list of all collocation node variables and parameters
        coll_vars = []
        for kdx in range(self.__n_k):
            for ddx in range(self.__d):
                var_at_time = struct_op.get_variables_at_time(options, V, Xdot, model, kdx, ddx)
                coll_vars = cas.horzcat(coll_vars, var_at_time)

        coll_params = cas.repmat(parameters(cas.vertcat(P['theta0'], V['phi'])), 1, N_coll)

        # evaluate dynamics and constraint functions on all intervals
        if options['parallelization']['include']:

            # use function map for parallellization
            parallellization = options['parallelization']['type']
            dynamics = model.dynamics.map('dynamics_map', parallellization, N_coll, [], [])
            path_constraints_fun = model.constraints_fun.map('constraints_map', parallellization, N_coll, [], [])
            integral_outputs_fun = model.integral_outputs_fun.map('integral_outputs_map', parallellization, N_coll, [], [])
            outputs_fun = model.outputs_fun.map('outputs_fun', parallellization, N_coll, [], [])

            # extract formulation information
            constraints_fun_ineq = formulation.constraints_fun['integral']['inequality'].map('integral_constraints_map_ineq', 'serial', N_coll, [], [])
            constraints_fun_eq = formulation.constraints_fun['integral']['equality'].map('integral_constraints_map_eq', 'serial', N_coll, [], [])

            # evaluate functions
            coll_dynamics = dynamics(coll_vars, coll_params)
            coll_constraints = path_constraints_fun(coll_vars, coll_params)
            coll_outputs = outputs_fun(coll_vars, coll_params)
            integral_outputs_deriv = integral_outputs_fun(coll_vars, coll_params)
            integral_constraints = OrderedDict()
            integral_constraints['inequality'] = constraints_fun_ineq(coll_vars, coll_params)
            integral_constraints['equality'] = constraints_fun_eq(coll_vars, coll_params)

        else:

            # initialize function evaluations
            coll_dynamics = []
            coll_constraints = []
            coll_outputs = []
            integral_outputs_deriv = []
            integral_constraints = OrderedDict()
            integral_constraints['inequality'] = []
            integral_constraints['equality'] = []

            # evaluate functions in for loop
            for i in range(N_coll):
                coll_dynamics = cas.horzcat(coll_dynamics, model.dynamics(coll_vars[:,i],coll_params[:,i]))
                coll_constraints = cas.horzcat(coll_constraints, model.constraints_fun(coll_vars[:,i],coll_params[:,i]))
                coll_outputs = cas.horzcat(coll_outputs, model.outputs_fun(coll_vars[:,i],coll_params[:,i]))
                integral_outputs_deriv = cas.horzcat(integral_outputs_deriv, model.integral_outputs_fun(coll_vars[:,i],coll_params[:,i]))
                integral_constraints['inequality'] = cas.horzcat(integral_constraints['inequality'], formulation.constraints_fun['integral']['inequality'](coll_vars[:,i],coll_params[:,i]))
                integral_constraints['equality'] = cas.horzcat(integral_constraints['equality'], formulation.constraints_fun['integral']['equality'](coll_vars[:,i],coll_params[:,i]))


        # integrate integral outputs
        Integral_outputs_list = [np.zeros(model.integral_outputs.cat.shape[0])]
        Integral_constraints_list = []
        for kdx in range(self.__n_k):
            tf = struct_op.calculate_tf(options, V, kdx)
            Integral_outputs_list = self.__integrate_integral_outputs(Integral_outputs_list, integral_outputs_deriv[:,kdx*self.__d:(kdx+1)*self.__d], model, tf)
            Integral_constraints_list += [self.__integrate_integral_constraints(integral_constraints, kdx, tf)]

        return coll_dynamics, coll_constraints, coll_outputs, Integral_outputs_list, Integral_constraints_list



    @property
    def quad_weights(self):
        return self.__quad_weights

    @quad_weights.setter
    def quad_weights(self, value):
        logging.warning('Cannot set quad_weights object.')
