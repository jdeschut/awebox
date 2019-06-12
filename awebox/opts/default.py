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
#################################################
# Method sets all user options to a default value
#################################################

import numpy as np
from . import funcs
import casadi as cas

def set_default_user_options(internal_access = False):

    ## notation for dict tree:
    ## (category, sub_categroy, sub_sub_category, parameter name, default value, (tooltip, tooltip list), sweep_type)

    default_user_options_tree = [

        ## user options
        ('user_options',    'trajectory',  None,        'type',                  'lift_mode',        ('possible options', ['lift_mode','transition']), 't'),
        ('user_options',    'trajectory',  'lift_mode', 'windings',              5,                  ('number of windings [int]', None),'s'),
        ('user_options',    'trajectory',  'lift_mode', 'phase_fix',             True,               ('choose True or False', [True, False]),'x'),
        ('user_options',    'trajectory',  'lift_mode', 'max_l_t',               None,               ('set maximum main tether length', None),'s'),
        ('user_options',    'trajectory',  'lift_mode', 'pumping_range',         None,               ('set predefined pumping range (only in comb. w. phase-fix)', None),'x'),
        ('user_options',    'trajectory',  'transition','initial_trajectory',    None,               ('relative path to pickled initial trajectory', None),'x'),
        ('user_options',    'trajectory',  'transition','terminal_trajectory',   None,               ('relative path to pickled terminal trajectory', None),'x'),
        ('user_options',    'trajectory',  'compromised_landing','emergency_scenario', ('broken_lift',2),  ('type of emergency scenario as tuple, with (SCENARIO, KITE_NODE)', None),'x'),
        ('user_options',    'trajectory',  'compromised_landing','xi_0_initial',   0.00,             ('starting position on initial trajectory between 0 and 1', None),'s'),
        ('user_options',    'trajectory',  'tracking',  'fix_tether_length',     True,                ('fixing tether length for the trajectory', [True, False]),'s'),
        ('user_options',    'trajectory',  None,        'fixed_params',          {},                 ('give dict of fixed system parameters and their values',None),'s'),
        ('user_options',    'trajectory',  'aero_test', 'phi_0',                 45. * np.pi / 180., ('pitch angle amplitude for pitch-plunge test [rad]', None),'x'),
        ('user_options',    'trajectory',  'aero_test', 'h_0',                   1.,                 ('plunge amplitude for pitch-plunge test [m]', None),'x'),
        ('user_options',    'trajectory',  'aero_test', 'omega',                 2. * np.pi,         ('frequency of pitching/plunging motion for pitch-plunge test [rad/s]', None),'x'),
        ('user_options',    'system_model',None,        'kite_dof',              6,                  ('give the number of states that designate each kites position [int]: 3 (implies roll-control), 6 (implies DCM rotation)',[3,6]),'t'),
        ('user_options',    'system_model',None,        'surface_control',       1,                  ('which derivative of the control-surface-deflection is controlled? [int]: 0 (control of deflections), 1 (control of deflection rates)', [0, 1]),'x'),
        ('user_options',    'system_model',None,        'architecture',          (1, 2),             ('choose tuple (layers,siblings)', None),'t'),
        ('user_options',    'wind',        None,        'model',                 'log_wind',         ('possible options', ['log_wind', 'uniform', 'datafile']),'x'),
        ('user_options',    'wind',        None,        'u_ref',                 5.,                 ('reference wind speed [m/s]', None),'s'),
        ('user_options',    'wind',        None,        'atmosphere_heightsdata', None,              ('data for the heights at this time instant', None),'s'),
        ('user_options',    'wind',        None,        'atmosphere_featuresdata',None,              ('data for the wind features at this time instant', None),'s'),
        ('user_options',    None,          None,        'induction_model',       'actuator',         ('possible options', ['not_in_use', 'actuator']),'x'),
        ('user_options',    None,          None,        'kite_standard',         None,               ('possible options',None),'x'),
        ('user_options',    None,          None,        'atmosphere',            'isa',              ('possible options', ['isa', 'uniform']),'x'),
        ('user_options',    None,          None,        'tether_model',          'default',          ('possible options',['default']),'x'),
        ('user_options',    None,          None,        'tether_drag_model',     'equivalence',      ('possible options',['trivial', 'simple', 'equivalence', 'not_in_use']),'t'),
        ('user_options',    None,          None,        'internal_access',       internal_access,    ('Only set internal parameters/options if you know what you are doing', [True, False]),'x'),
    ]

    default_user_options, help_options = funcs.build_options_tree(default_user_options_tree, {}, {})

    return default_user_options, help_options

def set_default_options(default_user_options, help_options):

    kite_colors = ['b', 'g', 'r', 'm', 'c', 'r', 'g', 'b', 'm', 'c', 'r', 'g', 'b', 'm', 'c']
    dim_colors = ['b', 'g', 'r', 'm', 'c', 'y', 'darkorange', 'darkkhaki', 'darkviolet']

    default_options_tree = [

        ## atmosphere model
        ('params',  'atmosphere', None, 'g',        9.81,     ('gravitational acceleration [m/s^2]', None),'s'),
        ('params',  'atmosphere', None, 'gamma',    1.4,      ('polytropic exponent of air [-]', None),'s'),
        ('params',  'atmosphere', None, 'r',        287.053,  ('universal gas constant [J/kg/K]', None),'s'),
        ('params',  'atmosphere', None, 't_ref',    288.15,   ('reference temperature [K]', None),'s'),
        ('params',  'atmosphere', None, 'p_ref',    101325.,    ('reference pressure [Pa]', None),'s'),
        ('params',  'atmosphere', None, 'rho_ref',  1.225,      ('reference air density [kg/m^3]', None),'s'),
        ('params',  'atmosphere', None, 'gamma_air',6.5e-3,     ('temperature gradient [K/m]', None),'s'),
        ('params',  'atmosphere', None, 'mu_ref',   1.789e-5,   ('dynamic viscosity of air kg/m/s', None),'s'),
        ('params',  'atmosphere', None, 'c_sutherland',   120., ('sutherland constant relating dynamic viscosity to air temperature [K]', None),'s'),

        ## wind mode
        ('params', 'wind', 'log_wind', 'z_ref',     10.,  ('reference height [m]', None),'s'),
        ('params', 'wind', 'log_wind', 'z0_air',    0.1,  ('surface roughness length of log-wind profile [m], (0.1: roughness farm land with wind breaks more than 1km apart)', None),'s'),

        ## aero model
        ('model', 'aero', 'three_dof',  'coeff_max',    [2., 80.0 * np.pi / 180.],      ('maximum coefficients in roll-control model', None),'x'),
        ('model', 'aero', 'three_dof',  'coeff_min',    [0., -80.0 * np.pi / 180.],     ('minimum coefficients in roll-control model', None),'x'),
        ('model', 'aero', 'actuator',   'a_ref',        1./3.,              ('reference value for the induction factors in actuator-disk model. takes values between 0. and 0.4', None),'x'),
        ('model', 'aero', 'actuator',   'a_range',      [-0.1, 0.5],        ('allowed range for induction factors', None),'x'),
        ('model', 'aero', 'actuator',   'scaling',      1.,                 ('scaling factor for the actuator-disk residual', None),'x'),
        ('model', 'aero', 'actuator',   'varrho_ref',   6.,                 ('approximation of the relative orbit radius, for normalization of the actuator disk equations', None),'x'),
        ('model', 'aero', 'actuator',   'varrho_range', [0., cas.inf],      ('allowed range for the relative orbit radius, for normalization of the actuator disk equations', None), 'x'),
        ('model', 'aero', 'actuator',   'steadyness',   'steady',           ('selection of steady vs unsteady actuator disk model', ['steady', 'unsteady']),'x'),
        ('model', 'aero', 'actuator',   'correct_tilt', True,               ('apply Glauert tilt correction', [True, False]), 'x'),
        ('model', 'aero', 'actuator',   'unsteady_model',       'axi_pitt_peters',      ('selection of appropriate unsteady actuator disk model', ['axi_pitt_peters', 'axi_new']),'x'),
        ('model', 'aero', 'actuator',   'normal_vector_model',  'default',              ('selection of estimation method for normal vector', ['default', 'least_squares', 'tether_parallel', 'binormal']), 'x'),

        # geometry (to be loaded!)
        ('model',  'geometry', 'overwrite', 'm_k',         None,     ('geometrical parameter', None),'s'),
        ('model',  'geometry', 'overwrite', 's_ref',       None,     ('geometrical parameter', None),'s'),
        ('model',  'geometry', 'overwrite', 'b_ref',       None,     ('geometrical parameter', None),'s'),
        ('model',  'geometry', 'overwrite', 'c_ref',       None,     ('geometrical parameter', None),'s'),
        ('model',  'geometry', 'overwrite', 'ar',          None,     ('geometrical parameter', None),'s'),
        ('model',  'geometry', 'overwrite', 'j',           None,     ('geometrical parameter', None),'s'),
        ('model',  'geometry', 'overwrite', 'length',      None,     ('geometrical parameter', None),'x'),
        ('model',  'geometry', 'overwrite', 'height',      None,     ('geometrical parameter', None),'x'),
        ('model',  'geometry', 'overwrite', 'delta_max',   None,     ('geometrical parameter', None),'t'),
        ('model',  'geometry', 'overwrite', 'ddelta_max',  None,     ('geometrical parameter', None),'t'),
        ('model',  'geometry', 'overwrite', 'c_root',      None,     ('geometrical parameter', None),'x'),
        ('model',  'geometry', 'overwrite', 'c_tip',       None,     ('geometrical parameter', None),'x'),
        ('model',  'geometry', 'overwrite', 'fuselage',    None,     ('geometrical parameter', None),'x'),
        ('model',  'geometry', 'overwrite', 'wing',        None,     ('geometrical parameter', None),'x'),
        ('model',  'geometry', 'overwrite', 'tail',        None,     ('geometrical parameter', None),'x'),
        ('model',  'geometry', 'overwrite', 'wing_profile',None,     ('geometrical parameter', None),'x'),

        # stability derivatives
        ('model',  'aero', 'overwrite', 'CL0',         None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CS0',         None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CD0',         None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLalpha',     None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSalpha',     None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDalpha',     None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLalpha2',    None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSalpha2',    None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDalpha2',    None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLbeta',      None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSbeta',      None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDbeta',      None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLbeta2',     None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSbeta2',     None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDbeta2',     None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLdeltae',    None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLdeltaa',    None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLdeltar',    None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSdeltae',    None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSdeltaa',    None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSdeltar',    None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDdeltae',    None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDdeltaa',    None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDdeltar',    None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLdeltaa2',   None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSdeltaa2',   None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDdeltaa2',   None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLdeltae2',   None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSdeltae2',   None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDdeltae2',   None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLdeltar2',   None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSdeltar2',   None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDdeltar2',   None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLalpha_deltae', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSalpha_deltae', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDalpha_deltae', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLbeta_deltaa', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSbeta_deltaa', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDbeta_deltaa', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLbeta_deltar', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSbeta_deltar', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDbeta_deltar', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLp', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSp', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDp', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLq', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSq', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDq', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CLr', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CSr', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'CDr', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cl0', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cm0', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cn0', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cldeltae', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cldeltaa', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cldeltar', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cmdeltae', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cmdeltaa', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cmdeltar', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cndeltae', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cndeltaa', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cndeltar', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Clalpha', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cmalpha', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cnalpha', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Clbeta', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cmbeta', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cnbeta', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Clp', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cmp', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cnp', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Clq', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cmq', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cnq', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Clr', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cmr', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'Cnr', None,     ('aerodynamic parameter', None),'s'),
        ('model',  'aero', 'overwrite', 'alpha_max_deg', None,     ('aerodynamic parameter', None),'t'),
        ('model',  'aero', 'overwrite', 'alpha_min_deg', None,     ('aerodynamic parameter', None),'t'),
        ('model',  'aero', 'overwrite', 'beta_max_deg', None,     ('aerodynamic parameter', None),'t'),
        ('model',  'aero', 'overwrite', 'beta_min_deg', None,     ('aerodynamic parameter', None),'t'),

        ## kite model
        #### tether properties
        ('params',  'tether', None, 'kappa',        10.,     ('Baumgarte stabilization constant for constraint formulation[-]', None),'s'),
        ('params',  'tether', None, 'rho',          970.,    ('tether material density [kg/m^3]', None),'s'),
        ('params',  'tether', None, 'cd',           1.,      ('drag coefficient [-]', None),'s'),
        ('params',  'tether', None, 'f_max',        5.,      ('max. reel-out factor [-]', None),'s'),
        ('params',  'tether', None, 'sigma_max',    3.6e9,   ('max. tether stress [Pa]', None),'s'),
        ('params',  'tether', None, 'f_sigma',      10.,     ('tether stress safety factor [-]', None),'x'),
        ('model',  'tether', None, 'control_var',  'dddl_t', ('tether control variable', ['ddl_t', 'dddl_t']),'x'),
        ('model',  'tether', None, 'aero_elements', 10,     ('number of discretizations made in approximating the tether drag. int greater than 1. [-]', None),'x'),
        ('model',  'tether', None, 'reynolds_smoothing',    1e-1,       ('smoothing width of the heaviside approximation in the cd vs. reynolds polynomial [-]', None),'x'),
        ('model',  'tether', None, 'cd_model',              'constant',  ('how to calculate the tether drag coefficient: piecewise interpolation, polyfit interpolation, constant', ['piecewise', 'polyfit', 'constant']),'x'),

        #### system bounds and limits (physical)
        ('model',  'system_bounds', 'theta',       'diam_t',       [1.0e-3, 1.0e-1],                                                  ('main tether diameter bounds [m]', None),'x'),
        ('model',  'system_bounds', 'theta',       'diam_s',       [1.0e-3, 1.0e-1],                                                  ('secondary tether diameter bounds [m]', None),'x'),
        ('model',  'system_bounds', 'xd',          'l_t',          [1.0e-2, 1.0e3],                                                   ('main tether length bounds [m]', None),'x'),
        ('model',  'system_bounds', 'theta',       'l_s',          [1.0e-2, 1.0e3],                                                   ('secondary tether length bounds [m]', None),'x'),
        ('model',  'system_bounds', 'theta',       'l_i',          [1.0e2, 1.0e2],                                                    ('intermediate tether length bounds [m]', None),'x'),
        ('model',  'system_bounds', 'xd',          'q',            [np.array([-cas.inf, -cas.inf, 10.0]), np.array([cas.inf, cas.inf, cas.inf])],         ('kite position bounds [m]', None),'x'),
        ('model',  'system_bounds', 'theta',       't_f',          [1e-3, 500.0],                                                     ('main tether max acceleration [m/s^2]', None),'x'),
        ('model',  'system_bounds', 'xa',          'lambda',       [0., cas.inf],                                                         ('multiplier bounds', None),'x'),

        #### model bounds (range of validity)
        ('model',   'model_bounds', 'tether_stress', 'include',              True,       ('include tether stress inequality in constraints', [True, False]),'x'),
        ('model',   'model_bounds', 'tether_stress', 'scaling',              1.,        ('tightness scaling for tether stress inequality', None),'x'),
        ('model',   'model_bounds', 'tether_force',  'include',              False,      ('include tether force inequality in constraints', [True, False]),'x'),
        ('params',  'model_bounds',  None,           'tether_force_limits',  np.array([1e0, 2e3]),  ('tether force limits', None),'s'),
        ('model',   'model_bounds', 'airspeed',  'include',                 False,      ('include airspeed inequality for kites in constraints', [True, False]),'x'),
        ('params',  'model_bounds',  None,           'airspeed_limits',     np.array([1.,150.]),  ('airspeed limits', None),'s'),
        ('model',   'model_bounds', 'aero_validity', 'include',              True,       ('include orientation bounds on alpha and beta (not possible in 3dof mode)', [True, False]),'x'),
        ('model',   'model_bounds', 'aero_validity', 'scaling',              1.,         ('tightness scaling for aero_validity inequalities', None),'x'),
        ('model',   'model_bounds', 'anticollision', 'safety_factor',        5.,         ('safety margin for anticollision constraint [m]', None),'x'),
        ('model',   'model_bounds', 'anticollision', 'include',              True,       ('include a minimum distance anticollision inequality in constraints', [True, False]),'x'),
        ('model',   'model_bounds', 'anticollision_radius', 'include',       False,      ('include a minimum radius anticollision inequality in constraints', [True, False]),'x'),
        ('model',   'model_bounds', 'anticollision_radius', 'scaling',       1.,         ('tightness scaling for anticollision inequalities', None),'x'),
        ('model',   'model_bounds', 'acceleration',  'include',              True,       ('include a hardware limit on node acceleration', [True, False]),'x'),
        ('model',   'model_bounds', 'acceleration',  'acc_max',              12.,        ('maximum acceleration [g]', None),'x'),
        ('model',   'model_bounds', 'rotation',     'include',               True,     ('include constraints on roll and pitch motion', None), 't'),
        ('params',  'model_bounds', None,           'rot_angles',            np.array([80.0*np.pi/180., 80.0*np.pi/180.]), ('[roll, pitch] - [rad]', None), 's'),
        ('model',   'model_bounds', 'dcoeff_actuation', 'include',      False,       ('include a bound on dcoeff', None), 'x'),
        ('model',   'model_bounds', 'coeff_actuation', 'include',      False,       ('include a bound on dcoeff', None), 'x'),
        ('model',   'model_bounds', None, 'dcoeff_max',      [5.,80.*np.pi/180],       ('include a bound on dcoeff', None), 'x'),
        ('model',   'model_bounds', None, 'dcoeff_min',      [-5.,-80.*np.pi/180],       ('include a bound on dcoeff', None), 'x'),
        ('params',   'model_bounds', None, 'coeff_compromised_max', np.array([1.5, 60*np.pi/180.]), ('include a bound on dcoeff', None), 's'),
        ('params',   'model_bounds', None, 'coeff_compromised_min', np.array([0., -60*np.pi/180.]), ('include a bound on dcoeff', None), 's'),
        ('model',   'model_bounds', None, 'dcoeff_compromised_factor', 1., ('???', None), 's'),

        #### scaling
        ('model',  'scaling', 'xd',     'l_t',      500.,     ('main tether natural length [m]', None),'x'),
        ('model',  'scaling', 'theta',  'l_i',      100.,     ('intermediate tether natural length [m]', None),'x'),
        ('model',  'scaling', 'theta',  'l_s',      50.,      ('secondary tether natural length [m]', None),'x'),
        ('model',  'scaling', 'theta',  'diam_t',   5e-3,     ('main tether natural diameter [m]', None),'x'),
        ('model',  'scaling', 'theta',  'diam_s',   5e-3,     ('secondary tether natural diameter [m]', None),'x'),
        ('model',  'scaling', 'xl',     'a',        1.0,      ('induction factor [-]', None),'x'),
        ('model',  'scaling', 'other',  'g',	    9.81,     ('acceleration to use for scaling [m/s^2]', None), 'x'),

        ('model',   'scaling_overwrite', 'lambda_tree', 'include', True, ('specific scaling of tether tension per length', None),'t'),
        ('model',   'scaling_overwrite', 'xa',     'lambda',    None,    ('scaling of tether tension per length', None),'t'),
        ('model',   'scaling_overwrite', 'xd',      'e',        None,    ('scaling of the energy', None),'t'),

        ('model',  'jit_code_gen',     None, 'include',              False,                  ('generate code with jit for model functions'),'t'),
        ('model',  'jit_code_gen',     None, 'compiler',             'clang',                ('compiler for generated code'),'t'),

        ('params',   None,       None,   'kappa_r',  1.,         ('baumgarte stabilization constant for dcm dynamics', None),'x'),

        #### ground_station
        ('params', 'ground_station', None, 'r_gen',            0.25,   ('winch generator drum radius [m]',None),'x'),
        ('params', 'ground_station', None, 'm_gen',            100.,   ('effective mass of generator [kg], guessed',None),'x'),
        ('model', 'ground_station', None, 'ddl_t_max',        10.,    ('reel-in/out acceleration limit on the tether [m/s^2]', None),'x'),

        ## formulation
        ('formulation',     'trajectory',   'aero_test',     'total_periods',    10.,         ('total number of oscillations of the wing', None),'x'),

        #### emergency landing
        ('formulation', 'nominal_landing', None, 'main_node_radius', 40., ('???', None), 'x'),
        ('formulation', 'nominal_landing', None, 'kite_node_radius', 80., ('???', None), 'x'),

        #### battery parameters
        # todo: some of these parameters have nothing to do with the battery.
        ('formulation', 'compromised_landing', 'battery', 'flap_length', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'flap_width', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'max_flap_defl', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'min_flap_defl', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'c_dl', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'c_dphi', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'defl_lift_0', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'defl_roll_0', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'voltage', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'mAh', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'charge', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'number_of_cells', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'conversion_efficiency', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'power_controller', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'power_electronics', None, ('???', None), 'x'),
        ('formulation', 'compromised_landing', 'battery', 'charge_fraction', None, ('???', None), 'x'),

        ## numerics
        #### NLP options
        ('nlp',  None,               None, 'n_k',                  40,                     ('control discretization [int]', None),'t'),
        ('nlp',  None,               None,  'discretization',      'direct_collocation',   ('possible options', ['direct_collocation']),'x'),
        ('nlp',  'collocation',      None, 'd',                    4,                      ('degree of lagrange polynomials inside collocation interval [int]', None),'t'),
        ('nlp',  'collocation',      None, 'scheme',               'radau',                ('collocation scheme', ['radau','legendre']),'x'),
        ('nlp',  'collocation',      None, 'u_param',              'zoh',                 ('control parameterization in collocation interval', ['poly','zoh']),'x'),
        ('nlp',  None,               None, 'lift_xddot',           True,                   ('lift xddot values on interval nodes', [True, False]),'x'),
        ('nlp',  None,               None, 'lift_xa',              True,                   ('lift xa values on interval nodes', [True, False]),'x'),
        ('nlp',  None,               None, 'phase_fix_reelout',    0.7,                    ('time fraction of reel-out phase', None),'x'),
        ('nlp',  None,               None, 'pumping_range',        [None, None],           ('set predefined pumping range (only in comb. w. phase-fix)', None),'x'),
        ('nlp',  'cost',             None, 'output_quadrature',    True,                   ('use quadrature for integral system outputs in cost function', (True, False)),'t'),
        ('nlp',  'parallelization',  None, 'overwrite',            None,                   ('parallellize function evaluations', (True, False)),'t'),
        ('nlp',  'parallelization',  None, 'type',                 'openmp',               ('parallellization type', (True, False)),'t'),
        ('nlp',  None,               None, 'slack_constraints',    False,                  ('slack path constraints', (True, False)),'t'),

        ### Multiple shooting integrator options
        ('nlp',  'integrator',       None, 'type',                 'collocation',          ('integrator type', ('idas', 'collocation')),'t'),
        ('nlp',  'integrator',       None, 'jit_coll',             True,                   ('code-generate coll integrator', (True, False)),'t'),
        ('nlp',  'integrator',       None, 'num_steps_coll',       1,                      ('number of steps within coll integrator', None),'t'),
        ('nlp',  'integrator',       None, 'jit_idas',             False,                  ('code-generate idas integrator', (True, False)),'t'),
        ('nlp',  'integrator',       None, 'num_steps_rk4root',    20,                     ('number of steps within rk4rootintegrator', None),'t'),
        ('nlp',  'integrator',       None, 'jit_overwrite',        None,                   ('code-generate integrator', (True, False)),'t'),
        ('nlp',  'integrator',       None, 'num_steps_overwrite',  None,                   ('number of steps within integrator', None),'t'),
        ('nlp',  'integrator',       None, 'collocation_scheme',   'radau',                ('scheme of collocation integrator', None),'t'),
        ('nlp',  'integrator',       None, 'interpolation_order',  3,                      ('order of interpolating polynomial', None),'t'),

        ### solver options
        # todo: embed other solvers
        ('solver',  None,   None,   'linear_solver',        'ma57',     ('which linear solver to use', ['ma57']),'x'),
        ('solver',  None,   None,   'hessian_approximation',False,      ('use a limited-memory hessian approximation instead of the exact Newton hessian', [True, False]),'x'),
        ('solver',  None,   None,   'max_iter',             2000,       ('maximum ipopt iterations [int]', None),'x'),
        ('solver',  None,   None,   'max_cpu_time',         1.e4,       ('maximum cpu time (seconds) ipopt can spend in one stage of the homotopy', None), 'x'),
        ('solver',  None,   None,   'mu_target',            0.,         ('target for interior point homotopy parameter in ipopt [float]', None),'x'),
        ('solver',  None,   None,   'mu_init',              1.,         ('start value for interior point homotopy parameter in ipopt [float]', None),'x'),
        ('solver',  None,   None,   'tol',                  1e-8,       ('ipopt solution tolerance [float]', None),'x'),
        ('solver',  None,   None,   'callback',             False,      ('plot intermediate solutions', [True,False]),'x'),
        ('solver',  None,   None,   'callback_step',        10,         ('callback interval [int]', None),'x'),
        ('solver',  None,   None,   'jit',                  False,      ('callback interval [int]', None),'t'),
        ('solver',  None,   None,   'compiler',            'clang',     ('callback interval [int]', None),'x'),
        ('solver',  None,   None,   'jit_flags',           '-O0',       ('flags to be passed to jit compiler', None),'t'),
        ('solver',  None,   None,   'expand_overwrite',     None,      ('expand MX --> SX [int]', None),'t'),

        ('solver',  None,   None,   'hippo_strategy',       True,       ('enable hippo strategy to increase homotopy speed', [True, False]),'x'),
        ('solver',  None,   None,   'mu_hippo',             1e-2,       ('target for interior point homotop parameter for hippo strategy [float]', None),'x'),
        ('solver',  None,   None,   'tol_hippo',            1e-4,       ('ipopt solution tolerance for hippo strategy [float]', None),'x'),
        ('solver',  None,   None,   'acceptable_iter_hippo',5,       ('ipopt solution tolerance for hippo strategy [float]', None),'x'),

        ('solver',  'initialization', None,   'ua_norm',               60.,       ('initial guess of apparent kite speed [m/s]', None),'x'),
        ('solver',  'initialization', None,   'incid_deg',             30.,       ('initial tether elevation angle [deg]', None),'x'),
        ('solver',  'initialization', None,   'initialization_type',
         'default',       ('set initialization type', None),'t'),
        ('solver',  'initialization', None,   'winding_period',        10.,        ('initial guess of reasonable period for one winding [s]', None),'x'),
        ('solver',  'initialization', None,   'min_rel_radius',        2.,        ('minimum allowed radius to span ratio allowed in initial guess [-]', None),'x'),
        ('solver',  'initialization', None,   'max_cone_angle_multi',  80.,       ('maximum allowed cone angle allowed in initial guess, for multi-kite scenarios [deg]', None),'x'),
        ('solver',  'initialization', None,   'max_cone_angle_single', 10.,       ('maximum allowed cone angle allowed in initial guess, for single-kite scenarios [deg]', None),'x'),
        ('solver',  'initialization', None,   'landing_velocity',      22.,       ('initial guess for average reel in velocity during the landing [m/s]', None),'x'),
        ('solver',  'initialization', None,   'interpolation_scheme',     's_curve',       ('interpolation scheme used for initial guess generation', ['s_curve', 'poly']),'x'),
        ('solver',  'initialization', None,   'fix_tether_length',     False,       ('fix tether length for trajectory', [True, False]),'x'),

        ('solver',   'tracking',       None,   'stagger_distance',      0.1,       ('distance between tracking trajectory and initial guess [m]', None),'x'),

        ('solver',   'weights',        None,   'dq',                    1e-1,       ('optimization weight for all dq variables [-]', None),'x'),
        ('solver',   'weights',        None,   'q',                     1e-1,       ('optimization weight for all q variables [-]', None),'x'),
        ('solver',   'weights',        None,   'omega',                 1e-1,       ('optimization weight for all omega variables [-]', None),'x'),
        ('solver',   'weights',        None,   'r',                     10.,        ('optimization weight for all r variables [-]', None),'x'),
        ('solver',   'weights',        None,   'delta',                 1e-10,      ('optimization weight for all delta variables [-]', None),'x'),
        ('solver',   'weights',        None,   'ddelta',                1e-10,      ('optimization weight for all ddelta variables [-]', None),'x'),
        ('solver',   'weights',        None,   'lambda',                1.,         ('optimization weight for all lambda variables [-]', None),'x'),
        ('solver',   'weights',        None,   'a',                     1e-3,       ('optimization weight for lifted variable a [-]', None),'x'),

        ('solver',   'weights_overwrite', None,   'dddl_t',                None,        ('optimization weight for control variable dddl_t [-]', None),'s'),

        ('solver',  'cost',             'tracking',         0,      1e-1,       ('starting cost for tracking', None),'x'),
        ('solver',  'cost',             'regularisation',   0,      1e-4,       ('starting cost for regularisation', None),'s'),
        ('solver',  'cost',             'ddq_regularisation', 0,    0,       ('starting cost for ddq_regularisation', None),'s'),

        ('solver',  'cost',             'gamma',            0,      0.,         ('starting cost for gamma', None),'x'),
        ('solver',  'cost',             'iota',             0,      0.,         ('starting cost for iota', None),'x'),
        ('solver',  'cost',             'psi',              0,      0.,         ('starting cost for psi', None),'x'),
        ('solver',  'cost',             'tau',              0,      0.,         ('starting cost for tau', None),'x'),
        ('solver',  'cost',             'eta',              0,      0.,         ('starting cost for tau', None),'x'),
        ('solver',  'cost',             'nu',               0,      0.,         ('starting cost for nu', None),'x'),
        ('solver',  'cost',             'upsilon',          0,      0.,         ('starting cost for upsilon', None),'x'),

        ('solver',  'cost',             'fictitious',       0,      1e-4,       ('starting cost for fictitious', None),'x'),
        ('solver',  'cost',             'power',            0,      0.,         ('starting cost for power', None),'x'),
        ('solver',  'cost',             't_f',              0,      1e-2,       ('starting cost for final time', None),'x'),
        ('solver',  'cost',             'theta',            0,      1e-2,       ('starting cost for theta', None),'x'),
        ('solver',  'cost',             'nominal_landing',  0,      0,          ('starting cost for nominal_landing', None),'x'),
        ('solver',  'cost',             'compromised_battery',  0,  0,          ('starting cost for compromised_battery', None),'x'),
        ('solver',  'cost',             'transition',       0,      0,          ('starting cost for transition', None),'x'),

        ('solver',  'cost',             'tracking',         1,      0.,         ('update cost for tracking', None),'x'),
        ('solver',  'cost',             'gamma',            1,      1e3,        ('update cost for gamma', None),'s'),
        ('solver',  'cost',             'iota',             1,      1e3,        ('update cost for iota', None),'x'),
        ('solver',  'cost',             'psi',              1,      1e3,        ('update cost for psi', None),'x'),
        ('solver',  'cost',             'tau',              1,      1e3,        ('update cost for tau', None),'x'),
        ('solver',  'cost',             'eta',              1,      1e3,        ('update cost for eta', None),'x'),
        ('solver',  'cost',             'nu',               1,      1e3,        ('update cost for nu', None),'x'),
        ('solver',  'cost',             'upsilon',          1,      1e3,        ('update cost for upsilon', None),'x'),

        ('solver',  'cost',             'fictitious',       1,      1e3,        ('update cost for fictitious', None),'x'),
        ('solver',  'cost',             'nominal_landing',  1,      1e-2,       ('update cost for nominal_landing', None),'x'),
        ('solver',  'cost',             'compromised_battery',  1,  1e1,        ('update cost for compromised_battery', None),'x'),
        ('solver',  'cost',             'transition',       1,      1e-1,        ('update cost for transition', None), 'x'),

        ('solver',  'cost',             'compromised_battery',  2,  0,          ('second update cost for compromised_battery', None),'x'),
        ('solver',  'cost',             'tracking',             2,  0,          ('second update cost for tracking', None),'x'),

        ('solver',  'cost_overwrite',   'power',            1,      None,       ('update cost for power', None),'t'),
        ('solver',    None,          None,        'save_trial',            False,              ('Automatically save trial after solving', [True, False]),'x'),

        ### problem health diagnostics options
        ('solver',  'health',   'singular_values',      'ratio_min_tol',                1e5,    ('ill-conditioning test threshold - largest ratio between max/min singular values', None),'x'),
        ('solver',  'health',   'singular_values',      'min_tol',                      1e-8,   ('tolerance of smallest accepted singular value', None),'x'),
        ('solver',  'health',   None,                   'active_threshold',             1e0,    ('threshold for a constraint to be considered active (smallest ratio between lambda and g). should be larger than 1', None),'x'),
        ('solver',  'health',   None,                   'autorun_check',                False,  ('run a health-check after every homotopy step', [True, False]),'x'),
        ('solver',  'health',   None,                   'after_failure_check',          False,   ('run a health-check when a homotopy step fails', [True, False]),'x'),
        ('solver',  'health',   'sosc',                 'reduced_hessian_null_tol',     1e-3,   ('tolerance of null-space test in reduced hessian', None),'x'),
        ('solver',  'health',   None,                   'matrix_entry_zero_tol',        1e-6,   ('tolerance for a matrix entry to be considered zero', None),'x'),

        ### simulation options
        ('sim', None,  None,    'number_of_finite_elements',  20,                 ('Integrator steps in one sampling interval', None), 'x'),
        ('sim', None,  None,    'sys_params',                 None,               ('system parameters dict', None), 'x'),

        ### mpc options
        ('mpc', None,  None,    'N',            10,                 ('MPC horizon', None), 'x'),
        ('mpc', None,  None,    'scheme',      'radau',             ('NLP collocation scheme', ['legendre','radau']), 'x'),
        ('mpc', None,  None,    'd',            4,                  ('NLP collocation polynomial order', None), 'x'),
        ('mpc', None,  None,    'jit',          False,              ('MPC solver jitting', None), 'x'),
        ('mpc', None,  None,    'expand',       True,               ('expand NLP expressions', None), 'x'),
        ('mpc', None,  None,    'cost_type',    'tracking',         ('MPC cost function type', ['tracking','economic']), 'x'),
        ('mpc', None,  None,    'linear_solver','ma57',             ('MPC cost function type', None), 'x'),
        ('mpc', None,  None,    'max_iter',     1000,               ('MPC solver max iterations', None), 'x'),
        ('mpc', None,  None,    'max_cpu_time', 2000,               ('MPC solver max cpu time', None), 'x'),
        ('mpc', None,  None,    'plot_flag',    False,              ('MPC plot solution for each step', None), 'x'),
        ('mpc', None,  None,    'ref_interpolator','spline',        ('periodic reference interpolation method', None), 'x'),
        ('mpc', None,  None,    'slack_inequalities',True,          ('use soft constraints or not', [True, False]), 'x'),

        ### visualization options
        ('visualization', 'cosmetics', 'trajectory', 'colors',      kite_colors,    ('list of colors for trajectory', None), 'x'),
        ('visualization', 'cosmetics', 'trajectory', 'axisfont',    {'size': '20'}, ('???', None), 'x'),
        ('visualization', 'cosmetics', 'trajectory', 'ylabelsize',  15,             ('???', None), 'x'),
        ('visualization', 'cosmetics', 'trajectory', 'kite_bodies', False,          ('choose whether kite bodies should be plotted or not', [True, False]), 'x'),
        ('visualization', 'cosmetics', 'trajectory', 'kite_num_per_meter', 3,       ('discretization level of kite body visualization', None), 'x'),
        ('visualization', 'cosmetics', 'trajectory', 'wake_nodes',  False,          ('draw wake nodes into instantaneous plots', [True, False]), 'x'),
        ('visualization', 'cosmetics', 'trajectory', 'alpha',       0.3,            ('transparency of trajectories in animation', None), 'x'),
        ('visualization', 'cosmetics', 'trajectory', 'margin',      0.05,           ('trajectory figure margins', None), 'x'),
        ('visualization', 'cosmetics', None,         'save_figs',   False,          ('save the figures', [True, False]), 'x'),
        ('visualization', 'cosmetics', None,         'plot_coll',   True,           ('plot the collocation variables', [True, False]), 'x'),
        ('visualization', 'cosmetics', None,         'plot_ref',    False,          ('plot the tracking reference trajectory', [True, False]), 'x'),
        ('visualization', 'cosmetics', 'interpolation', 'include',  True,           ('???', None), 'x'),
        ('visualization', 'cosmetics', 'interpolation', 'type',     'poly',         ('???', None), 'x'),
        ('visualization', 'cosmetics', 'interpolation', 'N',        100,            ('???', None), 'x'),
        ('visualization', 'cosmetics', 'states',      'colors',     dim_colors,     ('list of colors for states', None), 'x'),
        ('visualization', 'cosmetics', 'states',      'axisfont',   {'size': '20'}, ('???', None), 'x'),
        ('visualization', 'cosmetics', 'states',      'ylabelsize', 15,             ('???', None), 'x'),
        ('visualization', 'cosmetics', 'controls',    'colors',     dim_colors,     ('list of colors for controls', None), 'x'),
        ('visualization', 'cosmetics', 'controls',    'axisfont',   {'size': '20'}, ('???', None), 'x'),
        ('visualization', 'cosmetics', 'controls',    'ylabelsize', 15,             ('???', None), 'x'),
        ('visualization', 'cosmetics', 'invariants',  'colors',     dim_colors,     ('list of colors for invariants', None), 'x'),
        ('visualization', 'cosmetics', 'invariants',  'axisfont',   {'size': '20'}, ('???', None), 'x'),
        ('visualization', 'cosmetics', 'invariants',  'ylabelsize', 15,             ('???', None), 'x'),
        ('visualization', 'cosmetics', 'algebraic_variables', 'colors', dim_colors, ('list of colors for algebraic variables', None), 'x'),
        ('visualization', 'cosmetics', 'algebraic_variables', 'axisfont', {'size': '20'}, ('???', None), 'x'),
        ('visualization', 'cosmetics', 'algebraic_variables', 'ylabelsize', 15,     ('???', None), 'x'),
        ('visualization', 'cosmetics', 'diagnostics', 'colors',     dim_colors,     ('list of colors for algebraic variables', None), 'x'),
        ('visualization', 'cosmetics', 'diagnostics', 'axisfont',   {'size': '20'}, ('???', None), 'x'),
        ('visualization', 'cosmetics', 'diagnostics', 'ylabelsize', 15,             ('???', None), 'x'),
        ('visualization', 'cosmetics', None, 'show_when_ready', False,             ('display plots as soon as they are ready', [True, False]), 'x'),

        # quality check options
        ('quality', 'test_param', None, 'c_max', 1e0,             ('maximum invariant test parameter', None), 'x'),
        ('quality', 'test_param', None, 'dc_max', 1e1,             ('maximum invariant test parameter', None), 'x'),
        ('quality', 'test_param', None, 'ddc_max', 5e1,             ('maximum invariant test parameter', None), 'x'),
        ('quality', 'test_param', None, 'max_loyd_factor', 30,             ('maximum loyd factor test parameter', None), 'x'),
        ('quality', 'test_param', None, 'max_power_harvesting_factor', 100,             ('maximum power harvesting factor test parameter', None), 'x'),
        ('quality', 'test_param', None, 'max_tension', 1e6,             ('maximum max main tether tension test parameter', None), 'x'),
        ('quality', 'test_param', None, 'max_velocity', 100.,             ('maximum kite velocity test parameter', None), 'x'),
        ('quality', 'test_param', None, 't_f_min', 5.,             ('minimum final time test parameter', None), 'x'),
        ('quality', 'test_param', None, 'power_balance_tresh', 2e-2,             ('power balance threshold test parameter', None), 'x'),
        ('quality', 'test_param', None, 'max_control_interval', 10.,             ('max control interval test parameter', None), 'x'),
    ]

    default_options, help_options = funcs.build_options_tree(default_options_tree, default_user_options, help_options)

    return default_options, help_options
