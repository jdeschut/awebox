#!/usr/bin/python3
"""
Circular pumping trajectory for the Ampyx AP2 aircraft.
Model and constraints as in:

"Performance assessment of a rigid wing Airborne Wind Energy pumping system",
G. Licitra, J. Koenemann, A. Bürger, P. Williams, R. Ruiterkamp, M. Diehl
Energy, Vol.173, pp. 569-585, 2019.

:author: Jochem De Schutter
:edited: Rachel Leuthold
"""

import awebox as awe
from ampyx_ap2_settings import set_ampyx_ap2_settings
import matplotlib.pyplot as plt
import numpy as np

filefolder = './ltsweep/'

# indicate desired system architecture
# here: single kite with 6DOF Ampyx AP2 model
options = {}
options['user_options.system_model.architecture'] = {1:0}
options = set_ampyx_ap2_settings(options)

bref = options['user_options.kite_standard']['geometry']['b_ref']
mref = options['user_options.kite_standard']['geometry']['m_k']
jref = options['user_options.kite_standard']['geometry']['j']

kappa = 2.4
b = 26.0
options['user_options.kite_standard']['geometry']['b_ref'] = b
options['user_options.kite_standard']['geometry']['s_ref'] = b**2/options['user_options.kite_standard']['geometry']['ar']
options['user_options.kite_standard']['geometry']['c_ref'] = b/options['user_options.kite_standard']['geometry']['ar']
options['user_options.kite_standard']['geometry']['m_k'] = mref*(b/bref)**kappa
options['user_options.kite_standard']['geometry']['j'] = jref*(b/bref)**(kappa+2)
options['user_options.trajectory.lift_mode.pumping_range'] = [None, 400.0]

# indicate desired operation mode
# here: lift-mode system with pumping-cycle operation, with a one winding trajectory
options['user_options.trajectory.type'] = 'power_cycle'
options['user_options.trajectory.system_type'] = 'lift_mode'
options['user_options.trajectory.lift_mode.windings'] = 1

# indicate desired environment
# here: wind velocity profile according to power-law
options['params.wind.z_ref'] = 100.0
options['params.wind.power_wind.exp_ref'] = 0.15
options['user_options.wind.model'] = 'power'
options['user_options.wind.u_ref'] = 7.

# indicate numerical nlp details
# here: nlp discretization, with a zero-order-hold control parametrization, and a simple phase-fixing routine. also, specify a linear solver to perform the Newton-steps within ipopt.
options['nlp.n_k'] = 100
options['nlp.collocation.u_param'] = 'zoh'
options['user_options.trajectory.lift_mode.phase_fix'] = 'single_reelout'
options['solver.linear_solver'] = 'ma57' # if HSL is installed, otherwise 'mumps'
options['solver.initialization.l_t'] = 400.0
options['solver.mu_hippo'] = 1e-2

# build and optimize the NLP (trial)
trial = awe.Trial(options, 'Ampyx_AP2')
trial.build()
trial.optimize(options_seed = options, intermediate_solve=True)
import copy
ref_intermediate_sol = copy.deepcopy(trial.solution_dict)
trial.optimize(options_seed = options, warmstart_file = ref_intermediate_sol, intermediate_solve=False, recalibrate_viz = False)
trial.write_to_csv(file_name=filefolder+'lt400', frequency=50., rotation_representation='dcm')

lt_sweep = np.linspace(450, 1000, 20)
for idx, param in enumerate(lt_sweep):

    options['user_options.trajectory.lift_mode.pumping_range'] = [None, param]
    if (idx == 0):
        trial.optimize(options_seed = options, warmstart_file = ref_intermediate_sol, intermediate_solve=True)
        intermediate_sol = copy.deepcopy(trial.solution_dict)
        trial.optimize(options_seed = options, warmstart_file = trial.solution_dict, intermediate_solve=False, recalibrate_viz = False)
    else:
        trial.optimize(options_seed = options, warmstart_file = intermediate_sol, intermediate_solve=True)
        intermediate_sol = copy.deepcopy(trial.solution_dict)
        trial.optimize(options_seed = options, warmstart_file = trial.solution_dict, intermediate_solve=False, recalibrate_viz = False)

    file_name = 'lt{}'.format(int(param))
    trial.write_to_csv(file_name=filefolder + file_name, frequency=50., rotation_representation='dcm')


# draw some of the pre-coded plots for analysis
# trial.plot(['states', 'controls', 'constraints','quad'])

# # extract information from the solution for independent plotting or post-processing
# # here: plot relevant system outputs, compare to [Licitra2019, Fig 11].
# plot_dict = trial.visualization.plot_dict
# outputs = plot_dict['outputs']
# time = plot_dict['time_grids']['ip']
# avg_power = plot_dict['power_and_performance']['avg_power']/1e3

# print('======================================')
# print('Average power: {} kW'.format(avg_power))
# print('======================================')

# plt.subplots(5, 1, sharex=True)
# plt.subplot(511)
# plt.plot(time, plot_dict['x']['l_t'][0], label = 'Tether Length')
# plt.ylabel('[m]')
# plt.legend()
# plt.grid(True)

# plt.subplot(512)
# plt.plot(time, plot_dict['x']['dl_t'][0], label = 'Tether Reel-out Speed')
# plt.ylabel('[m/s]')
# plt.legend()
# plt.hlines([20, -15], time[0], time[-1], linestyle='--', color = 'black')
# plt.grid(True)

# plt.subplot(513)
# plt.plot(time, outputs['aerodynamics']['airspeed1'][0], label = 'Airspeed')
# plt.ylabel('[m/s]')
# plt.legend()
# plt.hlines([10, 32], time[0], time[-1], linestyle='--', color = 'black')
# plt.grid(True)

# plt.subplot(514)
# plt.plot(time, 180.0/np.pi*outputs['aerodynamics']['alpha1'][0], label = 'Angle of Attack')
# plt.plot(time, 180.0/np.pi*outputs['aerodynamics']['beta1'][0], label = 'Side-Slip Angle')
# plt.ylabel('[deg]')
# plt.legend()
# plt.hlines([9, -6], time[0], time[-1], linestyle='--', color = 'black')
# plt.grid(True)

# plt.subplot(515)
# plt.plot(time, outputs['local_performance']['tether_force10'][0], label = 'Tether Force Magnitude')
# plt.ylabel('[kN]')
# plt.xlabel('t [s]')
# plt.legend()
# plt.hlines([50, 1800], time[0], time[-1], linestyle='--', color = 'black')
# plt.grid(True)
# plt.show()