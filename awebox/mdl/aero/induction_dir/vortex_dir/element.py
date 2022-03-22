#
#    This file is part of awebox.
#
#    awebox -- A modeling and optimization framework for multi-kite AWE systems.
#    Copyright (C) 2017-2021 Jochem De Schutter, Rachel Leuthold, Moritz Diehl,
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
object-oriented-vortex-filament-and-cylinder operations
_python-3.5 / casadi-3.4.5
- authors: rachel leuthold 2021
'''
import copy
import pdb

import casadi.tools as cas
import matplotlib.pyplot as plt
import numpy as np

import awebox.mdl.aero.induction_dir.vortex_dir.biot_savart as biot_savart

import awebox.tools.struct_operations as struct_op
import awebox.tools.vector_operations as vect_op
import awebox.tools.print_operations as print_op

from awebox.logger.logger import Logger as awelogger

import matplotlib
matplotlib.use('TkAgg')

class Element:
    def __init__(self, info_dict):
        self.__info_dict = info_dict
        self.set_element_type(None)
        self.__expected_info_length = None

    def set_info(self, packed_info):
        self.__info = packed_info

    def get_strength_color(self, strength_val, cosmetics):

        strength_max = cosmetics['trajectory']['circulation_max_estimate']
        strength_min = -1. * strength_max

        if strength_val > strength_max:
            message = 'reported vortex element strength ' + str(strength_val) + ' is larger than the maximum expected ' \
                'vortex element strength ' + str(strength_max) + '. we recommend re-calculating the expected strength range bounds.'
            awelogger.logger.warning(message)

        if strength_val < strength_min:
            message = 'reported vortex element strength ' + str(strength_val) + ' is smaller than the minimum expected ' \
                'vortex element strength ' + str(strength_min)  + '. we recommend re-calculating the expected strength range bounds'
            awelogger.logger.warning(message)

        cmap = plt.get_cmap('seismic')
        strength_scaled = float((strength_val - strength_min) / (strength_max - strength_min))
        color = cmap(strength_scaled)
        return color

    def make_symbolic_info_function(self, model_variables, model_parameters):
        self.__info_fun = cas.Function('info_fun', [model_variables, model_parameters], [self.__info])
        return None

    def evaluate_info(self, variables, parameters):
        return self.__info_fun(variables, parameters)

    def basic_draw(self, ax, side, strength, x_start, x_end, cosmetics):
        color = self.get_strength_color(strength, cosmetics)
        x = [x_start[0], x_end[0]]
        y = [x_start[1], x_end[1]]
        z = [x_start[2], x_end[2]]

        if side == 'xy':
            ax.plot(x, y, c=color)
        elif side == 'xz':
            ax.plot(x, z, c=color)
        elif side == 'yz':
            ax.plot(y, z, c=color)
        elif side == 'isometric':
            ax.plot3D(x, y, z, c=color)

    def get_repeated_info(self, period, wind, optimization_period):
        repeated_dict = copy.deepcopy(self.__info_dict)

        for name in repeated_dict.keys():
            if name[0] == 'x':
                local_x = repeated_dict[name]
                local_u = wind.get_velocity(local_x[2])
                shifted_x = local_x + local_u * optimization_period
                repeated_dict[name] = shifted_x

        return repeated_dict

    @property
    def info_fun(self):
        return self.__info_fun

    @info_fun.setter
    def info_fun(self, value):
        awelogger.logger.error('Cannot set info_fun object.')

    @property
    def info(self):
        return self.__info

    @info.setter
    def info(self, value):
        awelogger.logger.error('Cannot set info object.')

    @property
    def info_dict(self):
        return self.__info_dict

    @info_dict.setter
    def info_dict(self, value):
        awelogger.logger.error('Cannot set info_dict object.')

    @property
    def info_length(self):
        return self.__info.shape[0] * self.__info.shape[1]

    @property
    def element_type(self):
        return self.__element_type

    @element_type.setter
    def element_type(self, value):
        awelogger.logger.error('Cannot set element_type object.')

    def set_element_type(self, value):
        self.__element_type = value

    @property
    def expected_info_length(self):
        return self.__expected_info_length

    @expected_info_length.setter
    def expected_info_length(self, value):
        awelogger.logger.error('Cannot set info_length object.')

    def set_expected_info_length(self, value):
        self.__expected_info_length = value

class Filament(Element):
    def __init__(self, info_dict):
        super().__init__(info_dict)
        self.set_element_type('filament')
        packed_info = self.pack_info(info_dict)
        self.set_info(packed_info)
        self.set_expected_info_length(8)

    def unpack_info(self, packed_info):
        x_start = packed_info[0:3]
        x_end = packed_info[3:6]
        r_core = packed_info[6]
        strength = packed_info[7]

        unpacked = {'x_start': x_start,
                    'x_end': x_end,
                    'r_core': r_core,
                    'strength': strength}
        return unpacked

    def pack_info(self, dict_info):
        packed = cas.vertcat(dict_info['x_start'],
                             dict_info['x_end'],
                             dict_info['r_core'],
                             dict_info['strength']
                             )
        return packed

    def draw(self, ax, side, variables_scaled, parameters, cosmetics):
        evaluated = self.evaluate_info(variables_scaled, parameters)
        unpacked = self.unpack_info(evaluated)

        x_start = unpacked['x_start']
        x_end = unpacked['x_end']
        super().basic_draw(ax, side, unpacked['strength'], x_start, x_end, cosmetics)

        return None


class SemiInfiniteFilament(Element):
    def __init__(self, info_dict):
        super().__init__(info_dict)
        self.set_element_type('semi_infinite_filament')
        packed_info = self.pack_info(info_dict)
        self.set_info(packed_info)
        self.set_expected_info_length(8)

    def unpack_info(self, packed_info):
        x_start = packed_info[0:3]
        l_hat = packed_info[3:6]
        r_core = packed_info[6]
        strength = packed_info[7]

        unpacked = {'x_start': x_start,
                    'l_hat': l_hat,
                    'r_core': r_core,
                    'strength': strength}
        return unpacked

    def pack_info(self, dict_info):
        packed = cas.vertcat(dict_info['x_start'],
                             dict_info['l_hat'],
                             dict_info['r_core'],
                             dict_info['strength']
                             )
        return packed

    def draw(self, ax, side, variables_scaled, parameters, cosmetics):
        evaluated = self.evaluate_info(variables_scaled, parameters)
        unpacked = self.unpack_info(evaluated)

        x_start = unpacked['x_start']
        l_hat = unpacked['l_hat']

        vortex_far_convection_time = cosmetics['trajectory']['vortex_far_convection_time']
        u_ref = parameters.prefix['theta0', 'wind', 'u_ref']
        vec_length = l_hat * u_ref * vortex_far_convection_time
        x_end = x_start + vec_length

        super().basic_draw(ax, side, unpacked['strength'], x_start, x_end, cosmetics)

        return None


class Annulus(Element):
    def __init__(self, info_dict):
        super().__init__(info_dict)
        self.set_element_type('annulus')
        packed_info = self.pack_info(info_dict)
        self.set_info(packed_info)
        self.set_expected_info_length(11)

    def draw(self, ax, side, variables_scaled, parameters, cosmetics):
        evaluated = self.evaluate_info(variables_scaled, parameters)
        unpacked = self.unpack_info(evaluated)

        x_center = unpacked['x_center']
        l_hat = unpacked['l_hat']
        r_start = unpacked['radius_start']
        r_end = unpacked['radius_end']
        l_start = unpacked['l_start']

        a_hat, b_hat, _ = biot_savart.get_cylinder_axes(unpacked)

        s_start = l_start
        s_end = s_start

        n_s = cosmetics['trajectory']['cylinder_n_s']

        for tdx in range(n_s):
            theta = 2. * np.pi * float(tdx) / float(n_s)

            x_start = x_center + l_hat * s_start + r_start * np.sin(theta) * a_hat + r_start * np.cos(theta) * b_hat
            x_end = x_center + l_hat * s_end + r_end * np.sin(theta) * a_hat + r_end * np.cos(theta) * b_hat
            super().basic_draw(ax, side, unpacked['strength'], x_start, x_end, cosmetics)

        return None

    def unpack_info(self, packed_info):

        x_center = packed_info[0:3]
        l_hat = packed_info[3:6]
        radius = packed_info[6]
        l_start = packed_info[7]
        epsilon = packed_info[8]
        strength = packed_info[9]

        unpacked = {'x_center': x_center,
                    'l_hat': l_hat,
                    'radius': radius,
                    'l_start': l_start,
                    'epsilon': epsilon,
                    'strength': strength
                    }
        return unpacked

    def pack_info(self, dict_info):
        packed = cas.vertcat(dict_info['x_center'],
                             dict_info['l_hat'],
                             dict_info['radius'],
                             dict_info['l_start'],
                             dict_info['epsilon'],
                             dict_info['strength']
                             )
        return packed


class Cylinder(Element):
    def __init__(self, info_dict):
        super().__init__(info_dict)
        self.set_element_type('cylinder')
        packed_info = self.pack_info(info_dict)
        self.set_info(packed_info)
        self.set_expected_info_length(10)

    def draw(self, ax, strength_max, strength_min):
        message = 'this element is insufficiently specified for use. please use either a Longitudinal Cylinder element or a Tangential Cylinder element'
        awelogger.logger.warning(message)
        raise Exception(message)

    def unpack_info(self, packed_info):

        x_center = packed_info[0:3]
        l_hat = packed_info[3:6]
        radius_start = packed_info[6]
        radius_end = packed_info[7]
        l_start = packed_info[8]
        epsilon = packed_info[9]
        strength = packed_info[10]

        unpacked = {'x_center': x_center,
                    'l_hat': l_hat,
                    'radius_start': radius_start,
                    'radius_end': radius_end,
                    'l_start': l_start,
                    'epsilon': epsilon,
                    'strength': strength
                    }
        return unpacked

    def pack_info(self, dict_info):
        packed = cas.vertcat(dict_info['x_center'],
                             dict_info['l_hat'],
                             dict_info['radius_start'],
                             dict_info['radius_end'],
                             dict_info['l_start'],
                             dict_info['epsilon'],
                             dict_info['strength']
                             )
        return packed


class LongitudinalCylinder(Cylinder):
    def __init__(self, info_dict):
        super().__init__(info_dict)
        self.set_element_type('longitudinal_cylinder')

    def draw(self, ax, side, variables_scaled, parameters, cosmetics):
        evaluated = self.evaluate_info(variables_scaled, parameters)
        unpacked = self.unpack_info(evaluated)

        x_center = unpacked['x_center']
        l_hat = unpacked['l_hat']
        r_cyl = unpacked['radius']
        l_start = unpacked['l_start']

        a_hat, b_hat, _ = biot_savart.get_cylinder_axes(unpacked)

        s_start = l_start
        s_end = s_start + cosmetics['trajectory']['cylinder_s_length']

        n_s = cosmetics['trajectory']['cylinder_n_s']

        for tdx in range(n_s):
            theta = 2. * np.pi * float(tdx) / float(n_s)

            x_start = x_center + l_hat * s_start + r_cyl * np.sin(theta) * a_hat + r_cyl * np.cos(theta) * b_hat
            x_end = x_center + l_hat * s_end + r_cyl * np.sin(theta) * a_hat + r_cyl * np.cos(theta) * b_hat
            super().basic_draw(ax, side, unpacked['strength'], x_start, x_end, cosmetics)

        return None

class TangentialCylinder(Cylinder):
    def __init__(self, info_dict):
        super().__init__(info_dict)
        self.set_element_type('tangential_cylinder')

    def draw(self, ax, side, variables_scaled, parameters, cosmetics):
        evaluated = self.evaluate_info(variables_scaled, parameters)
        unpacked = self.unpack_info(evaluated)

        x_center = unpacked['x_center']
        l_hat = unpacked['l_hat']
        r_cyl = unpacked['radius']
        l_start = unpacked['l_start']

        a_hat, b_hat, _ = biot_savart.get_cylinder_axes(unpacked)

        s_start = l_start
        s_end = s_start + cosmetics['trajectory']['cylinder_s_length']

        n_theta = cosmetics['trajectory']['cylinder_n_theta']
        n_s = cosmetics['trajectory']['cylinder_n_s']

        for sdx in range(n_s):

            s = s_start + (s_end - s_start) / float(n_s) * float(sdx)

            for tdx in range(n_theta):
                theta_start = 2. * np.pi / float(n_theta) * float(tdx)
                theta_end = 2. * np.pi / float(n_theta) * (tdx + 1.)

                x_start = x_center + l_hat * s + r_cyl * np.sin(theta_start) * a_hat + r_cyl * np.cos(theta_start) * b_hat
                x_end = x_center + l_hat * s + r_cyl * np.sin(theta_end) * a_hat + r_cyl * np.cos(theta_end) * b_hat
                super().basic_draw(ax, side, unpacked['strength'], x_start, x_end, cosmetics)

        return None

def get_test_filament():

    x_start = 0. * vect_op.xhat_np()
    x_end = 5. * vect_op.xhat_np()
    r_core = 0.
    strength = 1.
    dict_info = {'x_start': x_start,
                 'x_end': x_end,
                 'r_core': r_core,
                 'strength': strength}

    fil = Filament(dict_info)
    return fil

def test_filament_type():

    fil = get_test_filament()
    if not (fil.element_type == 'filament'):
        message = 'something went wrong when identifying vortex element types'
        awelogger.logger.error(message)
        raise Exception(message)

    return None

def get_test_cylinder():
    x_center = 0. * vect_op.xhat_np()
    radius = 5.
    l_start = 0.
    l_hat = vect_op.xhat_np()
    epsilon = 0.
    strength = 1.
    unpacked = {'x_center': x_center,
                'l_hat': l_hat,
                'radius': radius,
                'l_start': l_start,
                'epsilon': epsilon,
                'strength': strength
                }

    cyl = Cylinder(unpacked)
    return cyl


def get_test_tangential_cylinder():
    cyl_data = get_test_cylinder().info_dict
    cyl = TangentialCylinder(cyl_data)
    return cyl

def get_test_longitudinal_cylinder():
    cyl_data = get_test_cylinder().info_dict
    cyl = LongitudinalCylinder(cyl_data)
    return cyl
