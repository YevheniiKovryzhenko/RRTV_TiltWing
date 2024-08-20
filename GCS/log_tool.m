% /****************************************************************************
%  *
%  *    Copyright (C) 2024  Yevhenii Kovryzhenko. All rights reserved.
%  *
%  *    This program is free software: you can redistribute it and/or modify
%  *    it under the terms of the GNU Affero General Public License as published by
%  *    the Free Software Foundation, either version 3 of the License, or
%  *    (at your option) any later version.
%  *
%  *    This program is distributed in the hope that it will be useful,
%  *    but WITHOUT ANY WARRANTY; without even the implied warranty of
%  *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  *    GNU Affero General Public License Version 3 for more details.
%  *
%  *    You should have received a copy of the
%  *    GNU Affero General Public License Version 3
%  *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
%  *
%  *    1. Redistributions of source code must retain the above copyright
%  *       notice, this list of conditions, and the following disclaimer.
%  *    2. Redistributions in binary form must reproduce the above copyright
%  *       notice, this list of conditions, and the following disclaimer in
%  *       the documentation and/or other materials provided with the
%  *       distribution.
%  *    3. No ownership or credit shall be claimed by anyone not mentioned in
%  *       the above copyright statement.
%  *    4. Any redistribution or public use of this software, in whole or in part,
%  *       whether standalone or as part of a different project, must remain
%  *       under the terms of the GNU Affero General Public License Version 3,
%  *       and all distributions in binary form must be accompanied by a copy of
%  *       the source code, as stated in the GNU Affero General Public License.
%  *
%  ****************************************************************************/

function log_tool()
close all; clc;
%% load and parse ulog file %%
% file_name = "saves/FlightData/2024_05_31/log_21_2024-5-31-07-51-30.ulg";
% file_name = "saves/FlightData/2024_07_04/log_92_2024-7-4-08-20-04.ulg";
% file_name = "saves/FlightData/2024_07_04/log_94_2024-7-4-08-23-54.ulg";

% file_name = "saves/FlightData/2024_08_08/log_1_2024-8-8-07-57-26.ulg";
% file_name = "saves/FlightData/2024_08_08/log_10_2024-8-8-09-08-20.ulg";
file_name = "saves/FlightData/2024_08_08/log_16_2024-8-8-11-32-18.ulg";

log = ulog_plotter(file_name);


%% specify time to trim (if needed)
% log.trim_time_from_start_s = 20;
% log.trim_time_from_end_s = 0.08;

% log.trim_time_from_start_s = 20;
% log.trim_time_from_end_s = 0.95;

% log.trim_time_from_start_s = 17.5;
% log.trim_time_from_start_s = 20;
% log.trim_time_from_start_s = 30;
% log.trim_time_from_end_s = 0.95;

%% extra settings
def_fig_pos = [50 50 1600 900];

%% plot control inputs (sticks) and outputs (actuator commands)
fig_target_io_controls = figure(); fig_target_io_controls.Position = def_fig_pos;
n_subplots = 3;
i_subplot = 0;


sticks = [1 2 3 4];
log.plot_simulink_control_sticks(sticks);
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_io_controls);

% log.trim_time_from_end_s = 471.23;
% log.trim_time_from_end_s = 470.0;

sticks = [sticks, 5, 6, 12];
log.plot_simulink_inbound_sticks(sticks);
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_io_controls);

log.plot_actuators_sv([1, 2]);
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_io_controls);

log.linkaxes(fig_target_io_controls, 'x');



%% plot input states - attitude and rates 
fig_target_attitude_rates = figure(); fig_target_attitude_rates.Position = def_fig_pos;
n_subplots = 2;
i_subplot = 0;

att_indces = [1, 2, 3];
log.plot_simulink_inbound_attitude(att_indces)
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_attitude_rates);

log.plot_simulink_inbound_rates(att_indces)
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_attitude_rates);

log.linkaxes(fig_target_attitude_rates, 'x');

%% plot input states - position, velocity and accel
fig_target_translational_states = figure(); fig_target_translational_states.Position = def_fig_pos;
n_subplots = 3;
i_subplot = 0;

log.plot_simulink_inbound_position([1 2 3])
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_translational_states);

log.plot_simulink_inbound_velocity([1 2 3])
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_translational_states);

log.plot_simulink_inbound_acceleration([1,2,3])
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_translational_states);

log.linkaxes(fig_target_translational_states, 'x');

%% plot inbound states - gps
fig_target_gps_states = figure(); fig_target_gps_states.Position = def_fig_pos;
n_subplots = 2;
i_subplot = 0;

log.plot_simulink_inbound_gps_position()
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_gps_states);

log.plot_simulink_inbound_gps_altitude()
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_gps_states);

log.linkaxes(fig_target_gps_states, 'x');

%% plot extra input states
fig_target_extra_states = figure(); fig_target_extra_states.Position = def_fig_pos;
n_subplots = 2;
i_subplot = 0;

log.plot_simulink_inbound_distance_sensor_with_gc_flag()
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_extra_states);

log.plot_simulink_inbound_airspeed()
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_extra_states);

log.linkaxes(fig_target_extra_states, 'x');

%% plot internal px4 data - raw sensors, state estimates, etc
fig_target_px4_states = figure(); fig_target_px4_states.Position = def_fig_pos;
n_subplots = 4;
i_subplot = 0;

log.plot_vehicle_angular_velocity_rates([1,2,3])
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_px4_states);

log.plot_sensor_combined_gyro(att_indces)
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_px4_states);


log.plot_vehicle_acceleration([1,2,3])
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_px4_states);

log.plot_sensor_combined_accel([1,2,3])
i_subplot = i_subplot + 1;
log.insert_fig_into_subplot([n_subplots,1,i_subplot], fig_target_px4_states);

log.linkaxes(fig_target_px4_states, 'x');




end