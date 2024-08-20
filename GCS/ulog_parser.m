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

classdef ulog_parser    
    properties (Access = protected)
        ulogOBJ
        raw_data
    end

    methods
        function obj = ulog_parser(file_name)
            obj.ulogOBJ = ulogreader(file_name);
            obj.raw_data = readTopicMsgs(obj.ulogOBJ);
        end

        function out = get_topic(obj, topic_name)
            out = [];
            switch topic_name
                case 'simulink_inbound_ctrl'
                    out = obj.get_simulink_inbound_ctrl();
                case 'simulink_inbound'
                    out = obj.get_simulink_inbound();
                otherwise
                    [res, index] = obj.check_topic(topic_name);
                    if (res)
                        topic = obj.raw_data(index, :);
                        [ntopics, ~] = size(topic);
                        out = cell(1,ntopics);
                        for i = 1:ntopics
                            out{i} = topic.TopicMessages{i};
                        end
                    end
            end            
        end

        function out = get_simulink_inbound_ctrl(obj)
            out = [];
            [res, index] = obj.check_topic('simulink_inbound_ctrl');
            if (res)
                topic = obj.raw_data(index, :);
                [ntopics, ~] = size(topic);
                out = cell(1,ntopics);
                for i = 1:ntopics
                    messages = topic.TopicMessages{i};
                    sticks = timetable(...
                        messages.timestamp, ...
                        messages.data(:,1), ...
                        messages.data(:,2), ...
                        messages.data(:,3), ...
                        messages.data(:,4), ...
                        messages.data(:,5), ...
                        messages.data(:,6), ...
                        messages.data(:,7), ...
                        messages.data(:,8), ...
                        messages.data(:,9), ...
                        messages.data(:,10), ...
                        messages.data(:,11), ...
                        messages.data(:,12), ...
                        messages.data(:,13), ...
                        messages.data(:,14), ...
                        messages.data(:,15), ...
                        messages.data(:,16), ...
                        messages.data(:,17), ...
                        messages.data(:,18));
                    sticks.Properties.VariableNames = {
                        'ROLL', ...
                        'PITCH', ...
                        'YAW', ...
                        'THROTTLE', ...
                        'ARMED', ...
                        'MODE', ...
                        'AUX1', ...
                        'AUX2', ...
                        'AUX3', ...
                        'AUX4', ...
                        'AUX5', ...
                        'AUX6', ...
                        'EXTR1', ...
                        'EXTR2', ...
                        'EXTR3', ...
                        'EXTR4', ...
                        'EXTR5', ...
                        'EXTR6'};
                    % CONTROL_VEC_OFFSET = length(sticks.Properties.VariableNames)-1;

                    out{i} = timetable(...
                        messages.timestamp, ...
                        sticks);
                    out{i}.Properties.VariableNames = {
                        'sticks'};
                end
            end
        end

        function out = get_simulink_inbound(obj)
            out = [];
            [res, index] = obj.check_topic('simulink_inbound');
            if (res)
                topic = obj.raw_data(index, :);
                [ntopics, ~] = size(topic);
                out = cell(1,ntopics);
                for i = 1:ntopics
                    messages = topic.TopicMessages{i};
                    sticks = timetable(...
                        messages.timestamp, ...
                        messages.data(:,1), ...
                        messages.data(:,2), ...
                        messages.data(:,3), ...
                        messages.data(:,4), ...
                        messages.data(:,5), ...
                        messages.data(:,6), ...
                        messages.data(:,7), ...
                        messages.data(:,8), ...
                        messages.data(:,9), ...
                        messages.data(:,10), ...
                        messages.data(:,11), ...
                        messages.data(:,12), ...
                        messages.data(:,13), ...
                        messages.data(:,14), ...
                        messages.data(:,15), ...
                        messages.data(:,16), ...
                        messages.data(:,17), ...
                        messages.data(:,18));
                    sticks.Properties.VariableNames = {
                        'ROLL', ...
                        'PITCH', ...
                        'YAW', ...
                        'THROTTLE', ...
                        'ARMED', ...
                        'MODE', ...
                        'AUX1', ...
                        'AUX2', ...
                        'AUX3', ...
                        'AUX4', ...
                        'AUX5', ...
                        'AUX6', ...
                        'EXTR1', ...
                        'EXTR2', ...
                        'EXTR3', ...
                        'EXTR4', ...
                        'EXTR5', ...
                        'EXTR6'};
                    CONTROL_VEC_OFFSET = length(sticks.Properties.VariableNames);
                    
                    
                    vehicle_local_position = timetable(...
                        messages.timestamp, ...
                        messages.data(:,1 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,2 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,3 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,15 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,16 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,17 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,18 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,19 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,20 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,21 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,32 + CONTROL_VEC_OFFSET));
                    vehicle_local_position.Properties.VariableNames = {
                        'vx', ...
                        'vy', ...
                        'vz', ...
                        'ax', ...
                        'ay', ...
                        'az', ...
                        'x', ...
                        'y', ...
                        'z', ...
                        'z_deriv', ...
                        'eph'};

                    vehicle_angular_velocity = timetable(...
                        messages.timestamp, ...
                        messages.data(:,4 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,5 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,6 + CONTROL_VEC_OFFSET));
                    vehicle_angular_velocity.Properties.VariableNames = {
                        'x', ...
                        'y', ...
                        'z'};

                    vehicle_attitude = timetable(...
                        messages.timestamp, ...
                        messages.data(:,(7:10) + CONTROL_VEC_OFFSET));
                    vehicle_attitude.Properties.VariableNames = {
                        'q'};

                    vehicle_odometry = timetable(...
                        messages.timestamp, ...
                        messages.data(:,(7:10) + CONTROL_VEC_OFFSET), ...
                        messages.data(:,4 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,5 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,6 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,26 + CONTROL_VEC_OFFSET));
                    vehicle_odometry.Properties.VariableNames = {
                        'q', ...
                        'rollspeed', ...
                        'pitchspeed', ...
                        'yawspeed', ...
                        'vz'};

                    vehicle_global_position = timetable(...
                        messages.timestamp, ...
                        messages.data(:,11 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,12 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,13 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,14 + CONTROL_VEC_OFFSET));
                    vehicle_global_position.Properties.VariableNames = {
                        'lat', ...
                        'lon', ...
                        'alt', ...
                        'terrain_alt'};

                    ground_contact = timetable(...
                        messages.timestamp, ...
                        messages.data(:,22 + CONTROL_VEC_OFFSET));
                    ground_contact.Properties.VariableNames = {
                        'ground_contact'};

                    distance_sensor = timetable(...
                        messages.timestamp, ...
                        messages.data(:,25 + CONTROL_VEC_OFFSET));
                    distance_sensor.Properties.VariableNames = {
                        'current_distance'};

                    airspeed = timetable(...
                        messages.timestamp, ...
                        messages.data(:,23 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,24 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,33 + CONTROL_VEC_OFFSET));
                    airspeed.Properties.VariableNames = {
                        'true_airspeed_m_s', ...
                        'indicated_airspeed_m_s', ...
                        'confidence'};

                    vehicle_gps_position = timetable(...
                        messages.timestamp, ...
                        messages.data(:,27 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,28 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,29 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,30 + CONTROL_VEC_OFFSET), ...
                        messages.data(:,31 + CONTROL_VEC_OFFSET));
                    vehicle_gps_position.Properties.VariableNames = {
                        'lat', ...
                        'lon', ...
                        'vel_e_m_s', ...
                        'vel_d_m_s', ...
                        'vel_n_m_s'};

                    out{i} = timetable(...
                        messages.timestamp, ...
                        sticks, ...
                        vehicle_local_position, ...
                        vehicle_angular_velocity, ...
                        vehicle_attitude, ...
                        vehicle_odometry, ...
                        vehicle_global_position, ...
                        ground_contact, ...
                        distance_sensor, ...
                        airspeed, ...
                        vehicle_gps_position);
                    out{i}.Properties.VariableNames = {
                        'sticks', ...
                        'vehicle_local_position', ...
                        'vehicle_angular_velocity', ...
                        'vehicle_attitude', ...
                        'vehicle_odometry', ...
                        'vehicle_global_position', ...
                        'ground_contact', ...
                        'distance_sensor', ...
                        'airspeed', ...
                        'vehicle_gps_position'};
                end
            end
        end
    end

    methods (Access=private)
        function [res, index] = check_topic(obj, name)
            index = -1;
            res = obj.raw_data.TopicNames == name;
            if (any(res))
                index = find(res);
            end
            res = any(res);
        end
    end
end

