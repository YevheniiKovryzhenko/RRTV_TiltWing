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

classdef ulog_plotter < ulog_parser
    properties
        convert_time_to_seconds (1,1) logical = true
        start_time_at_zero_abs (1,1) logical = true
        start_time_at_zero (1,1) logical = false
        % armed_only (1,1) bool = true

        trim_time_from_start_s = 0
        trim_time_from_end_s = 0
    end
    methods
        function obj = ulog_plotter(file_name)
            obj = obj@ulog_parser(file_name);
        end

        function plot_simulink_inbound_gps_position(obj)
            topic_list = obj.get_topic('simulink_inbound');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    
                    obj.plot_gps_position(topic.Time, topic.vehicle_global_position.lat(:,1), topic.vehicle_global_position.lon(:,1), "(Simulink Inbound MSG)");
                end
            end
        end

        function plot_simulink_inbound_gps_altitude(obj)
            topic_list = obj.get_topic('simulink_inbound');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    
                    obj.plot_gps_altitude(topic.Time, topic.vehicle_global_position.alt(:,1), topic.vehicle_global_position.terrain_alt(:,1), "(Simulink Inbound MSG)");
                end
            end
        end

        function plot_simulink_inbound_airspeed(obj)
            topic_list = obj.get_topic('simulink_inbound');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    
                    obj.plot_airspeed(topic.Time, topic.airspeed.true_airspeed_m_s(:,1), topic.airspeed.indicated_airspeed_m_s(:,1), "(Simulink Inbound MSG)");
                end
            end
        end

        function plot_simulink_inbound_distance_sensor_with_gc_flag(obj)
            topic_list = obj.get_topic('simulink_inbound');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    
                    obj.plot_distance_sensor(true, topic.Time, topic.distance_sensor.current_distance(:,1), topic.ground_contact.ground_contact, "(Simulink Inbound MSG)");
                end
            end
        end

        function plot_simulink_inbound_distance_sensor(obj)
            topic_list = obj.get_topic('simulink_inbound');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    obj.plot_distance_sensor(false, topic.Time, topic.distance_sensor.current_distance(:,1), [], "(Simulink Inbound MSG)");
                end
            end
        end
        
        function plot_simulink_inbound_position(obj, ind)
            topic_list = obj.get_topic('simulink_inbound');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    obj.plot_pos(ind, topic.Time, topic.vehicle_local_position.x(:,1), topic.vehicle_local_position.y(:,1), topic.vehicle_local_position.z(:,1), "(Simulink Inbound MSG)");
                end
            end
        end

        function plot_simulink_inbound_velocity(obj, ind)
            topic_list = obj.get_topic('simulink_inbound');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    obj.plot_vel(ind, topic.Time, topic.vehicle_local_position.vx(:,1), topic.vehicle_local_position.vy(:,1), topic.vehicle_local_position.vz(:,1), "(Simulink Inbound MSG)");
                end
            end
        end

        function plot_vehicle_local_position(obj, ind)
            topic_list = obj.get_topic('vehicle_local_position');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    obj.plot_pos(ind, topic.timestamp, topic.x(:,1), topic.y(:,1), topic.z(:,1), "(Vehicle Local Position MSG)");
                end
            end
        end

        function plot_vehicle_local_position_velocity(obj, ind)
            topic_list = obj.get_topic('vehicle_local_position');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    obj.plot_vel(ind, topic.timestamp, topic.vx(:,1), topic.vy(:,1), topic.vz(:,1), "(Vehicle Local Position MSG)");
                end
            end
        end

        function plot_sensor_combined_gyro(obj, ind)
            topic_list = obj.get_topic('sensor_combined');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    obj.plot_rates(ind, topic.timestamp, topic.gyro_rad(:,1), topic.gyro_rad(:,2), topic.gyro_rad(:,3), "(Sensor Combined MSG)");
                end
            end
        end

        function plot_sensor_combined_accel(obj, ind)
            topic_list = obj.get_topic('sensor_combined');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    obj.plot_accel(ind, topic.timestamp, topic.accelerometer_m_s2(:,1), topic.accelerometer_m_s2(:,2), topic.accelerometer_m_s2(:,3), "(Sensor Combined MSG)");
                end
            end
        end

        function plot_vehicle_acceleration(obj, ind)
            topic_list = obj.get_topic('vehicle_acceleration');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    obj.plot_accel(ind, topic.timestamp, topic.xyz(:,1), topic.xyz(:,2), topic.xyz(:,3), "(Vehicle Acceleration MSG)");
                end
            end
        end

        function plot_simulink_inbound_acceleration(obj, ind)
            topic_list = obj.get_topic('simulink_inbound');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i}.vehicle_local_position;
                    obj.plot_accel(ind, topic.Time, topic.ax(:), topic.ay(:), topic.az(:), "(Simulink Inbound MSG)");
                end
            end
        end

        function plot_simulink_inbound_attitude_rates(obj, ind)
            fig_target = figure();

            obj.plot_simulink_inbound_attitude(ind);            
            obj.insert_fig_into_subplot([2,1,1], fig_target);

            obj.plot_simulink_inbound_rates(ind);
            obj.insert_fig_into_subplot([2,1,2], fig_target);
        end

        % function plot_odometry_rates(obj, ind)
        %     topic_list = obj.get_topic('odometry');
        %     if (~isempty(topic_list))
        %         for topic_i = 1:length(topic_list)
        %             topic = topic_list{topic_i};
        %             obj.plot_sim_rates(ind, topic.timestamp, topic.xyz(:,1), topic.xyz(:,2), topic.xyz(:,3), "(Odometry MSG)");
        %         end
        %     end
        % end

        function plot_vehicle_angular_velocity_rates(obj, ind)
            topic_list = obj.get_topic('vehicle_angular_velocity');
            if (~isempty(topic_list))
                for topic_i = 1:length(topic_list)
                    topic = topic_list{topic_i};
                    obj.plot_rates(ind, topic.timestamp, topic.xyz(:,1), topic.xyz(:,2), topic.xyz(:,3), "(Vehicle Angular Velocity MSG)");
                end
            end
        end

        function plot_simulink_inbound_rates(obj, ind)
            simulink_inbound = obj.get_topic('simulink_inbound');
            if (~isempty(simulink_inbound))
                for topic_i = 1:length(simulink_inbound)
                    topic = simulink_inbound{topic_i}.vehicle_angular_velocity;
                    obj.plot_rates(ind, topic.Time, topic.x(:), topic.y(:), topic.z(:), "(Simulink Inbound MSG)");
                end
            end
        end

        function plot_simulink_inbound_attitude(obj, ind)
            simulink_inbound = obj.get_topic('simulink_inbound');
            if (~isempty(simulink_inbound))
                obj.plot_att(ind, simulink_inbound, "(Simulink Inbound MSG)");
            end
        end

        function plot_simulink_control_sticks(obj, stick_ind)
            simulink_inbound = obj.get_topic('simulink_inbound_ctrl');
            if (~isempty(simulink_inbound))
                obj.plot_sim_sticks(stick_ind, simulink_inbound, "- GCS Sticks (Simulink Inbound Control MSG)");
            end
        end

        function plot_simulink_inbound_sticks(obj, stick_ind)
            simulink_inbound = obj.get_topic('simulink_inbound');
            if (~isempty(simulink_inbound))
                obj.plot_sim_sticks(stick_ind, simulink_inbound, "(Simulink Inbound MSG)");
            end
        end

        


        function plot_actuators_sv(obj, actuator_ids)
            actuators = obj.get_topic('actuator_outputs_sv');
            if (~isempty(actuators))
                for topic_i = 1:length(actuators)
                    topic = actuators{topic_i};
                    [index_ok, index2plot, time] = obj.get_cropped_index(topic.timestamp);
                    if (~index_ok)
                         continue;
                    end

                    time = time(index2plot);
                    
                    if (obj.start_time_at_zero_abs)
                        time = obj.start_time_from_zero_abs(time);
                    end
                    if (obj.start_time_at_zero)
                        time = obj.start_time_from_zero(time);
                    end

                    figure
                    hold on
                    title("Actuator Outputs (CAN)")
                    n_act = length(actuator_ids);
                    leg = cell(1,n_act);
                    for i_actuator = 1:n_act
                        i_actuator_ = min(max(actuator_ids(i_actuator), 1),16);                        
                        data = topic.output(index2plot, i_actuator_);
                        plot(time, data);
                        leg{i_actuator} = sprintf("out_{%i}", i_actuator_);
                    end
                     if obj.convert_time_to_seconds
                        xlabel('Time (s)')
                    else
                        xlabel('Time (ms)')
                    end
                    ylabel("[0 1000]")
                    legend(leg(:),'Location','northwest');
                    axis tight
                    hold off
                end
            end
        end
    end

    methods (Static)
        function insert_fig_into_subplot(subpot_size, fig_target)
            fig_src = gcf;
            hTemp = subplot(subpot_size(1), subpot_size(2), subpot_size(3),'Parent',fig_target);         %# Create a temporary subplot
            newPos = get(hTemp,'Position');                  %# Get its position
            delete(hTemp);                                   %# Delete the subplot
            
            hAx = gca(fig_src); %get(fig_src, 'Child');
            set(hAx, 'Parent', fig_target)

            %# resize it to match subplot position
            set(hAx, 'Position', newPos);

            delete(fig_src)
        end

        function linkaxes(fig_, dimension)
            allAxesInFigure = findall(fig_,'type','axes');
            axNoLegendsOrColorbars= allAxesInFigure(~ismember(get(allAxesInFigure,'Tag'),{'legend','Colobar'}));
            linkaxes(axNoLegendsOrColorbars,dimension)
        end
    end

    methods (Access = private)

        function plot_gps_position(obj, Time, lat, lon, suffix)
            
            [index_ok, index2plot, time] = obj.get_cropped_index(Time);
            if (~index_ok)
                 return;
            end

            time = time(index2plot);
            
            if (obj.start_time_at_zero_abs)
                time = obj.start_time_from_zero_abs(time);
            end
            if (obj.start_time_at_zero)
                time = obj.start_time_from_zero(time);
            end

            figure
            hold on
            title(sprintf("GPS Position %s", suffix))
            
            yyaxis left
            plot(time, lat(index2plot));
            ylabel("Degrees Latitude")
            axis tight

            yyaxis right
            plot(time, lon(index2plot))    
            ylabel("Degreen Longitude")
            axis tight

            if obj.convert_time_to_seconds
                xlabel('Time (s)')
            else
                xlabel('Time (ms)')
            end
            hold off
        end

        function plot_gps_altitude(obj, Time, alt, terrain_alt, suffix)
            
            [index_ok, index2plot, time] = obj.get_cropped_index(Time);
            if (~index_ok)
                 return;
            end

            time = time(index2plot);
            
            if (obj.start_time_at_zero_abs)
                time = obj.start_time_from_zero_abs(time);
            end
            if (obj.start_time_at_zero)
                time = obj.start_time_from_zero(time);
            end

            figure
            hold on
            title(sprintf("GPS Altitude %s", suffix))
            
            yyaxis left
            plot(time, alt(index2plot));
            ylabel("m")
            axis tight

            yyaxis right
            plot(time, terrain_alt(index2plot))    
            ylabel("Terrain (m)")
            axis tight

            if obj.convert_time_to_seconds
                xlabel('Time (s)')
            else
                xlabel('Time (ms)')
            end

            hold off
        end

        function plot_airspeed(obj, Time, indicated_airspeed, true_airspeed, suffix)
            
            [index_ok, index2plot, time] = obj.get_cropped_index(Time);
            if (~index_ok)
                 return;
            end

            time = time(index2plot);
            
            if (obj.start_time_at_zero_abs)
                time = obj.start_time_from_zero_abs(time);
            end
            if (obj.start_time_at_zero)
                time = obj.start_time_from_zero(time);
            end

            figure
            hold on
            title(sprintf("Airspeed %s", suffix))

            yyaxis left
            plot(time, indicated_airspeed(index2plot));
            ylabel("Indicated (m/s)")
            axis tight

            yyaxis right
            plot(time, true_airspeed(index2plot))    
            ylabel("True (m/s)")
            axis tight

            if obj.convert_time_to_seconds
                xlabel('Time (s)')
            else
                xlabel('Time (ms)')
            end
            hold off
        end

        function plot_distance_sensor(obj, plot_gc_flag, Time, distance_data, gc_flag_data, suffix)
            
            [index_ok, index2plot, time] = obj.get_cropped_index(Time);
            if (~index_ok)
                 return;
            end

            time = time(index2plot);
            
            if (obj.start_time_at_zero_abs)
                time = obj.start_time_from_zero_abs(time);
            end
            if (obj.start_time_at_zero)
                time = obj.start_time_from_zero(time);
            end

            figure
            hold on
            title(sprintf("Distance sensor %s", suffix))
            
            
            if plot_gc_flag
                yyaxis left
                plot(time, distance_data(index2plot));
                ylabel("Distance (m)")
                axis tight

                yyaxis right
                plot(time, gc_flag_data(index2plot))    
                ylabel("Ground Contact (0/1)")
                axis tight
            else
                plot(time, distance_data(index2plot));
                ylabel("m")
                axis tight
            end

             if obj.convert_time_to_seconds
                xlabel('Time (s)')
            else
                xlabel('Time (ms)')
             end            
            
            
            hold off
        end

        function plot_pos(obj, ind, Time, x, y, z, suffix)
            
            [index_ok, index2plot, time] = obj.get_cropped_index(Time);
            if (~index_ok)
                 return;
            end

            time = time(index2plot);
            
            if (obj.start_time_at_zero_abs)
                time = obj.start_time_from_zero_abs(time);
            end
            if (obj.start_time_at_zero)
                time = obj.start_time_from_zero(time);
            end

            figure
            hold on
            title(sprintf("Position %s", suffix))
            xyz = [x(index2plot), y(index2plot), z(index2plot)];

            n_ = length(ind);
            leg = cell(1,n_);
            N_max = 3;
            leg__ = {"x", "y", "z"};
            for i_ = 1:n_
                i__ = min(max(ind(i_), 1),N_max);
                
                data = xyz(:, i__);

                plot(time, data);
                leg{i_} = leg__{i__};    
            end
             if obj.convert_time_to_seconds
                xlabel('Time (s)')
            else
                xlabel('Time (ms)')
            end
            ylabel("m")
            legend(leg(:),'Location','northwest');
            axis tight
            hold off
        end

        function plot_vel(obj, ind, Time, x, y, z, suffix)
            
            [index_ok, index2plot, time] = obj.get_cropped_index(Time);
            if (~index_ok)
                 return;
            end

            time = time(index2plot);
            
            if (obj.start_time_at_zero_abs)
                time = obj.start_time_from_zero_abs(time);
            end
            if (obj.start_time_at_zero)
                time = obj.start_time_from_zero(time);
            end

            figure
            hold on
            title(sprintf("Velocity %s", suffix))
            xyz = [x(index2plot), y(index2plot), z(index2plot)];

            n_ = length(ind);
            leg = cell(1,n_);
            N_max = 3;
            leg__ = {"v_x", "v_y", "v_z"};
            for i_ = 1:n_
                i__ = min(max(ind(i_), 1),N_max);
                
                data = xyz(:, i__);

                plot(time, data);
                leg{i_} = leg__{i__};    
            end
             if obj.convert_time_to_seconds
                xlabel('Time (s)')
            else
                xlabel('Time (ms)')
            end
            ylabel("m/s")
            legend(leg(:),'Location','northwest');
            axis tight
            hold off
        end

        function plot_accel(obj, ind, Time, x, y, z, suffix)
            
            [index_ok, index2plot, time] = obj.get_cropped_index(Time);
            if (~index_ok)
                 return;
            end

            time = time(index2plot);
            
            if (obj.start_time_at_zero_abs)
                time = obj.start_time_from_zero_abs(time);
            end
            if (obj.start_time_at_zero)
                time = obj.start_time_from_zero(time);
            end

            figure
            hold on
            title(sprintf("Acceleration %s", suffix))
            xyz = [x(index2plot), y(index2plot), z(index2plot)];

            n_ = length(ind);
            leg = cell(1,n_);
            N_max = 3;
            leg__ = {"x", "y", "z"};
            for i_ = 1:n_
                i__ = min(max(ind(i_), 1),N_max);
                
                data = xyz(:, i__);

                plot(time, data);
                leg{i_} = leg__{i__};    
            end
             if obj.convert_time_to_seconds
                xlabel('Time (s)')
            else
                xlabel('Time (ms)')
            end
            ylabel("m/s^2")
            legend(leg(:),'Location','northwest');
            axis tight
            hold off
        end

        function plot_rates(obj, ind, Time, x, y, z, suffix)
            
            [index_ok, index2plot, time] = obj.get_cropped_index(Time);
            if (~index_ok)
                 return;
            end

            time = time(index2plot);
            
            if (obj.start_time_at_zero_abs)
                time = obj.start_time_from_zero_abs(time);
            end
            if (obj.start_time_at_zero)
                time = obj.start_time_from_zero(time);
            end

            figure
            hold on
            title(sprintf("Rates %s", suffix))
            rates = [x(index2plot), y(index2plot), z(index2plot)];

            n_ = length(ind);
            leg = cell(1,n_);
            N_max = 3;
            leg__ = {"Roll Rate", "Pitch Rate", "Yaw Rate"};
            for i_ = 1:n_
                i__ = min(max(ind(i_), 1),N_max);
                
                data = rates(:, i__)*180/pi;

                plot(time, data);
                leg{i_} = leg__{i__};    
            end
             if obj.convert_time_to_seconds
                xlabel('Time (s)')
            else
                xlabel('Time (ms)')
            end
            ylabel("Degrees / second")
            legend(leg(:),'Location','northwest');
            axis tight
            hold off
        end

        function plot_att(obj, ind, simulink_inbound, suffix)
            for topic_i = 1:length(simulink_inbound)
                topic = simulink_inbound{topic_i}.vehicle_attitude;
                
                [index_ok, index2plot, time] = obj.get_cropped_index(topic.Time);
                if (~index_ok)
                     continue;
                end
    
                time = time(index2plot);
                
                if (obj.start_time_at_zero_abs)
                    time = obj.start_time_from_zero_abs(time);
                end
                if (obj.start_time_at_zero)
                    time = obj.start_time_from_zero(time);
                end
    
                figure
                hold on
                title(sprintf("Attitude %s", suffix))
                q = topic.q(index2plot,:);
                att = zeros(length(index2plot), 3);
                for i = 1:length(index2plot)
                    att(i, [3,2,1]) = lib.math.quat2eul(q(i,:), "ZYX");
                end

                n_ = length(ind);
                leg = cell(1,n_);
                N_max = 3;
                leg__ = {"Roll", "Pitch", "Yaw"};
                for i_ = 1:n_
                    i__ = min(max(ind(i_), 1),N_max);
                    
                    data = att(:, i__)*180/pi;
    
                    plot(time, data);
                    leg{i_} = leg__{i__};    
                end
                 if obj.convert_time_to_seconds
                    xlabel('Time (s)')
                else
                    xlabel('Time (ms)')
                end
                ylabel("Degrees")
                legend(leg(:),'Location','northwest');
                axis tight
                hold off
            end
        end
        
        function plot_sim_sticks(obj, stick_ind, simulink_inbound, suffix)
            for topic_i = 1:length(simulink_inbound)
                topic = simulink_inbound{topic_i};

                [index_ok, index2plot, time] = obj.get_cropped_index(topic.Time);
                if (~index_ok)
                     continue;
                end                

                time = time(index2plot);
                if (obj.start_time_at_zero_abs)
                    time = obj.start_time_from_zero_abs(time);
                end
                if (obj.start_time_at_zero)
                    time = obj.start_time_from_zero(time);
                end

                figure
                hold on
                title(sprintf("Normalized Stick Inputs %s", suffix))
                n_ = length(stick_ind);
                leg = cell(1,n_);
                N_max = length(topic.sticks.Properties.VariableNames);
                for i_ = 1:n_
                    i__ = min(max(stick_ind(i_), 1),N_max);
                    data = topic.sticks(index2plot, i__).Variables;

                    plot(time, data);
                    leg{i_} = topic.sticks.Properties.VariableNames{i__};
                end
                 if obj.convert_time_to_seconds
                    xlabel('Time (s)')
                else
                    xlabel('Time (ms)')
                end
                ylabel("[-1 1]")
                legend(leg(:),'Location','northwest');
                axis tight
                hold off
            end
        end

        function [time] = start_time_from_zero_abs(obj, time)
            if obj.convert_time_to_seconds
                time = time - seconds(obj.ulogOBJ.StartTime);
            else
                time = time - milliseconds(obj.ulogOBJ.StartTime);
            end
            
        end

        function [time] = start_time_from_zero(obj, time)
            time = time - time(1);
        end

        function [index_ok, index, time] = get_cropped_index(obj, Time)
            index_ok = false;
                
            if obj.convert_time_to_seconds
                time = seconds(Time);
                index = time >= (obj.trim_time_from_start_s + time(1)) & time <= time(end) - obj.trim_time_from_end_s; 
            else
                time = milliseconds(Time);
                index = time >= (obj.trim_time_from_start_s*1E3 + time(1)) & time <= time(end) - obj.trim_time_from_end_s*1E3;
            end
            if ~any(index)
                fprintf("Bad time cropping settings!\n")
                return;
            end
            index = find(index);
            index_ok = true;
        end
    end
end

