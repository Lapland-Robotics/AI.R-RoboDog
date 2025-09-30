clear % Uncomment to clear all variables in workspace
close all

flag_use_subplot        = false;
flag_use_timestamp      = false; % true to plot time using timestamp, false to use samples
flag_use_plot_limit     = true; % true to limit the plot to set values
flag_resize_axis        = true; % true to resize the axis so they feat the curves
flag_segment_position   = false; % true to plot positions of each segment
flag_segment_speed      = false; % true to plot speeds of each segment
flag_segment_torque     = false; % true to plot torques of each segment
flag_foot_forces        = false; % true to plot foot forces
flag_foot_pos2body      = false; % true to plot foot position to body
flag_legs_positions     = true;  % true to plot positions of each leg
flag_plot_viapoints     = true; % true to plot vertical lines at via points

data=readtable("catkin_ws\src\robotdog\datas\roll_data.csv"); % Uncomment to load datas

if (flag_use_plot_limit)
    plot_limit              = 250 : 512; % limit the plot to the useful samples
else
    plot_limit              = 1 : length(data.timestamp); % use all samples
end
t = data.timestamp - data.timestamp(plot_limit(1)); % starting plot time at 0

via_points=[255, 261, 268, 276, 292, 306, 360, 369, 384, 412, 434, 441, 447, 477, 506];

%% Segment position
if (flag_segment_position)
    figure()
    if (flag_use_subplot)
        set(gcf, 'Position', get(0, 'Screensize'));
        subplot(1,3,1);
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.q0(plot_limit));
        plot(t(plot_limit), data.q3(plot_limit));
        plot(t(plot_limit), data.q6(plot_limit));
        plot(t(plot_limit), data.q9(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else
        plot(plot_limit, data.q0(plot_limit));
        plot(plot_limit, data.q3(plot_limit));
        plot(plot_limit, data.q6(plot_limit));
        plot(plot_limit, data.q9(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("position (rad)", "Interpreter","latex")
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Hips position", "Interpreter","latex")
    
    if (flag_use_subplot)
        subplot(1,3,2);
    else
        figure()
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.q1(plot_limit));
        plot(t(plot_limit), data.q4(plot_limit));
        plot(t(plot_limit), data.q7(plot_limit));
        plot(t(plot_limit), data.q10(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else
        plot(plot_limit, data.q1(plot_limit));
        plot(plot_limit, data.q4(plot_limit));
        plot(plot_limit, data.q7(plot_limit));
        plot(plot_limit, data.q10(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("position (rad)", "Interpreter","latex")
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Thighs position", "Interpreter","latex")
    
    if (flag_use_subplot)
        subplot(1,3,3);
    else
        figure()
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.q2(plot_limit));
        plot(t(plot_limit), data.q5(plot_limit));
        plot(t(plot_limit), data.q8(plot_limit));
        plot(t(plot_limit), data.q11(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else
        plot(plot_limit, data.q2(plot_limit));
        plot(plot_limit, data.q5(plot_limit));
        plot(plot_limit, data.q8(plot_limit));
        plot(plot_limit, data.q11(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("position (rad)", "Interpreter","latex")
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Calves position", "Interpreter","latex")
end
%% Segment speed
if (flag_segment_speed)
    figure()
    if (flag_use_subplot)
        set(gcf, 'Position', get(0, 'Screensize'));
        subplot(1,3,1);
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.dq0(plot_limit));
        plot(t(plot_limit), data.dq3(plot_limit));
        plot(t(plot_limit), data.dq6(plot_limit));
        plot(t(plot_limit), data.dq9(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else
        plot(plot_limit, data.dq0(plot_limit));
        plot(plot_limit, data.dq3(plot_limit));
        plot(plot_limit, data.dq6(plot_limit));
        plot(plot_limit, data.dq9(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("speed (rad/s)", "Interpreter","latex")
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Hips speed", "Interpreter","latex")
    
    if (flag_use_subplot)
        subplot(1,3,2);
    else
        figure()
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.dq1(plot_limit));
        plot(t(plot_limit), data.dq4(plot_limit));
        plot(t(plot_limit), data.dq7(plot_limit));
        plot(t(plot_limit), data.dq10(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else
        plot(plot_limit, data.dq1(plot_limit));
        plot(plot_limit, data.dq4(plot_limit));
        plot(plot_limit, data.dq7(plot_limit));
        plot(plot_limit, data.dq10(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("speed (rad/s)", "Interpreter","latex")
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Thighs speed", "Interpreter","latex")
    
    if (flag_use_subplot)
        subplot(1,3,3);
    else
        figure()
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.dq2(plot_limit));
        plot(t(plot_limit), data.dq5(plot_limit));
        plot(t(plot_limit), data.dq8(plot_limit));
        plot(t(plot_limit), data.dq11(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else
        plot(plot_limit, data.dq2(plot_limit));
        plot(plot_limit, data.dq5(plot_limit));
        plot(plot_limit, data.dq8(plot_limit));
        plot(plot_limit, data.dq11(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("speed (rad/s)", "Interpreter","latex")
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Calves speed", "Interpreter","latex")
end
%% Segment torque
if (flag_segment_torque)
    figure()
    if (flag_use_subplot)
        set(gcf, 'Position', get(0, 'Screensize'));
        subplot(1,3,1);
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.tau0(plot_limit));
        plot(t(plot_limit), data.tau3(plot_limit));
        plot(t(plot_limit), data.tau6(plot_limit));
        plot(t(plot_limit), data.tau9(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else
        plot(plot_limit, data.tau0(plot_limit));
        plot(plot_limit, data.tau3(plot_limit));
        plot(plot_limit, data.tau6(plot_limit));
        plot(plot_limit, data.tau9(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("torque (N.m)", "Interpreter","latex")
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Hips torque", "Interpreter","latex")
    
    if (flag_use_subplot)
        subplot(1,3,2);
    else
        figure()
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.tau1(plot_limit));
        plot(t(plot_limit), data.tau4(plot_limit));
        plot(t(plot_limit), data.tau7(plot_limit));
        plot(t(plot_limit), data.tau10(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else
        plot(plot_limit, data.tau1(plot_limit));
        plot(plot_limit, data.tau4(plot_limit));
        plot(plot_limit, data.tau7(plot_limit));
        plot(plot_limit, data.tau10(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("torque (N.m)", "Interpreter","latex")
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Thighs torque", "Interpreter","latex")
    
    if (flag_use_subplot)
        subplot(1,3,3);
    else
        figure()
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.tau2(plot_limit));
        plot(t(plot_limit), data.tau5(plot_limit));
        plot(t(plot_limit), data.tau8(plot_limit));
        plot(t(plot_limit), data.tau11(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else
        plot(plot_limit(plot_limit), data.tau2(plot_limit));
        plot(plot_limit(plot_limit), data.tau5(plot_limit));
        plot(plot_limit(plot_limit), data.tau8(plot_limit));
        plot(plot_limit(plot_limit), data.tau11(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("torque (N.m)", "Interpreter","latex")
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Calves torque", "Interpreter","latex")
end
%% Foot forces
if (flag_foot_forces)
    figure()
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.ff0(plot_limit));
        plot(t(plot_limit), data.ff1(plot_limit));
        plot(t(plot_limit), data.ff2(plot_limit));
        plot(t(plot_limit), data.ff3(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else
        plot(plot_limit, data.ff0(plot_limit));
        plot(plot_limit, data.ff1(plot_limit));
        plot(plot_limit, data.ff2(plot_limit));
        plot(plot_limit, data.ff3(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("force (no unit)", "Interpreter","latex")
    legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex")
    title("Foot forces", "Interpreter","latex")
end
%% Foot positions to body
if (flag_foot_pos2body)
    figure()
    if (flag_use_subplot)
        set(gcf, 'Position', get(0, 'Screensize'));
        subplot(1,3,1);
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.px0(plot_limit));
        plot(t(plot_limit), data.px1(plot_limit));
        plot(t(plot_limit), data.px2(plot_limit));
        plot(t(plot_limit), data.px3(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else 
        plot(plot_limit, data.px0(plot_limit));
        plot(plot_limit, data.px1(plot_limit));
        plot(plot_limit, data.px2(plot_limit));
        plot(plot_limit, data.px3(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("position x (m)", "Interpreter","latex")
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Foot position to body x", "Interpreter","latex")
    
    if (flag_use_subplot)
        subplot(1,3,2);
    else
        figure()
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.py0(plot_limit));
        plot(t(plot_limit), data.py1(plot_limit));
        plot(t(plot_limit), data.py2(plot_limit));
        plot(t(plot_limit), data.py3(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else
        plot(plot_limit, data.py0(plot_limit));
        plot(plot_limit, data.py1(plot_limit));
        plot(plot_limit, data.py2(plot_limit));
        plot(plot_limit, data.py3(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("position y (m)", "Interpreter","latex")
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Foot position to body y", "Interpreter","latex")
    
    if (flag_use_subplot)
        subplot(1,3,3);
    else
        figure()
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.pz0(plot_limit));
        plot(t(plot_limit), data.pz1(plot_limit));
        plot(t(plot_limit), data.pz2(plot_limit));
        plot(t(plot_limit), data.pz3(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else
        plot(plot_limit, data.pz0(plot_limit));
        plot(plot_limit, data.pz1(plot_limit));
        plot(plot_limit, data.pz2(plot_limit));
        plot(plot_limit, data.pz3(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("position z (m)", "Interpreter","latex")
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end    
    title("Foot position to body z", "Interpreter","latex")

    figure()
    hold on
    plot3(data.px0(plot_limit), data.py0(plot_limit), data.pz0(plot_limit));
    plot3(data.px1(plot_limit), data.py1(plot_limit), data.pz1(plot_limit));
    plot3(data.px2(plot_limit), data.py2(plot_limit), data.pz2(plot_limit));
    plot3(data.px3(plot_limit), data.py3(plot_limit), data.pz3(plot_limit));
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    lgd = legend("Front Right","Front Left","Rear Right","Rear Left","Interpreter","latex");
    lgd.Location = 'northoutside'; 
    title("3D foot position to body", "Interpreter","latex")
end

%% Legs Positions
if (flag_legs_positions)
    figure()
    if (flag_use_subplot)
        set(gcf, 'Position', get(0, 'Screensize'));
        subplot(1,4,1);
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.q0(plot_limit));
        plot(t(plot_limit), data.q1(plot_limit));
        plot(t(plot_limit), data.q2(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else 
        plot(plot_limit, data.q0(plot_limit));
        plot(plot_limit, data.q1(plot_limit));
        plot(plot_limit, data.q2(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("positions (rad)", "Interpreter","latex")
    lgd = legend("Hips","Thighs","Calves","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Front right leg positions", "Interpreter","latex")

    if (flag_use_subplot)
        set(gcf, 'Position', get(0, 'Screensize'));
        subplot(1,4,2);
    else
        figure()
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.q3(plot_limit));
        plot(t(plot_limit), data.q4(plot_limit));
        plot(t(plot_limit), data.q5(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else 
        plot(plot_limit, data.q3(plot_limit));
        plot(plot_limit, data.q4(plot_limit));
        plot(plot_limit, data.q5(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("positions (rad)", "Interpreter","latex")
    lgd = legend("Hip","Thigh","Calve","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Front left leg positions", "Interpreter","latex")

    if (flag_use_subplot)
        set(gcf, 'Position', get(0, 'Screensize'));
        subplot(1,4,2);
    else
        figure()
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.q6(plot_limit));
        plot(t(plot_limit), data.q7(plot_limit));
        plot(t(plot_limit), data.q8(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else 
        plot(plot_limit, data.q6(plot_limit));
        plot(plot_limit, data.q7(plot_limit));
        plot(plot_limit, data.q8(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("positions (rad)", "Interpreter","latex")
    lgd = legend("Hip","Thigh","Calve","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Rear right leg positions", "Interpreter","latex")

    if (flag_use_subplot)
        set(gcf, 'Position', get(0, 'Screensize'));
        subplot(1,4,2);
    else
        figure()
    end
    hold on
    if (flag_use_timestamp)
        plot(t(plot_limit), data.q9(plot_limit));
        plot(t(plot_limit), data.q10(plot_limit));
        plot(t(plot_limit), data.q11(plot_limit));
        xlabel("time (s)", "Interpreter","latex")
    else 
        plot(plot_limit, data.q9(plot_limit));
        plot(plot_limit, data.q10(plot_limit));
        plot(plot_limit, data.q11(plot_limit));
        xlabel("sample ", "Interpreter","latex")
    end
    if (flag_resize_axis)
        axis([-inf inf -inf inf]);
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    if (flag_plot_viapoints)
        for i=1:length(via_points)
            if (flag_use_timestamp)
                xline(t(via_points(i)))
            else
                xline(via_points(i))
            end
        end
    end
    ylabel("positions (rad)", "Interpreter","latex")
    lgd = legend("Hip","Thigh","Calve","Interpreter","latex");
    if (flag_use_subplot)
        lgd.Location = 'northoutside';
    end
    title("Rear left leg positions", "Interpreter","latex")
end
%% Output
f = length(data.timestamp) / (data.timestamp(length(data.timestamp))-data.timestamp(1)); % average frequency of the whole dataset
out = data(via_points,{'timestamp', 'q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'q8', 'q9', 'q10', 'q11'});
out.timestamp = t(via_points);

% Clear useless variables in workspace
clear i;
clear lgd;
