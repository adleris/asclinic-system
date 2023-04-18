% Test cascade of sigmoid functions
% Generates a random set of target poses and draws sigmoids to connect them. 
% Provides a comparison to use of step reference

% time values for calculation
t = 0:0.01:1;
% time steps for new reference generation
T = 0:5;

% path to generate trjectories for
path = 3*rand(1, length(T)+ 2);

figure()
hold on

% planned trajectory
x = zeros(1, length(t) * length(T));

% path info
path_idx = 2;
curr_pose = path(1);
next_pose = path(path_idx);

for T_step = T
    this_t = t + T_step;
    x(T_step+1:T_step+ + length(t)) = sigmoid(this_t, curr_pose, next_pose);

    plot(this_t, x(T_step+1:T_step+ + length(t)), 'b');

    x_step_reference = next_pose * ones(1, length(this_t));
    x_step_reference(1) = curr_pose; % draw line from last pose
    plot(this_t, x_step_reference, 'r');
    

    % update x path info
    path_idx = path_idx + 1;
    curr_pose = next_pose;
    next_pose = path(path_idx);
end

xlabel('time (s)');
ylabel('pose (m or rad)');
title('pose vs time for single variable');
legend({'sigmoid reference', 'step reference'})
