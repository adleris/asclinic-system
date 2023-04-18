function s = sigmoid(t, pi, pf)
% sigmoid generation
    % This function generates the sigmoid function for use in trajectory
    % generation in path-planning 
    middle_index = round(length(t)/2);
    t_halfway = t(middle_index);
    amplitude = (pf-pi);

    % calculate dilation parameter of sigmoid
    a = - 1/(t(end)-t_halfway) * log((1-.99)/.99);

    s = 1./(1+exp(-a*(t-t_halfway)));

    s = amplitude * s + pi;
end