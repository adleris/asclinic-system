function s = sigmoid(t, pi, pf, closeness)
% sigmoid generation
    % This function generates the sigmoid function for use in trajectory
    % generation in path-planning
    % @param t:  time vector over which the transition 
    % @param pi: initial pose value
    % @param pf: final pose value
    % @param closeness: How close the sigmoid should reach to the initial
    % and final poses. Since the function asymptotically approaches 0 and
    % 1, it cannot exactly reach the references. By default, the function
    % will start from 0.01 and reach 0.99 times the ditance from pi to pf.
    % Bounds are [0.5, 1).

    DEFAULT_CLOSENESS = 0.99;
    if nargin < 4 || closeness < 0.5 || closeness >= 1
        closeness = DEFAULT_CLOSENESS;
    end

    middle_index = round(length(t)/2);
    t_halfway = t(middle_index);
    amplitude = (pf-pi);

    % calculate dilation parameter of sigmoid
    a = - 1/(t(end)-t_halfway) * log((1-closeness)/closeness);

    s = 1./(1+exp(-a*(t-t_halfway)));

    s = amplitude * s + pi;
end