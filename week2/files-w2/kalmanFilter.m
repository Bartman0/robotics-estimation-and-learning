function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    dt = t-previous_t;
    
    %% Place parameters like covarainces, etc. here:
    A = [1, 0, dt, 0; ...
        0, 1, 0, dt; ...
        0, 0, 1, 0; ...
        0, 0, 0, 1];
    C = [1, 0, 0, 0; ...
        0, 1, 0, 0];
    sigma_m = 1 .* 1e-3 .* [ 1, 0, 0, 0;
        0, 1, 0, 0;
        0, 0, 10, 0;
        0, 0, 0, 10 ];
    sigma_o = 1 .* 1e-2 .* eye(2);
    % P = eye(4)
    % R = eye(2);

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = sigma_m; % 1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    x_pred = A * state';
    P_pred = A * param.P * A' + sigma_m;
    R = sigma_o;
    z = [ x; y ]; % C * state';
    K = P_pred * C' * inv(C * P_pred * C' + R);
    state = x_pred + K * (z - C * x_pred);
    param.P = P_pred - K * C * P_pred;
    predictx = state(1) + state(3) * dt * 10;
    predicty = state(2) + state(4) * dt * 10;
    state = state';
end
