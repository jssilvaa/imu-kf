function [theta_hat, bias_hat] = kf_angle_bias(omega, z, dt, q_theta, q_bias, r_meas, static_ok)
% 2-state KF for angle + gyro bias:
% x=[theta;b], theta_k=theta_{k-1}+dt*(omega-b)+w_theta, b_k=b_{k-1}+w_b, z=theta+v

N = numel(z);
theta_hat = zeros(N,1);
bias_hat  = zeros(N,1);

x = [z(1); 0];
P = eye(2);

A = [1, -dt; 0, 1];
B = [dt; 0];
H = [1, 0];

Q = diag([q_theta, q_bias]);
R = r_meas;

for k=2:N
    % Predict
    x = A*x + B*omega(k);
    P = A*P*A' + Q;

    % Update (gated)
    if static_ok(k)
        y = z(k) - H*x;
        S = H*P*H' + R;
        K = (P*H') / S;
        x = x + K*y;
        P = (eye(2) - K*H) * P;
    end

    theta_hat(k) = x(1);
    bias_hat(k)  = x(2);
end
end

% this performs the kf for angle + gyro bias