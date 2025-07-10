function [new_x, new_y, new_theta, new_v, new_P] = kalmanFilter(prev_x, prev_y, prev_theta, prev_v, viewed_x, viewed_y, viewed_theta, viewed_v, P, dt)
    prev_vx = prev_v * cos(prev_theta);
    prev_vy = prev_v * sin(prev_theta);


    F = [1 0 dt 0;
         0 1 0 dt;
         0 0 1 0;
         0 0 0 1];


    H = [1 0 0 0;
         0 1 0 0];


    sigma_a = 0.01;
    Q = sigma_a^2 * [ ...
        dt^4/4,     0,      dt^3/2,     0;
        0,          dt^4/4, 0,          dt^3/2;
        dt^3/2,     0,      dt^2,       0;
        0,          dt^3/2, 0,          dt^2 ];

    R = diag([5, 5]);

    X = [prev_x; prev_y; prev_vx; prev_vy];

    X_pred = F * X;
    P_pred = F * P * F' + Q;

    z = [viewed_x; viewed_y];
    Y = z - H * X_pred;

    K_numerator = P_pred * H';
    K_denominator = H * P_pred * H' + R;
    K = K_numerator / K_denominator;

    X_upd = X_pred + K * Y;

    I = eye(size(F));
    P_upd = (I - K * H) * P_pred;

    new_x = X_upd(1);
    new_y = X_upd(2);
    new_vx = X_upd(3);
    new_vy = X_upd(4);
    new_v = sqrt(new_vx^2 + new_vy^2);
    new_theta = atan2(new_vy, new_vx);
    new_P = P_upd;
end