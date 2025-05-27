function [pos, vel, P] = kalmanFilter(pos, vel, P, z, dt)
    A = [1 0 dt 0;
         0 1 0 dt;
         0 0 1  0;
         0 0 0  1];

    H = [1 0 0 0;
         0 1 0 0];

    Q = diag([0.1, 0.1, 0.1, 0.1]); % шум модели

    pos_pred = A * [pos; vel];
    P_pred = A * P * A' + Q;

    y = z - H * pos_pred;
    S = H * P_pred * H';
    K = P_pred * H' / S;

    pos_update = pos_pred + K * y;
    P_update = (eye(4) - K * H) * P_pred;

    pos = pos_update(1:2);
    vel = pos_update(3:4);
    P = P_update;
end