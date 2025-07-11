function [x_pred, y_pred, vx_pred, vy_pred, P_pred] = kalmanPredictOnly(prev_x, prev_y, prev_v, prev_theta, P, dt)
    % Переводим скорость в компоненты
    prev_vx = prev_v * cos(prev_theta);
    prev_vy = prev_v * sin(prev_theta);

    % Матрица перехода
    F = [1 0 dt 0;
         0 1 0 dt;
         0 0 1 0;
         0 0 0 1];

    % Дисперсия шума ускорения
    sigma_a = 0.01;

    % Процессный шум
    Q = sigma_a^2 * [ ...
        dt^4/4,     0,      dt^3/2,     0;
        0,          dt^4/4, 0,          dt^3/2;
        dt^3/2,     0,      dt^2,       0;
        0,          dt^3/2, 0,          dt^2 ];

    % Предыдущее состояние
    X = [prev_x; prev_y; prev_vx; prev_vy];

    % Предсказание состояния и ковариации
    X_pred = F * X;
    P_pred = F * P * F' + Q;

    % Возврат предсказанных координат и ковариации
    x_pred = X_pred(1);
    y_pred = X_pred(2);
    vx_pred = X_pred(3);
    vy_pred = X_pred(4);
end
