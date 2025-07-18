function [new_x, new_y, new_theta, new_v, new_P] = kalmanFilter(prev_x, prev_y, prev_theta, prev_v, viewed_x, viewed_y, viewed_theta, viewed_v, P, dt)
    % prev - параметры объекта на предыдущем шаге
    % viewed - предсказанные параметры объекта
    % deltaT - временной шаг
    
    prev_vx = prev_v * cos(prev_theta);
    prev_vy = prev_v * sin(prev_theta);
    viewed_vx = viewed_v * cos(viewed_theta);
    viewed_vy = viewed_v * sin(viewed_theta);

    %% 1. Модель динамики (F) - постоянная скорость
    F = [1 0 dt 0;
         0 1 0 dt;
         0 0 1 0;
         0 0 0 1];
   
    %% 2. Модель наблюдений (H)
    H = [1 0 0 0;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
    
    %% 3. Ковариационная матрица процесса (Q)
    sigma_a = 0.1; % шум ускорения, его пока нет
    Q = sigma_a^2 * [ ...
        dt^4/4,     0,      dt^3/2,     0;
        0,          dt^4/4, 0,          dt^3/2;
        dt^3/2,     0,      dt^2,       0;
        0,          dt^3/2, 0,          dt^2 ];
    
    %% 4. Ковариационная матрица наблюдений (R)
    R = diag([5, 5, 0.1, 0.1]);
    
    %% 5. Вектор состояния
    X = [prev_x; prev_y; prev_vx; prev_vy];
    
    %% 6. Прогноз состояния
    X_pred = F * X;
    
    %% 7. Прогноз ковариации
    P_pred = F * P * F' + Q;
    
    %% 8. Вычисление матрицы Калмана
    K_numerator = P_pred * H';
    K_denominator = H * P_pred * H' + R;
    K = K_numerator / K_denominator;
    
    %% 9. Обновление состояния
    z = [viewed_x; viewed_y; viewed_vx; viewed_vy];
    Y = z - H * X_pred; % инновация
    X_upd = X_pred + K * Y;
    
    %% 10. Обновление ковариации
    I = eye(size(F));
    P_upd = (I - K * H) * P_pred;
    
    %% 11. Возврат обновленных параметров
    new_x = X_upd(1);
    new_y = X_upd(2);
    new_vx = X_upd(3);
    new_vy = X_upd(4);
    new_v = sqrt(new_vx^2 + new_vy^2);
    new_theta = atan2(new_vy, new_vx);
    new_P = P_upd;
end