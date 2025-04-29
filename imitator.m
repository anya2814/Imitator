% Параметры
radius = 1000; % Радиус окружности
center = [0, 0]; % Центр окружности
deltaT = 0.5; % Шаг замеров
n = 3; % Количество объектов
figure;
hold on;
axis equal;
xlim([-radius, radius]);
ylim([-radius, radius]);
xlabel('X Axis');
ylabel('Y Axis');

% Установка скорости объекта
speed1 = 80; % Постоянная скорость (единицы в секунду)
speed2 = 60;
speed3 = 100;
falseObjProb = 0.1; % Вероятность увидеть ложную цель

% Массивы для хранения предыдущих отметок
x1_points = [];
y1_points = [];
x2_points = [];
y2_points = [];
x3_points = [];
y3_points = [];

% Основной цикл
while true
    % Новая отметка (с координатами на границе окружности)
    if isempty(x1_points) 
        % Для позиции первой точки
        theta1 = rand * 2 * pi;
        x1_position = radius * cos(theta1);
        y1_position = radius * sin(theta1); 
        % Направление
        theta1 = rand * 2 * pi;
    end    

    if isempty(x2_points) 
        % Для позиции первой точки
        theta2 = rand * 2 * pi;
        x2_position = radius * cos(theta2);
        y2_position = radius * sin(theta2);
        % Направление
        theta2 = rand * 2 * pi;
    end   

    if isempty(x3_points) 
        % Для позиции первой точки
        theta3 = rand * 2 * pi;
        x3_position = radius * cos(theta3);
        y3_position = radius * sin(theta3);
        % Направление
        theta3 = rand * 2 * pi;
    end   

    % Обновление массивов с координатами
    x1_points(end + 1) = x1_position; 
    y1_points(end + 1) = y1_position;
    x2_points(end + 1) = x2_position; 
    y2_points(end + 1) = y2_position;
    x3_points(end + 1) = x3_position; 
    y3_points(end + 1) = y3_position;
    
    % Удаляем старые точки, оставляя только последние 200
    if length(x1_points) > 50/deltaT
        x1_points = x1_points(end-idivide(50,deltaT):end);
        y1_points = y1_points(end-idivide(50,deltaT):end);
    end
    if length(x2_points) > 50/deltaT
        x2_points = x2_points(end-idivide(50,deltaT):end);
        y2_points = y2_points(end-idivide(50,deltaT):end);
    end
    if length(x3_points) > 50/deltaT
        x3_points = x3_points(end-idivide(50,deltaT):end);
        y3_points = y3_points(end-idivide(50,deltaT):end);
    end
    
    % Очистка текущего графика
    cla;
    
    % Рисуем окружность
    theta_circle = linspace(0, 2*pi, 100);
    plot(radius*cos(theta_circle), radius*sin(theta_circle), 'k-'); % Окружность
    
    % Рисуем ложные цели
    for i = 1:n
        if rand < falseObjProb
            theta0 = rand * 2 * pi;
            r0 = rand * radius;
            r0 * cos(theta0)
            r0 * sin(theta0)
            plot(r0 * cos(theta0), r0 * sin(theta0), '*', 'Color', "#000000"); 
        end
    end

    % Определение старых и новых точек
    if length(x1_points) > 1
    % Соединяем линии для старых точек
        plot(x1_points(1:end), y1_points(1:end), 'c-*', 'LineWidth', 1.5, 'MarkerSize', 3); % Соединяем линии старых точек
    end
    % Отображаем новую точку (последнюю точку)
    if ~isempty(x1_points)
        plot(x1_points(end), y1_points(end), 'b.-', 'MarkerSize', 15); % Новая отметка - синяя
    end

    % Определение старых и новых точек
    if length(x2_points) > 1
    % Соединяем линии для старых точек
        plot(x2_points(1:end), y2_points(1:end), '-*', 'LineWidth', 1.5, 'MarkerSize', 3, 'Color', "#EDB120"); % Соединяем линии старых точек
    end
    % Отображаем новую точку (последнюю точку)
    if ~isempty(x2_points)
        plot(x2_points(end), y2_points(end), 'r.-', 'MarkerSize', 15); % Новая отметка - красная
    end

    % Определение старых и новых точек
    if length(x3_points) > 1
    % Соединяем линии для старых точек
        plot(x3_points(1:end), y3_points(1:end), 'y-*', 'LineWidth', 1.5, 'MarkerSize', 3); % Соединяем линии старых точек
    end
    % Отображаем новую точку (последнюю точку)
    if ~isempty(x3_points)
        plot(x3_points(end), y3_points(end), 'g.-', 'MarkerSize', 15); % Новая отметка - зеленая
    end

    % Рассчитываем новое положение объекта
    x1_position = x1_position + speed1 * deltaT * cos(theta1);
    y1_position = y1_position + speed1 * deltaT * sin(theta1);
    x2_position = x2_position + speed2 * deltaT * cos(theta2);
    y2_position = y2_position + speed2 * deltaT * sin(theta2);
    x3_position = x3_position + speed3 * deltaT * cos(theta3);
    y3_position = y3_position + speed3 * deltaT * sin(theta3);

    % Проверяем, не вышел ли объект за пределы радиуса
    if sqrt(x1_position^2 + y1_position^2) >= radius
        x1_points = [];
        y1_points = [];
    end
    if sqrt(x2_position^2 + y2_position^2) >= radius
        x2_points = [];
        y2_points = [];
    end
    if sqrt(x3_position^2 + y3_position^2) >= radius
        x3_points = [];
        y3_points = [];
    end
    
    % Подождать deltaT секунд
    pause(deltaT);
end