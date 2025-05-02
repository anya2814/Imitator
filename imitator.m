% Параметры
radius = 100; % Радиус окружности
center = [0, 0]; % Центр окружности
deltaT = 0.5; % Шаг замеров
falseObjProb = 0.3; % Вероятность увидеть ложную цель
curr_time = 0; % Общее время

% x_points - координаты по x
% y_points - координаты по y
% real_speed - реальная скорость на следующий отрезок (настоящий момент)
% viewed_speed - наблюдаемая скорость за счет пути поделенного на время за последний отрезок

figure;
hold on;
axis equal;
xlim([-radius, radius]);
ylim([-radius, radius]);
xlabel('X Axis');
ylabel('Y Axis');

% Параметры шумов (нормальное распределение)
muX = 0; muY = 0; muS = 0; % Cреднее значение (x - для координат x, y - для координат y, s - для real_speed) 
sigmaX = 1; sigmaY = 1; sigmaS = 2; % Новое стандартное отклонение

% Установка скорости объекта
n = 5; % Количество объектов
real_speed = { [10], [17], [30], [50], [35] }; % Скорость (метры в секунду), должно быть n значений

% Массивы для хранения предыдущих отметок
x_points = {};
y_points = {};


for i = 1:n
    time{i} = [0]; % Время замера
    viewed_speed{i} = [0]; % Наблюдаемая скорость
    x_points{i} = [0];
    y_points{i} = [0];
end

% Массивы для хранения параметров
theta = zeros(1,n);
x_position = zeros(1,n);
y_position = zeros(1,n);

% Основной цикл
while true
    % Новая отметка (с координатами на границе окружности)
    for i = 1:n
        if x_points{i} == [0] & isscalar(x_points{i})
            theta(i) = rand * 2 * pi;
            x_position(i) = radius * cos(theta(i));
            y_position(i) = radius * sin(theta(i)); 
            % Направление
            theta(i) = rand * 2 * pi;
            x_points{i}(1) = x_position(i);
            y_points{i}(1) = y_position(i);
        else
            x_points{i}(end+1) = x_position(i);
            y_points{i}(end+1) = y_position(i);
            time{i}(end+1) = curr_time;
            viewed_speed{i}(end+1) = sqrt((x_points{i}(end)-x_points{i}(end-1))^2+(y_points{i}(end)-y_points{i}(end-1))^2) / (time{i}(end)-time{i}(end-1));
            real_speed{i}(end+1) = real_speed{i}(end) + normrnd(muS, sigmaS);
        end
    end
    
    % Удаляем старые точки, оставляя только последние 50/deltaT
    for i = 1:n
        if length(x_points{i}) > 50/deltaT
            x_points{i} = x_points{i}(end-floor(50/deltaT):end);
            y_points{i} = y_points{i}(end-floor(50/deltaT):end);
        end
    end
    
    % Очистка текущего графика
    cla;
    
    % Рисуем окружность
    theta_circle = linspace(0, 2*pi, 100);
    plot(radius*cos(theta_circle), radius*sin(theta_circle), 'k-'); % Окружность
    plot(0, 0, 'o', 'Color', "#000000");

    % Рисуем ложные цели (на каждую отметку приходится в среднем falseObjProb ложных отметок)
    for i = 1:n
        if rand < falseObjProb
            theta0 = rand * 2 * pi;
            r0 = rand * radius;
            plot(r0 * cos(theta0), r0 * sin(theta0), '*', 'Color', "#000000"); 
        end
    end

    % Определение старых и новых точек
    for i = 1:n
        if length(x_points{i}) > 1
            % Соединяем линии для старых точек
            plot(x_points{i}, y_points{i}, 'c-*', 'LineWidth', 1.5, 'MarkerSize', 3); % Соединяем линии старых точек
        end
        % Отображаем новую точку (последнюю точку)
        if ~isempty(x_points{i})
            plot(x_points{i}(end), y_points{i}(end), 'b.-', 'MarkerSize', 15); % Новая отметка - синяя
        end
    end

    % Рассчитываем новое положение объекта
    curr_time = curr_time + deltaT;
    for i = 1:n
        x_position(i) = x_position(i) + real_speed{i}(end) * deltaT * cos(theta(i)) + normrnd(muX, sigmaX);
        y_position(i) = y_position(i) + real_speed{i}(end) * deltaT * sin(theta(i)) + normrnd(muY, sigmaY);
    end

    % Проверяем, не вышел ли объект за пределы радиуса
    for i = 1:n
        if sqrt(x_position(i)^2 + y_position(i)^2) >= radius
            x_points{i} = [0];
            y_points{i} = [0];
            time{i} = [curr_time];
            real_speed{i} = real_speed{i}(1);
        end
    end
    
    % Подождать deltaT секунд
    pause(deltaT);
end