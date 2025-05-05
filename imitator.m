% Параметры
radius = 100; % Радиус окружности
center = [0, 0]; % Центр окружности
deltaT = 0.017; % Шаг времени моделирования
falseObjProb = 0.3; % Вероятность ложной цели
curr_time = 0; % Текущее время

figure;
hold on;
axis equal;
xlim([-radius, radius]);
ylim([-radius, radius]);
xlabel('X Axis');
ylabel('Y Axis');

% Параметры шумов
muX = 0; muY = 0; muS = 0;
sigmaX = 1; sigmaY = 1; sigmaS = 2;

% Радар
radar_angle = 0; % Начальный угол луча
full_rotation_time = 4.5; % Полный оборот за 4.5 секунды
omega = 2 * pi / full_rotation_time; % Угловая скорость радара
angle_resolution_deg = 30; % Разрешение луча радара (в градусах)
angle_resolution_rad = deg2rad(angle_resolution_deg); % Переводим в радианы

% Количество целей
n = 5;
real_speed = { [20], [25], [30], [40], [10] }; % Скорости целей
viewed_speed = cell(1,n);

% Массивы для хранения данных
true_x = cell(1,n);
true_y = cell(1,n);

viwed_x = cell(1,n);
viewed_y = cell(1,n);
time = cell(1,n);

% Храним графические объекты
intersection_markers = cell(1,n); % Для зелёных кружков пересечения

% Параметры движения целей
theta = zeros(1,n); % Направление движения
x_position = zeros(1,n);
y_position = zeros(1,n);

% Основной цикл
while true
    cla;

    % Рисуем окружность
    theta_circle = linspace(0, 2*pi, 100);
    plot(radius*cos(theta_circle), radius*sin(theta_circle), 'k-', 'HandleVisibility','off');
    plot(0, 0, 'o', 'Color', "#000000", 'HandleVisibility','off');

    % Рисуем луч радара
    beam_x = radius * cos(radar_angle);
    beam_y = radius * sin(radar_angle);
    plot([0 beam_x], [0 beam_y], '--', 'LineWidth', 0.5, 'Color', "#000000");
    beam_x = radius * cos(radar_angle+angle_resolution_rad/2);
    beam_y = radius * sin(radar_angle+angle_resolution_rad/2);
    plot([0 beam_x], [0 beam_y], 'b--', 'LineWidth', 0.5);
    beam_x = radius * cos(radar_angle-angle_resolution_rad/2);
    beam_y = radius * sin(radar_angle-angle_resolution_rad/2);
    plot([0 beam_x], [0 beam_y], 'b--', 'LineWidth', 0.5, 'Color', "b");

    % Обработка целей
    for i = 1:n
        if isempty(true_x{i}) || sqrt(x_position(i)^2 + y_position(i)^2) >= radius
            start_angle = rand() * 2 * pi;
            x_position(i) = radius * cos(start_angle);
            y_position(i) = radius * sin(start_angle);
            theta(i) = rand() * 2 * pi;
            real_speed{i} = [real_speed{i}(1)];
            true_x{i}=[x_position(i)];
            true_y{i}=[y_position(i)];
            viewed_x{i}=[];
            viewed_y{i}=[];
            time{i} = [curr_time];
        else
            dx = real_speed{i}(end) * deltaT * cos(theta(i)) + muX + sigmaX * randn* deltaT;
            dy = real_speed{i}(end) * deltaT * sin(theta(i)) + muY + sigmaY * randn* deltaT;
            x_position(i) = x_position(i) + dx;
            y_position(i) = y_position(i) + dy;

            true_x{i}=[true_x{i}, x_position(i)];
            true_y{i}=[true_y{i}, y_position(i)];

            if length(true_x{i}) > 1
                plot(true_x{i}, true_y{i}, '--k');
            end
        end

        target_angle=mod(atan2(y_position(i), x_position(i)),2*pi);
        angle_diff=min(mod(abs(target_angle - radar_angle),2*pi),...
                       mod(abs(radar_angle - target_angle),2*pi));

        if angle_diff < angle_resolution_rad/2 
            distance=sqrt(x_position(i)^2 + y_position(i)^2); 
            
            x_points{i}=[x_points{i}, x_position(i)+sigmaX*randn*deltaT];
            y_points{i}=[y_points{i}, y_position(i)+sigmaY*randn*deltaT];
            time{i}=[time{i}, curr_time];
            viewed_x{i}=[viewed_x{i}, x_points{i}(end)];
            viewed_y{i}=[viewed_y{i}, y_points{i}(end)];

            plot(x_points{i}(end),y_points{i}(end),'b.','MarkerSize',15);
            
            if ~isempty(intersection_markers{i}) && ishandle(intersection_markers{i})
                delete(intersection_markers{i});
            end
            
            intersection_markers{i}=plot(x_position(i),y_position(i),'o',...
                'MarkerFaceColor','g','MarkerEdgeColor','g','MarkerSize',10);
            
            % Выводим измеренные угол и расстояние на график 
            text(x_points{i}(end), y_points{i}(end), ...
                sprintf('Angle: %.2f°\nDistance: %.2f', rad2deg(target_angle), distance), ...
                'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
        else 
            if ~isempty(intersection_markers{i}) && ishandle(intersection_markers{i})
                delete(intersection_markers{i});
            end
            
            intersection_markers{i} =[];
        end 
        plot(viewed_x{i},viewed_y{i},'g', 'LineStyle', "None", 'MarkerSize', 10, 'Marker', ".");
    end 

    for i=1:n 
        if rand<falseObjProb 
            theta0=rand*2*pi; 
            r0=sqrt(rand)*radius; 
            plot(r0*cos(theta0),r0*sin(theta0),'*','Color',"#000000"); 
        end 
    end 

    radar_angle=mod(radar_angle+omega*deltaT,2*pi);

    drawnow;
    curr_time=curr_time+deltaT;
end 