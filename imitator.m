% параметры
radius = 5000; % Радиус зоны обзора радара
deltaT = 0.017; % Шаг времени моделирования
falseObjProb = 0.05; % Вероятность появления ложной цели на каждую реальную цель
curr_time = 0; % Текущее время

% Графика
figure;
hold on;
axis equal;
xlim([-radius, radius]);
ylim([-radius, radius]);
xlabel('X Axis');
ylabel('Y Axis');

% Параметры шума
muX = 0; muY = 0;
sigmaX = 1; sigmaY = 1;

% Параметры наблюдений (с шумами)
muD = 0; muAngle = 0; muTime = 0;
sigmaD = sqrt(3); sigmaAngle = deg2rad(0.5); sigmaTime = sqrt(0.01);

% Радар 
radar_angle = 0; % Начальный угол луча
full_rotation_time = 4.5; % Полный оборот за 4.5 секунды
omega = 2 * pi / full_rotation_time; % Угловая скорость
angle_resolution_deg = 30; % Разрешение луча радара (в градусах)
angle_resolution_rad = deg2rad(angle_resolution_deg); % Переводим в радианы

% Цели 
n = 15;
real_speed =  [20, 30, 15, 27, 35, 20, 30, 15, 27, 35, 20, 30, 15, 27, 35]; % Скорости целей в метрах в секунду
min_speed = 10; max_speed = 40; % минимальная и максимальная скорость отслеживаемых объектов

% Хранение данных
true_x = cell(1,n);
true_y = cell(1,n);

% Трекеры, missedDetections - количество пропущенных отметок,
% notMissedDetections - количество последовательно полученных отметок,
% Инициализация 1 структуры с нужными полями
emptyObj = struct( ...
    'id', 0, ...
    'distance', [], ...
    'angle', [], ...  % угол положения цели в радианах
    'x', [], ...
    'y', [], ...
    'direction', [], ... 	% угол направления цели в декартовой системе координат
    'velocity', [], ...
    'countForAverage', 0, ...
    'missedDetections', 0, ...
    'timestamps', [], ...
    'P', eye(4));

% Пустой массив структур
tracks = repmat(emptyObj, 0, 1);

theta = zeros(1,n); % Направление движения
x_position = zeros(1,n);
y_position = zeros(1,n);

max_missed_frames = 5; % Максимальное число пропусков перед удалением

while true
    cla;

    % Массивы для хранения обнаруженных на данном шаге отметок (дистанция, угол, координаты и
    % время)
    viewed_d = [];
    viewed_angle = [];
    viewed_time = [];
    viewed_x = [];
    viewed_y = [];

    %Обновление радара
    radar_angle = mod(radar_angle + omega * deltaT, 2 * pi);

    % Рисуем окружность
    theta_circle = linspace(0, 2*pi, 50);
    plot(radius*cos(theta_circle), radius*sin(theta_circle), 'k--', 'HandleVisibility','off');
    plot(1000*cos(theta_circle), 1000*sin(theta_circle), 'k--', 'LineWidth', 0.2, 'HandleVisibility','off');
    plot(2000*cos(theta_circle), 2000*sin(theta_circle), 'k--', 'LineWidth', 0.2, 'HandleVisibility','off');
    plot(3000*cos(theta_circle), 3000*sin(theta_circle), 'k--', 'LineWidth', 0.2, 'HandleVisibility','off');
    plot(4000*cos(theta_circle), 4000*sin(theta_circle), 'k--', 'LineWidth', 0.2, 'HandleVisibility','off');
    plot([-radius radius], [0 0], 'k--', 'LineWidth', 0.2);
    plot([0 0], [-radius radius], 'k--', 'LineWidth', 0.2);
    plot(0, 0, '.', 'Color', "#000000", 'HandleVisibility','off');

    % Рисуем луч радара
    beam_x = radius * cos(radar_angle);
    beam_y = radius * sin(radar_angle);
    plot([0 beam_x], [0 beam_y], 'g-', 'LineWidth', 0.5, 'Color', "#000000");
    beam_x = radius * cos(radar_angle + angle_resolution_rad/2);
    beam_y = radius * sin(radar_angle + angle_resolution_rad/2);
    plot([0 beam_x], [0 beam_y], 'b-', 'LineWidth', 0.5);
    beam_x = radius * cos(radar_angle - angle_resolution_rad/2);
    beam_y = radius * sin(radar_angle - angle_resolution_rad/2);
    plot([0 beam_x], [0 beam_y], 'b-', 'LineWidth', 0.5, 'Color', "b");

    % Цикл
    for i = 1:n
        % если какая-то цель вышла за область видимости создаем новые
        % массивы для моделирования ее движения
        if isempty(true_x{i}) || sqrt(x_position(i)^2 + y_position(i)^2) >= radius
            start_angle = rand() * 2 * pi;
            x_position(i) = radius * cos(start_angle);
            y_position(i) = radius * sin(start_angle);
            theta(i) = mod(atan2(-y_position(i), -x_position(i)), 2*pi) + 2 * rand() - 1; % делаем чтобы цель двигалась на радар отклоняясь максимум на 1 радиан
            true_x{i}=[x_position(i)];
            true_y{i}=[y_position(i)];

        else % иначе находим ее координаты в соответствии со скоростью и направлением
            dx = real_speed(i) * deltaT * cos(theta(i)) + muX + sigmaX * randn * deltaT;
            dy = real_speed(i) * deltaT * sin(theta(i)) + muY + sigmaY * randn * deltaT;
            x_position(i) = x_position(i) + dx;
            y_position(i) = y_position(i) + dy;

            true_x{i}=[true_x{i}, x_position(i)];
            true_y{i}=[true_y{i}, y_position(i)];
            
            % рисуем реальную траекторию целей
            if length(true_x{i}) > 1
                plot(true_x{i}, true_y{i}, '--k');
            end
        end

        % попадание в луч радарa
        target_angle = mod(atan2(y_position(i), x_position(i)), 2*pi);
        angle_diff = min(mod(abs(target_angle - radar_angle), 2*pi), ...
                         mod(abs(radar_angle - target_angle), 2*pi));

        % если цель попала в луч радара вносим отметку в массив новых
        % отметок на текущем шаге (сделано)
        if angle_diff < angle_resolution_rad/2 
            distance = sqrt(x_position(i)^2 + y_position(i)^2) + muD + sigmaD * randn; 

            viewed_d = [viewed_d, distance];
            viewed_angle = [viewed_angle, target_angle + muAngle + sigmaAngle * randn];
            viewed_time = [viewed_time, curr_time + muTime + sigmaTime * randn];

            viewed_x = [viewed_x, distance*cos(viewed_angle(end))];
            viewed_y = [viewed_y, distance*sin(viewed_angle(end))];

            plot(viewed_x(end), viewed_y(end), 'o',...
                'MarkerFaceColor','g','MarkerEdgeColor','g','MarkerSize',10);

            %text(viewed_x{i}(end), viewed_y{i}(end), ...
                %sprintf('Angle: %.2f°\nDistance: %.2f', rad2deg(viewed_angle{i}(end)), distance), ...
                %'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
        end

        %Добавляем ложные отметки к массиву отметок полученных на данном
        %шаге (сделано)
        if rand < (falseObjProb * angle_resolution_deg / 180)

            theta0 = radar_angle + (2 * rand() - 1) * (angle_resolution_rad / 2); 
            r0 = sqrt(rand) * radius;

            viewed_d = [viewed_d, r0];
            viewed_angle = [viewed_angle, theta0];
            viewed_time = [viewed_time, curr_time + muTime + sigmaTime * randn];

            viewed_x = [viewed_x, r0*cos(viewed_angle(end))];
            viewed_y = [viewed_y, r0*sin(viewed_angle(end))];

            plot(viewed_x(end), viewed_y(end), '*', 'Color', "black"); 
        end

    end 
    
    if ~isempty(viewed_x)
        %% сначала делаем отождествление полученных отметок с существующими траекториями
        % пробегаемся по имеющимся траекториям и каждой присваиваем 
        % ближайшую к предсказанной отметку от радара, если она
        % удовлетворяет условиям о близости

        for t_id = 1:length(tracks)
            bestId = -1;
            if length(tracks(t_id).x) > 1 % проверка что в треке есть хотя бы 2 точки
	            minDist = Inf;
                % по старым x и y вычисляем предсказание x и y, и по ним
                % предсказание дистанции
                pred_x = tracks(t_id).x(end) + tracks(t_id).velocity(end) * (curr_time - tracks(t_id).timestamps(end)) * cos(tracks(t_id).direction(end));
                pred_y = tracks(t_id).y(end) + tracks(t_id).velocity(end) * (curr_time - tracks(t_id).timestamps(end)) * sin(tracks(t_id).direction(end));
                pred_d = sqrt(pred_x^2 + pred_y^2);
                pred_angle = mod(atan2(pred_y, pred_x), 2*pi);

                % ищем минимальное расстояние 
                for i = length(viewed_x):-1:1
                    if abs(viewed_d(i)-pred_d) <= 6*sigmaD && abs(viewed_angle(i)-pred_angle) <= 6*sigmaAngle
                        dist = sqrt((pred_x-viewed_x(i))^2+(pred_y-viewed_y(i)));
                        if dist < minDist
                            minDist = dist;
                            bestId = i;
                        end
                    end
                end

                if bestId ~= -1
                    % добавляем лучшую отметку
    
                    % если нужно усреднять с предыдущими
                    if curr_time - tracks(t_id).timestamps(end) < 0.7*full_rotation_time

                        % Обновляем счётчики
                        tracks(t_id).countForAverage = tracks(t_id).countForAverage + 1;
    
                        % Усредняем отметку с countForAverage-1 последними отметками
                        tracks(t_id).distance(end) = (tracks(t_id).distance(end)*(tracks(t_id).countForAverage-1) + viewed_d(bestId))/tracks(t_id).countForAverage;
                        tracks(t_id).angle(end) = (tracks(t_id).angle(end)*(tracks(t_id).countForAverage-1) + viewed_angle(bestId))/tracks(t_id).countForAverage;
                        tracks(t_id).x(end) = (tracks(t_id).x(end)*(tracks(t_id).countForAverage-1) + viewed_x(bestId))/tracks(t_id).countForAverage;
                        tracks(t_id).y(end) = (tracks(t_id).y(end)*(tracks(t_id).countForAverage-1) + viewed_y(bestId))/tracks(t_id).countForAverage;
        
                        % Усредняем временную метку
                        tracks(t_id).timestamps(end) = (tracks(t_id).timestamps(end)*(tracks(t_id).countForAverage-1) + curr_time)/tracks(t_id).countForAverage;
                        
                        % Замеры скорости и направления
                        dx = diff(tracks(t_id).x(end-1:end));
                        dy = diff(tracks(t_id).y(end-1:end));
                        dt = diff(tracks(t_id).timestamps(end-1:end));
                        tracks(t_id).velocity(end) = hypot(dx, dy) / dt;
                        tracks(t_id).direction(end) = mod(atan2(dy, dx), 2*pi);
                        
                    else % если не нужно усреднять
                        tracks(t_id).distance(end+1) = viewed_d(bestId);
                        tracks(t_id).angle(end+1) = viewed_angle(bestId);
                        tracks(t_id).x(end+1) = viewed_x(bestId);
                        tracks(t_id).y(end+1) = viewed_y(bestId);
        
                        % Добавляем временную метку
                        tracks(t_id).timestamps(end+1) = curr_time;
    
                        % Добавляем в трек замеры скорости и направления
                        dx = diff(tracks(t_id).x(end-1:end));
                        dy = diff(tracks(t_id).y(end-1:end));
                        dt = diff(tracks(t_id).timestamps(end-1:end));
                        tracks(t_id).velocity(end+1)  = hypot(dx, dy) / dt;
                        tracks(t_id).direction(end+1) = mod(atan2(dy, dx), 2*pi);
                        
                        % Обновляем счётчики
                        tracks(t_id).countForAverage = 1;
                    
                    end
                    
                    tracks(t_id).missedDetections = 0;
    
                    % Удаляем отождествленную отметку из массива новых отметок
                    viewed_d(bestId) = [];
                    viewed_angle(bestId) = [];
                    viewed_time(bestId) = [];
                    viewed_x(bestId) = [];
                    viewed_y(bestId) = [];
    
                else
                    % проверяем, попадает ли предсказанное значение отметки в
                    % область видимости радара
                    angle_diff = min(mod(abs(pred_angle - radar_angle), 2*pi), ...
                             mod(abs(radar_angle - pred_angle), 2*pi));
                    if angle_diff < angle_resolution_rad/2 && (curr_time-tracks(t_id).timestamps(end)) > (full_rotation_time*1.5)
                        % если предсказанное положение цели попадает в область видимости но
                        % подходящую отметку не нашли, и прошел полный оборот, в треке пропуск отметки
                        tracks(t_id).missedDetections = tracks(t_id).missedDetections + 1;
                    end
                end
            end
        end

        %% Затем находим вторые точки для траектории (завязка траектории)
        for t_id = length(tracks):-1:1
            if length(tracks(t_id)) == 1 % проверка что в треке 1 точка
                flag = 0; % для первой подходящей точки записываем в этот же трек, 
                            % для других подходящих копируем трек
                % проверяем расстояние 
                for i = length(viewed_x):-1:1                   
                    distance = sqrt((viewed_x(i) - tracks(t_id).x(end))^2 + (viewed_y(i) - tracks(t_id).y(end))^2);
                    max_distance = max_speed*(curr_time-tracks(t_id).timestamps(end)) + 0.03*sqrt(viewed_x(i)^2+viewed_y(i)^2); % максимальная дистанция для попадания в строб
                    min_distance = min_speed*(curr_time-tracks(t_id).timestamps(end)) - 0.03*sqrt(viewed_x(i)^2+viewed_y(i)^2); % минимальная дистанция для попадания в строб
                    
                    if distance < max_distance && distance > min_distance
                        % если нужно усреднять с предыдущими, то это все еще первая точка
                        if curr_time - tracks(t_id).timestamps(1) < 0.7*full_rotation_time
                            % Обновляем счётчики
                            tracks(t_id).countForAverage = tracks(t_id).countForAverage + 1;
    
                            % Усредняем отметку с countForAverage-1 последними отметками
                            tracks(t_id).distance(end) = (tracks(t_id).distance(end)*(tracks(t_id).countForAverage-1) + viewed_d(i))/tracks(t_id).countForAverage;
                            tracks(t_id).angle(end) = (tracks(t_id).angle(end)*(tracks(t_id).countForAverage-1) + viewed_angle(i))/tracks(t_id).countForAverage;
                            tracks(t_id).x(end) = (tracks(t_id).x(end)*(tracks(t_id).countForAverage-1) + viewed_x(i))/tracks(t_id).countForAverage;
                            tracks(t_id).y(end) = (tracks(t_id).y(end)*(tracks(t_id).countForAverage-1) + viewed_y(i))/tracks(t_id).countForAverage;
        
                            % Усредняем временную метку
                            tracks(t_id).timestamps(end) = (tracks(t_id).timestamps(end)*(tracks(t_id).countForAverage-1) + curr_time)/tracks(t_id).countForAverage;

                        else % если не нужно усреднять
                            %if flag % Копируем трек
                            %    new_track = tracks(t_id);  % Копирование всех полей текущего трека
                            %    new_track.id = max([tracks.id]) + 1;  % Увеличиваем ID для нового трека
                            %    t_id = new_track.id;
                            %    tracks(end + 1) = new_track;  % Добавляем новый трек в массив tracks
                            %end % Вставляем точку

                            tracks(t_id).distance(end+1) = viewed_d(i);
                            tracks(t_id).angle(end+1) = viewed_angle(i);
                            tracks(t_id).x(end+1) = viewed_x(i);
                            tracks(t_id).y(end+1) = viewed_y(i);
            
                            % Добавляем временную метку
                            tracks(t_id).timestamps(end+1) = curr_time;
        
                            % Добавляем в трек замеры скорости и направления
                            dx = diff(tracks(t_id).x(end-1:end));
                            dy = diff(tracks(t_id).y(end-1:end));
                            dt = diff(tracks(t_id).timestamps(end-1:end));
                            tracks(t_id).velocity(end+1)  = hypot(dx, dy) / dt;
                            tracks(t_id).direction(end+1) = mod(atan2(dy, dx), 2*pi);
                            
                            % Обновляем счётчики
                            tracks(t_id).countForAverage = 1;
                        end
                    
                        tracks(t_id).missedDetections = 0;
        
                        % Удаляем отождествленную отметку из массива новых отметок
                        viewed_d(i) = [];
                        viewed_angle(i) = [];
                        viewed_time(i) = [];
                        viewed_x(i) = [];
                        viewed_y(i) = [];

                        flag = 1;
                    end
                end
                
                % Если не нашли и с момента последней отмекти прошло full_rotation_time*1.5.
                % Получается если на следующем обороте не нашли вторую
                % отметку то удаляем траекторию
                if flag == 0 && (curr_time-tracks(t_id).timestamps(end)) > (full_rotation_time*1.5)
                    tracks(t_id) = [];
                end
            end
        end
    
        %% Оставшиеся точки сохраняем как возможные стартовые для новой траектории
        for i = 1:length(viewed_x)
            new_track = emptyObj;
            if ~isempty(tracks) && isstruct(tracks)
                new_track.id = max([tracks.id]) + 1;  % уникальный ID
            else
                new_track.id = 1; 
            end
            new_track.x = [viewed_x(i)];
            new_track.y = [viewed_y(i)];
            new_track.distance = [viewed_d(i)];
            new_track.angle = [viewed_angle(i)];
            new_track.timestamps = curr_time;
            new_track.direction = [NaN]; 	% угол направления и скорость пока неизвестны
            new_track.velocity = [NaN];
            new_track.countForAverage = 1;
            new_track.P = eye(4);
            tracks(end+1) = new_track;
        end
    end

    % удаление старых треков
    if ~isempty(tracks) && isstruct(tracks)
        tracks = tracks([tracks.missedDetections] < max_missed_frames);
    end
    
    % Здесь фильтр Калмана
    for t_id = 1:length(tracks)
        if length(tracks(t_id).x) > 2
            dt = tracks(t_id).timestamps(end) - tracks(t_id).timestamps(end-1);
            [new_x, new_y, new_theta, new_v, new_P] = kalmanFilter(tracks(t_id).x(end-1), tracks(t_id).y(end-1), tracks(t_id).direction(end-1), tracks(t_id).velocity(end-1), ...
                tracks(t_id).x(end),  tracks(t_id).y(end), tracks(t_id).direction(end), tracks(t_id).velocity(end), tracks(t_id).P, dt);

            tracks(t_id).x(end) = new_x;
            tracks(t_id).y(end) = new_y;
            tracks(t_id).velocity(end) = new_v;
            tracks(t_id).direction(end) = new_theta;
            tracks(t_id).P = new_P;
        end
    end

    % анимация
    for t_id = 1:length(tracks)
        plot(tracks(t_id).x, tracks(t_id).y, '-o', ...
             'MarkerFaceColor', "#80B3FF", 'Color', "k", 'LineWidth', 2, 'MarkerEdgeColor',"k", 'MarkerSize',4);
        if length(tracks(t_id).x) > 1
            plot(tracks(t_id).x(end), tracks(t_id).y(end), '-o', ...
                'MarkerFaceColor','blue', 'Color', "k", 'MarkerEdgeColor',"#80B3FF", 'MarkerSize',7);
            text(tracks(t_id).x(end), tracks(t_id).y(end), num2str(tracks(t_id).id), ...
             'VerticalAlignment','bottom', 'HorizontalAlignment','right');
        else
            plot(tracks(t_id).x(end), tracks(t_id).y(end), '-o', ...
                'MarkerFaceColor', [0.5 0.5 0.5], 'Color', "k", 'MarkerEdgeColor', [0.5 0.5 0.5], 'MarkerSize',3);
        end
    end

    drawnow;
    if curr_time > 30
        stop = 0; % Чтобы смотреть результат
    end
    curr_time = curr_time + deltaT;
    
end