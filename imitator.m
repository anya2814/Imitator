% параметры
radius = 5000; % Радиус зоны обзора радара
deltaT = 0.017; % Шаг времени моделирования
lambda = 0.1/(1000^2); % интенсивность появления ложных отметок на 1000 км^2 
curr_time = 0; % Текущее время

% Графика
figure;
hold on;
axis equal;
xlim([-radius, radius]);
ylim([-radius, radius]);
xlabel('X Axis');
ylabel('Y Axis');

% Для расчета
valid_ties = 0; % подсчет правильных завязок
trajectories_count = 0; % всего траекторий от реальных целей
false_marks_count = 0; % подсчет ложных отметок
false_ties = 0; % подсчет ложных завязок

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

% Хранение данных о реальных отметках и их отождествлении
emptyTrue = struct( ...
    'x', [], ...
    'y', [], ...
    'assigned', [], ... % массив с отметками о том отслежена ли отметка от цели и присвоена правильному треку или нет 
    'track_id', [], ...    % массив о том какому треку была присвоена отметка на предыдущем шаге
    'prev_track_nums', [], ... % массив с номерами треков с которыми отождествляли отметки от цели
    'viewed', []);      % попала ли отметка в луч радара (для наглядности проверки)

% Массив структур (размер n)
true_data = repmat(emptyTrue, n, 1);

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
    'P', eye(4), ...
    'filtered', 0); % чтобы не фильтровать одну и ту же точку

% Пустой массив структур
tracks = repmat(emptyObj, 0, 1);

theta = zeros(1,n); % Направление движения
x_position = zeros(1,n);
y_position = zeros(1,n);

max_missed_frames = 5; % Максимальное число пропусков перед удалением

% Флаг: рисовать график или нет
DRAW_GRAPHICS = 0;

%% Блок для создания графики
    hold on;

    % --- Истинные треки ---
    h_true = gobjects(1, n);
    for k = 1:n
        h_true(k) = plot(nan, nan, '--k');
    end

    % --- Просмотренные цели ---
    h_viewed = plot(nan, nan, 'o', 'MarkerFaceColor','g','MarkerEdgeColor','g','MarkerSize', 10, 'LineWidth', 0.5);

    % --- Окружности ---
    theta = linspace(0, 2*pi, 50);
    radii = [radius*0.2 radius*0.4 radius*0.6 radius*0.8 radius];
    h_circles = gobjects(1, length(radii));
    for r_id = 1:length(radii)
        h_circles(r_id) = plot(radii(r_id)*cos(theta), radii(r_id)*sin(theta), 'k--', ...
                               'LineWidth', 0.2, 'HandleVisibility','off');
    end
    plot([-radius radius], [0 0], 'k--', 'LineWidth', 0.2);
    plot([0 0], [-radius radius], 'k--', 'LineWidth', 0.2);
    plot(0, 0, '.', 'Color', "#000000", 'HandleVisibility','off');

    % --- Луч радара ---
    h_beam(1) = plot(nan, nan, '-', 'Color', "#000000", 'LineWidth', 0.5); % центральный луч
    h_beam(2) = plot(nan, nan, '-', 'Color', "b", 'LineWidth', 0.5); % +angle_resolution
    h_beam(3) = plot(nan, nan, '-', 'Color', "b", 'LineWidth', 0.5); % -angle_resolution

    % --- Треки (максимум N_TRACKS треков) ---
    N_TRACKS = 200;
    h_tracks = gobjects(1, N_TRACKS);
    for k = 1:N_TRACKS
        h_tracks(k) = plot(nan, nan, '-o', ...
            'MarkerFaceColor', "#80B3FF", 'Color', "k", ...
            'LineWidth', 2, 'MarkerEdgeColor',"k", 'MarkerSize', 4, 'LineWidth', 0.5);
    end

while true

    % Массивы для хранения обнаруженных на данном шаге отметок (дистанция, угол, координаты и
    % время)
    viewed_d = [];
    viewed_angle = [];
    viewed_time = [];
    viewed_x = [];
    viewed_y = [];
    viewed_assigned = []; % массив чтобы показать какие из отметок уже были присвоены какой-либо траектории
    viewed_realnum = []; % показывает цели с каким номером соответствует отметка

    %Обновление радара
    radar_angle = mod(radar_angle + omega * deltaT, 2 * pi);

    % Цикл
    for i = 1:n
        % если какая-то цель вышла за область видимости создаем новые
        % массивы для моделирования ее движения
        r = sqrt(x_position(i)^2 + y_position(i)^2);

        if r >= radius && length(true_data(i).track_id) > 100
            is_valid = computeTrackingMask(true_data(i).track_id);
            valid_ties = valid_ties + is_valid;
            trajectories_count = trajectories_count + 1;
        
            updateGraphics(true_data, h_true, radar_angle, angle_resolution_rad, ...
                           radius, h_beam, tracks, h_tracks, N_TRACKS);
        
            % --- Если накопили 50 завершённых траекторий ---
            if trajectories_count == 50
                ratio = valid_ties / trajectories_count;
        
                % --- Запись в файл ---
                filename = 'valid_ratio.txt';
                fid = fopen(filename, 'a');  % append
                fprintf(fid, '%.6f\n', ratio);
                fclose(fid);
        
                % --- Обнулить счётчики ---
                valid_ties = 0;
                trajectories_count = 0;
            end
        end

        if isempty(true_data(i).x) || r >= radius
            % новая цель
            start_angle = rand() * 2 * pi;
            x_position(i) = radius * cos(start_angle);
            y_position(i) = radius * sin(start_angle);
            theta(i) = mod(atan2(-y_position(i), -x_position(i)), 2*pi) + 2 * rand() - 1;
    
            true_data(i).x = x_position(i);
            true_data(i).y = y_position(i);
            true_data(i).assigned = 0;
            true_data(i).track_id = nan;
            true_data(i).prev_track_nums = [];
            true_data(i).viewed = 0;

        else % иначе находим ее координаты в соответствии со скоростью и направлением
            dx = real_speed(i) * deltaT * cos(theta(i)) + muX + sigmaX * randn * deltaT;
            dy = real_speed(i) * deltaT * sin(theta(i)) + muY + sigmaY * randn * deltaT;
            x_position(i) = x_position(i) + dx;
            y_position(i) = y_position(i) + dy;

            true_data(i).x(end+1) = x_position(i);
            true_data(i).y(end+1) = y_position(i);
            true_data(i).assigned(end+1) = 0;
            true_data(i).track_id(end+1) = nan;
            true_data(i).viewed(end+1) = 0;
            
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
            viewed_assigned = [viewed_assigned, false];
            viewed_realnum = [viewed_realnum, i]; % в этой ячейке viewed отметка от i-той цели
            true_data(i).viewed(end) = 1;
        end

    end 

    if false_marks_count > 1000
        ratio = false_ties / false_marks_count;
    
        filename = 'false_tracking_ratios.txt';
    
        % Открываем файл для дозаписи, записываем и закрываем
        fid = fopen(filename, 'a');  
        fprintf(fid, '%.6f\n', ratio);
        fclose(fid);
    
        % Обнуляем счетчики
        false_ties = 0;
        false_marks_count = 0;
    end
        
    N_false = poissrnd(lambda*radius^2*angle_resolution_rad/(2*pi)); % Количество ложных целей
    false_marks_count = false_marks_count + N_false; % подсчет ложных отметок

    %Добавляем ложные отметки к массиву отметок полученных на данном шаге 
    if N_false > 0
        % Генерация углов (равномерно внутри текущего сектора обзора)
        theta0 = radar_angle + (2 * rand(1, N_false) - 1) * (angle_resolution_rad / 2);
    
        % Генерация расстояний (равномерно по площади круга)
        r0 = sqrt(rand(1, N_false)) * radius;
    
        % Время обнаружения (нормальное распределение)
        new_time = curr_time + muTime + sigmaTime * randn(1, N_false);
    
        % Координаты
        new_x = r0 .* cos(theta0);
        new_y = r0 .* sin(theta0);
    
        % Добавление в массивы
        viewed_d = [viewed_d, r0];
        viewed_angle = [viewed_angle, theta0];
        viewed_time = [viewed_time, new_time];
        viewed_x = [viewed_x, new_x];
        viewed_y = [viewed_y, new_y];
        viewed_assigned = [viewed_assigned, false(1, N_false)];
        viewed_realnum = [viewed_realnum, nan(1, N_false)];
        
    end
    
    % --- Обновляем просмотренные цели ---
    set(h_viewed, 'XData', viewed_x, 'YData', viewed_y);

    if ~isempty(viewed_x)
        %% сначала делаем отождествление полученных отметок с существующими траекториями
        % пробегаемся по имеющимся траекториям и каждой присваиваем 
        % ближайшую к предсказанной отметку от радара, если она
        % удовлетворяет условиям о близости

        % находим сразу разницу между последними измерениями времени
        % отметок в треках и текущим временем
        time_since_last = curr_time - arrayfun(@(tracks) tracks.timestamps(end), tracks);

        for t_id = 1:length(tracks)
            bestId = -1;
            if length(tracks(t_id).x) > 1 % проверка что в треке есть хотя бы 2 точки
	            minDist = Inf;
                % по старым x и y вычисляем предсказание x и y, и по ним
                % предсказание дистанции
                pred_x = tracks(t_id).x(end) + tracks(t_id).velocity(end) * (time_since_last(t_id)) * cos(tracks(t_id).direction(end));
                pred_y = tracks(t_id).y(end) + tracks(t_id).velocity(end) * (time_since_last(t_id)) * sin(tracks(t_id).direction(end));
                pred_d = sqrt(pred_x^2 + pred_y^2);
                pred_angle = mod(atan2(pred_y, pred_x), 2*pi);

                % ищем минимальное расстояние 
                for i = 1:length(viewed_x)
                    if abs(viewed_d(i)-pred_d) <= 6*sigmaD && abs(viewed_angle(i)-pred_angle) <= 6*sigmaAngle && ~viewed_assigned(i)
                        dist = sqrt((pred_x-viewed_x(i))^2+(pred_y-viewed_y(i)));
                        if dist < minDist
                            minDist = dist;
                            bestId = i;
                        end
                    end
                end

                if bestId ~= -1
                        
    
                    % если нужно усреднять с предыдущими
                    if time_since_last(t_id) < 0.7*full_rotation_time
                        % Обновляем счётчики
                        tracks(t_id) = updateTrackAverage(tracks(t_id), viewed_x(bestId), viewed_y(bestId), viewed_d(bestId), viewed_angle(bestId), curr_time);
                        
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

                    % добавляем лучшую отметку
                    viewed_assigned(bestId) = true;
                    obj_num = viewed_realnum(bestId); % какому номеру реальной цели соответствует отметка или nan
                    if ~isnan(obj_num) && any( true_data(obj_num).prev_track_nums == tracks(t_id).id) % проверка была ли данная цель когда либо отождествлена с данным треком
                        true_data(obj_num).assigned(end) = 1;
                        true_data(obj_num).track_id(end) = tracks(t_id).id; % какому треку присвоена отметка
                    elseif ~isnan(obj_num)
                        true_data(obj_num).prev_track_nums(end+1) =  tracks(t_id).id; % если нет то запоминаем номер трека
                        true_data(obj_num).track_id(end) = tracks(t_id).id; % какому треку присвоена отметка
                    end 

                    tracks(t_id).missedDetections = 0;
        
                else
                    % проверяем, попадает ли предсказанное значение отметки в
                    % область видимости радара
                    angle_diff = min(mod(abs(pred_angle - radar_angle), 2*pi), ...
                             mod(abs(radar_angle - pred_angle), 2*pi));
                    if angle_diff < angle_resolution_rad/2 && (time_since_last(t_id)) > (full_rotation_time*1.5)
                        % если предсказанное положение цели попадает в область видимости но
                        % подходящую отметку не нашли, и прошел полный оборот, в треке пропуск отметки
                        tracks(t_id).missedDetections = tracks(t_id).missedDetections + 1;
                    end
                end
            end
        end

        % Сначала находим вторые точки для треков
        expired_track_ids = false(1, length(tracks)); % логический вектор для пометки

        % Удаляем assigned
        viewed_d         = viewed_d(~viewed_assigned);
        viewed_angle     = viewed_angle(~viewed_assigned);
        viewed_time      = viewed_time(~viewed_assigned);
        viewed_x         = viewed_x(~viewed_assigned);
        viewed_y         = viewed_y(~viewed_assigned);
        viewed_assigned  = viewed_assigned(~viewed_assigned);

        %% Затем находим вторые точки для траектории (завязка траектории)
        for t_id = 1:length(tracks)
            if length(tracks(t_id)) == 1 % проверка что в треке 1 точка
                flag = 0; % для первой подходящей точки записываем в этот же трек, 
                            % для других подходящих копируем трек
                % проверяем расстояние 
                for i = 1:length(viewed_x)                 
                    distance = sqrt((viewed_x(i) - tracks(t_id).x(end))^2 + (viewed_y(i) - tracks(t_id).y(end))^2);
                    max_distance = max_speed*(curr_time-tracks(t_id).timestamps(end)) + 0.03*sqrt(viewed_x(i)^2+viewed_y(i)^2); % максимальная дистанция для попадания в строб
                    min_distance = min_speed*(curr_time-tracks(t_id).timestamps(end)) - 0.03*sqrt(viewed_x(i)^2+viewed_y(i)^2); % минимальная дистанция для попадания в строб
                    
                    if distance < max_distance && distance > min_distance
                        % если нужно усреднять с предыдущими, то это все еще первая точка
                        if curr_time - tracks(t_id).timestamps(1) < 0.7*full_rotation_time
                            % Обновляем счётчики
                            tracks(t_id) = updateTrackAverage(tracks(t_id), viewed_x(i), viewed_y(i), viewed_d(i), viewed_angle(i), curr_time);

                        else % если не нужно усреднять
                            %if flag % Копируем трек
                            %    new_track = tracks(t_id);  % Копирование всех полей текущего трека
                            %    new_track.id = max([tracks.id]) + 1;  % Увеличиваем ID для нового трека
                            %    t_id = new_track.id;
                            %    tracks(end + 1) = new_track;  % Добавляем новый трек в массив tracks
                            %end % Вставляем точку

                            % проверка на ложную завязку
                            obj_num = viewed_realnum(i); % какому номеру реальной цели соответствует отметка или nan
                            if isnan(obj_num)
                                false_ties = false_ties + 1;
                            end

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
        
                        % Отмечаем что точка присвоена траектории
                        viewed_assigned(i) = true;
                        obj_num = viewed_realnum(i); % какому номеру реальной цели соответствует отметка или nan
                        if ~isnan(obj_num) && any( true_data(obj_num).prev_track_nums == tracks(t_id).id) % проверка была ли данная цель когда либо отождествлена с данным треком
                            true_data(obj_num).assigned(end) = 1;
                            true_data(obj_num).track_id(end) = tracks(t_id).id; % какому треку присвоена отметка
                        elseif ~isnan(obj_num)
                            true_data(obj_num).prev_track_nums(end+1) =  tracks(t_id).id; % если нет то запоминаем номер трека
                            true_data(obj_num).track_id(end) = tracks(t_id).id; % какому треку присвоена отметка
                        end 
    
                        tracks(t_id).missedDetections = 0;
                        flag = 1;
                    end
                end
                
                % Если не нашли и с момента последней отмекти прошло full_rotation_time*1.5.
                % Получается если на следующем обороте не нашли вторую
                % отметку то удаляем траекторию
                if flag == 0 && (time_since_last(t_id)) > (full_rotation_time*1.5)
                    expired_track_ids(t_id) = true;
                end
            end
        end

    % Удаляем траектории
    tracks(expired_track_ids) = [];

     % удаление старых треков
    if ~isempty(tracks) && isstruct(tracks)
        tracks = tracks([tracks.missedDetections] < max_missed_frames);
    end

    % Удаляем assigned
        viewed_d         = viewed_d(~viewed_assigned);
        viewed_angle     = viewed_angle(~viewed_assigned);
        viewed_time      = viewed_time(~viewed_assigned);
        viewed_x         = viewed_x(~viewed_assigned);
        viewed_y         = viewed_y(~viewed_assigned);
        viewed_assigned  = viewed_assigned(~viewed_assigned);
        viewed_realnum   = viewed_realnum(~viewed_assigned);

    %% Оставшиеся точки сохраняем как возможные стартовые для новой траектории
        nNew = length(viewed_x);
        if isempty(tracks) || ~isstruct(tracks)
            last_id = 0;
        else
            last_id = max([tracks.id]);
        end
        
        new_tracks = repmat(emptyObj, 1, nNew);
        
        for i = 1:nNew
            new_tracks(i).id = last_id + i;
            new_tracks(i).x = viewed_x(i);
            new_tracks(i).y = viewed_y(i);
            new_tracks(i).distance = viewed_d(i);
            new_tracks(i).angle = viewed_angle(i);
            new_tracks(i).timestamps = curr_time;
            new_tracks(i).direction = NaN;
            new_tracks(i).velocity = NaN;
            new_tracks(i).countForAverage = 1;
            new_tracks(i).P = eye(4);


            % добавляем лучшую отметку
            obj_num = viewed_realnum(i); % какому номеру реальной цели соответствует отметка или nan
            if ~isnan(obj_num)
                true_data(obj_num).assigned(end) = 1;
                true_data(obj_num).prev_track_nums(end+1) = new_tracks(i).id;
                true_data(obj_num).track_id(end) = new_tracks(i).id; % какому треку присвоена отметка
            end 
        end

        tracks = [tracks, new_tracks];
    
    % Здесь фильтр Калмана
    for t_id = 1:length(tracks)
        if length(tracks(t_id).x) > 2 && ~tracks(t_id).filtered
            dt = tracks(t_id).timestamps(end) - tracks(t_id).timestamps(end-1);
            [new_x, new_y, new_theta, new_v, new_P] = kalmanFilter(tracks(t_id).x(end-1), tracks(t_id).y(end-1), tracks(t_id).direction(end-1), tracks(t_id).velocity(end-1), ...
                tracks(t_id).x(end),  tracks(t_id).y(end), tracks(t_id).direction(end), tracks(t_id).velocity(end), tracks(t_id).P, dt);

            tracks(t_id).x(end) = new_x;
            tracks(t_id).y(end) = new_y;
            tracks(t_id).velocity(end) = new_v;
            tracks(t_id).direction(end) = new_theta;
            tracks(t_id).P = new_P;
            tracks(t_id).filtered = 1;
        end
    end
end

    %% рисование можно отключать
    if DRAW_GRAPHICS
        updateGraphics(true_data, h_true, radar_angle, angle_resolution_rad, ...
                   radius, h_beam, tracks, h_tracks, N_TRACKS);
    end

    % Можно смотреть результаты на графике после нескольких секунд
    if curr_time > 10000
        DRAW_GRAPHICS = 1;
    end

    curr_time = curr_time + deltaT;
    
end