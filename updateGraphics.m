function updateGraphics(true_data, h_true, radar_angle, angle_resolution_rad, radius, ...
                        h_beam, tracks, h_tracks, N_TRACKS)

    % --- Обновляем истинные позиции ---
    for i = 1:length(true_data)
        set(h_true(i), 'XData', true_data(i).x, 'YData', true_data(i).y);
    end

    % --- Обновляем луч радара ---
    beam_x = radius * cos(radar_angle);
    beam_y = radius * sin(radar_angle);
    set(h_beam(1), 'XData', [0 beam_x], 'YData', [0 beam_y]);

    beam_x = radius * cos(radar_angle + angle_resolution_rad/2);
    beam_y = radius * sin(radar_angle + angle_resolution_rad/2);
    set(h_beam(2), 'XData', [0 beam_x], 'YData', [0 beam_y]);

    beam_x = radius * cos(radar_angle - angle_resolution_rad/2);
    beam_y = radius * sin(radar_angle - angle_resolution_rad/2);
    set(h_beam(3), 'XData', [0 beam_x], 'YData', [0 beam_y]);

    % --- Обновляем треки ---
    for t_id = 1:min(length(tracks), N_TRACKS)
        if length(tracks(t_id).x) > 1
            set(h_tracks(t_id), 'XData', tracks(t_id).x, 'YData', tracks(t_id).y);
        else 
            set(h_tracks(t_id), 'MarkerFaceColor', "#C0C0C0", ...
                                'XData', tracks(t_id).x, ...
                                'YData', tracks(t_id).y);
        end
    end

    % --- Скрываем неиспользуемые треки ---
    for t_id = length(tracks)+1:N_TRACKS
        set(h_tracks(t_id), 'XData', nan, 'YData', nan);
    end

    drawnow limitrate;
end
