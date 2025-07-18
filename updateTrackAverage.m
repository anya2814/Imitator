function track = updateTrackAverage(track, new_x, new_y, new_d, new_angle, curr_time)
    track.countForAverage = track.countForAverage + 1;
    n = track.countForAverage;
    track.distance(end) = (track.distance(end)*(n-1) + new_d)/n;
    track.angle(end) = (track.angle(end)*(n-1) + new_angle)/n;
    track.x(end) = (track.x(end)*(n-1) + new_x)/n;
    track.y(end) = (track.y(end)*(n-1) + new_y)/n;
    track.timestamps(end) = (track.timestamps(end)*(n-1) + curr_time)/n;

    if length(track.x) > 1
        dx = diff(track.x(end-1:end));
        dy = diff(track.y(end-1:end));
        dt = diff(track.timestamps(end-1:end));
        track.velocity(end) = hypot(dx, dy) / dt;
        track.direction(end) = mod(atan2(dy, dx), 2*pi);
    end
end