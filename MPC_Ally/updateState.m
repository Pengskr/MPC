function [x, y, yaw] = updateState(x, y, yaw, v, Delta,dt,Length,max_steer)
    Delta = max(min(max_steer, Delta), -max_steer);
    x = x + v * cos(yaw) * dt;
    y = y + v * sin(yaw) * dt;
    yaw = yaw + v / Length * tan(Delta) * dt ;
end