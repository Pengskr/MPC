function target_idx = calc_target_index(x,y, refPos_x,refPos_y)
    i = 1:length(refPos_x)-1;
    dist = sqrt((refPos_x(i)-x).^2 + (refPos_y(i)-y).^2);
    [~, target_idx] = min(dist);
end