function dist_cost = distance_cost(x_pos, x, cautious_dist, max_speed)
%DISTANCE_COST Summary of this function goes here
%   Detailed explanation goes here
    dist = pdist2(x,x_pos);
    
    dist_cost = 0;

    for i = dist
        if  i > 2*cautious_dist
            continue;
        elseif (i < max_speed)
            dist_cost = dist_cost + inf;
        else
            dist_cost = dist_cost + (2*cautious_dist-i);
        end
    end
end

