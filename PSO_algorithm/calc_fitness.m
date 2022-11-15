function [fit] = calc_fitness(x, leader_pos, im, weight, distances, caution_distance, max_speed)

%CALC_DISTANCE Summary of this function goes here
%   Detailed explanation goes here
  
    if im(x(2),x(1)) == 0
        fit = inf;
        %path = [];
    else
        
        path = pathfinder(im,x,leader_pos, weight);
        fit = size(path,1) + distance_cost(distances,x,caution_distance, max_speed)*0.8;
        
    end
end

