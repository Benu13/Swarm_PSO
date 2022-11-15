function [is_elite, fit_all] = elite_selection(x,e,ee,leader_pos,img,weight,dist,caution_distance, max_speed)
% Input:
%   x - position matrix
%   e - threshold for elite candidate
%   ee - threshold for elite population
% Output:
%   P_sorted - sorted population
%   E_pop - elite population

X = x;
s_size = size(X,1);
fb = zeros(s_size,1);
E_pop = [];
fb_e = [];

for i=1:s_size
    fb(i) = calc_fitness(X(i,:),leader_pos,img,weight,dist,caution_distance, max_speed);
end

fit_all = fb';
[fb_s, ii] = sort(fb, 'ascend');

P_sorted = X(ii,:);

E_pop(1,:) = P_sorted(1,:);
fb_e(1) = fb_s(1);

is_elite = [];
is_elite = [is_elite, ii(1)];


for i = 2:length(fb)
    if (fb_s(i)-fb_e(1) <= e)
        if norm(E_pop(1,:) - P_sorted(i,:)) > ee
            is_elite = [is_elite, ii(i)];
        end
    else
        break;
    end
end
end

