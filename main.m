%% Init
clear all
addpath('Path_algorithm')
addpath('PSO_algorithm')
%% Read map
img = imread('mappa5.png');
img = imresize(img,[120 120]);
img = imbinarize(img);
img = img(:,:,1); 
imshow(img,[]);
%% Swarm/robot parameters
max_robot_speed = 4;
caution_distance = 2*max_robot_speed;
number_of_robots = 4;
%% Live plot
LIVE_PLOT = 1;
max_step = 50;
%% Initialize starting population
% Starting bounds for group of robot; 
start_X_bound = [6;50]; % [left position bound; right position bound]
start_Y_bound = [75;110]; % [left position bound; right position bound]
%end point for group of robots
end_point = [100,7];
%%
i = 0;
x = [];
while (i < 1000)
    x = select_starting(start_X_bound, start_Y_bound, number_of_robots, max_robot_speed+caution_distance);
    if size(x,1) == number_of_robots
        break;
    end
    i = i+1;
end
if i >= 1000
    disp("Couldn't estimate starting position. Too many robots/to little space.")
end

imshow(img,[]);
hold on
scatter(x(:,1),x(:,2), 'x')
viscircles(x,max_robot_speed*ones(number_of_robots,1));
viscircles(x,ones(number_of_robots,1)*caution_distance, 'Color', 'g');

%% PSO OPTIONS
for a = 1 % PSO parameters, expand to see.
    pop_size = 12; % Size of swarm population
    sol_space = 2; % size of solution space

    e = 1; % threshold for elite population
    r = 20; %distance threshold
    L = 2; % neighbourhood size

    w = 0.8; % weight coeff/ inertia weight
    c=[0.3;0.6]; % [self learning coeff; group learning coeff]
    Vx_max = [-2;2]; % maximum velocity [left;right]
    Vy_max = [-2;2]; % maximum velocity [up;down]

    K = 10; % generations between mutations 
    F = 0.25;
    CR = 0.25;
end

%% Initialize best positions

weight = 1;
for i=1:1:number_of_robots
    paths{i} = pathfinder(img,x(i,:),end_point, weight);
end
weight = 100;
for i=1:number_of_robots
    X(i)= size(paths{1,i},1);
end
[X, ii] = sort(X, 'ascend');
x = x(ii,:);
for k = 1:number_of_robots
    movement{k} = x(k,:);
end
%%
current_ind_leader_path = 1;
ticc = [];
for ff = 1:max_step
tic
disp("Iteration: "+ num2str(ff));
% move best poit (poin't with best paht length can move freely), selecting
% leader and moving him.
if current_ind_leader_path+max_robot_speed <= size(paths{1,ii(1)},1)
    x(1,:) = paths{1,ii(1)}(current_ind_leader_path+max_robot_speed,:);
    current_ind_leader_path = current_ind_leader_path+max_robot_speed;
else
    x(1,:) = paths{1,ii(1)}(end,:);
    current_ind_leader_path = size(paths{1,ii(1)},1);
end
leader_pos = x(1,:);
movement{1}(end+1,:) = leader_pos;

%% reestimate best positions
if mod(ff,5) == 0
    for i=1:number_of_robots
        paths2{i} = pathfinder(img,x(i,:),x(1,:), weight);
    end
    weight = 5;
    X2 = [];
    for i=1:number_of_robots
        X2(i)= size(paths2{1,i},1);
    end
    [X2, ii2] = sort(X, 'ascend');
    x = x(ii2,:);
end

%% ALGORITHM
for robot=2:number_of_robots
    %% PSO algorithm for each robot to find best move (max speed, min destruction prob)
    for a = 1
        % Initialize velocity vectors
        v = zeros(pop_size,sol_space);

        % Initialize position vectors
        [sx, sy] = random_on_circle(x(robot,1),x(robot,2), max_robot_speed, pop_size-1);
        sx(sx>120) = 120;
        sy(sy>120) = 120;
        sx(sx<1) = 1;
        sy(sy<1) = 1;
        PSO_x = round([sx', sy']);
        PSO_x = [PSO_x;x(robot,:)];
        pbest_fit = ones(pop_size,1)*1000;

        % Population
        P_sort = [];
        % Initialize elites
        pbest = PSO_x;
    end
    %% PSO
    generation = 1;
    max_generation = 18;

    %figure(2)

    while (generation < max_generation)
        [is_elite, fit_all] = elite_selection(PSO_x,e,r,leader_pos,img,weight,[x(1:robot-1,:);x(robot+1:end,:)],caution_distance, max_robot_speed);
        
        G = close_neigh(PSO_x,L,fit_all);
        for i = 1:size(PSO_x,1)
            if ~ismember(i,is_elite)
                if fit_all(i) < pbest_fit(i)
                    pbest_fit(i) = fit_all(i);
                    pbest(i,:) =  PSO_x(i,:);
                end
                v_new = vel_update(v(i,:),PSO_x(i,:),w,pbest(i,:),G(i,:), c);
                if (v_new(1) < Vx_max(1))
                    v_new(1) = Vx_max(1);
                elseif (v_new(1) > Vx_max(2))
                    v_new(1) = Vx_max(2);
                end

                if (v_new(2) < Vy_max(1))
                    v_new(2) = Vy_max(1);
                elseif (v_new(2) > Vy_max(2))
                    v_new(2) = Vy_max(2);
                end

                x_new = pos_update(PSO_x(i,:),v(i,:));
                
                if (round((x(robot,1)-x_new(1))^2 + (x(robot,2) - x_new(2))^2) > max_robot_speed^2)
                    x_new = x(robot,:)+max_robot_speed*((x_new-x(robot,:))/norm(x_new-x(robot,:)));
                end               
                if x_new(1) < 1
                    x_new(1) = 1;
                end
                if x_new(2) < 1
                    x_new(2) = 1;
                end
                if x_new(1) > 117
                    x_new(1) = 117;
                end
                if x_new(2) > 117
                    x_new(2) = 117;
                end
                x_new = round(x_new);                
                
                v(i,:) = (v_new);
                PSO_x(i,:) = x_new;                
            end
        end

        if(LIVE_PLOT) %roboks live plot
            imshow(img)
            hold on
            scatter(x(robot,1), x(robot,2))
            scatter(PSO_x(:,1), PSO_x(:,2), 'bx')
            viscircles(x,max_robot_speed*ones(number_of_robots,1));
            viscircles(x,ones(number_of_robots,1)*caution_distance, 'Color', 'g');
            drawnow

            hold off
        end
        generation = generation+1;
    end
    
    [mfit, iii] = min(pbest_fit);
    movement{robot}(end+1,:) = pbest(iii,:);
    x(robot,:) = pbest(iii,:);
    
end
tocc = toc;
ticc = [ticc, tocc];
end
%disp("Mean step time: "+ num2str(sum(ticc)/size(ticc,2)))
%% Plot paths
figure()
for i = 1:ff
    imshow(img,[],'InitialMagnification', 800);
    hold on
    for j = 1:number_of_robots
        scatter(movement{1,j}(i,1),movement{1,j}(i,2))
        viscircles([movement{1,j}(i,1),movement{1,j}(i,2)],max_robot_speed);
        viscircles([movement{1,j}(i,1),movement{1,j}(i,2)],caution_distance);
    end
    hold off
    pause(0.1)
end