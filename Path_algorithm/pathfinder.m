function path = pathfinder(im,start_point,end_point,weight)

%     se = strel('disk',6);
%     im = imerode(im,se);
    MAP = double(im(:,:,1));
    [xmax, ymax] = size(MAP);
    MAP(MAP == 0) = inf;
    MAP(MAP ~= inf) = 0;

    start_y = start_point(1);
    start_x = start_point(2);

    goal_y = end_point(1);
    goal_x = end_point(2);

    start = [start_x, start_y];
    goal = [goal_x, goal_y];

    %%
    %Increasing weight makes the algorithm greedier, and likely to take a
    %longer path, but with less computations.
    %weight = 0 gives Djikstra algorithm
    
    H = zeros([xmax, ymax]);
    G = zeros([xmax, ymax]);
    %Heuristic Map of all nodes
    for x = 1:size(MAP,1)
        for y = 1:size(MAP,2)
            if(MAP(x,y)~=inf)
                H(x,y) = weight*norm(goal-[x,y]);
                G(x,y) = inf;
            end
        end
    end
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %initial conditions%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    G(start(1),start(2)) = 0;
    F(start(1),start(2)) = H(start(1),start(2));
    closedNodes = ones([10000,5]) * inf;
    openNodes = [start G(start(1),start(2)) F(start(1),start(2)) 0]; %[x y G F cameFrom]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Solve
    solved = false;
    ind = 0;
    while(~isempty(openNodes))

        %find node from open set with smallest F value
        [~,I] = min(openNodes(:,4));

        %set current node
        current = openNodes(I,:);

        %if goal is reached, break the loop
        if(isequal(current(1:2),goal))
            ind = ind+1;
            closedNodes(ind,:) = current;
            solved = true;
            break;
        end

        %remove current node from open set and add it to closed set
        openNodes(I,:) = [];
        ind = ind+1;
        closedNodes(ind,:) = current;
        
        
        %for all neighbors of current node
        for x = current(1)-1:current(1)+1
            for y = current(2)-1:current(2)+1

                %if out of range skip
                if (x<1||x>xmax||y<1||y>ymax)
                    continue
                end

                %if object skip
                if (isinf(MAP(x,y)))
                    continue
                end

                %if current node skip
                if (x==current(1)&&y==current(2))
                    continue
                end

                %if already in closed set skip
                skip = 0;
                for j = 1:ind
                    if(x == closedNodes(j,1) && y==closedNodes(j,2))
                        skip = 1;
                        break;
                    end
                end
                if(skip == 1)
                    continue
                end

                A = [];
                %Check if already in open set
                if(~isempty(openNodes))
                    for j = 1:size(openNodes,1)
                        if(x == openNodes(j,1) && y==openNodes(j,2))
                            A = j;
                            break;
                        end
                    end
                end


                newG = G(current(1),current(2)) + round(norm([current(1)-x,current(2)-y]),1);

                %if not in open set, add to open set
                if(isempty(A))
                    G(x,y) = newG;
                    newF = G(x,y) + H(x,y);
                    newNode = [x y G(x,y) newF ind];
                    openNodes = [openNodes; newNode];
                    continue
                end

                %if no better path, skip
                if (newG >= G(x,y))
                    continue
                end
                
                G(x,y) = newG;
                newF = newG + H(x,y);
                openNodes(A,3:5) = [newG newF ind];
            end
        end
    end
    if (solved)
        j = ind;
        i = 0;
        path = zeros(j,2);
        while(j > 0)
            x = closedNodes(j,1);
            y = closedNodes(j,2);
            j = closedNodes(j,5);
            path(end-i,:) = [y,x];
            i = i+1;
        end
        path = path(end-i+1:end,:);
    else
        disp('No path Found')
        disp(start_point)
        disp(end_point)
        path = [];
    end

end

