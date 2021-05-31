%%%A-star search: 
clear; clc; close all;
EPS = 1e-6;

%%%Domain
global domain;
[N, domain, dx, dy] = get_domain;
figure(1)
drawdomain(domain, 'k', N);
hold on;
%[start, ~] = get_startgoal;  %returns starting point and goal point


%%%Vehicle specifics
[speed, turnrad] = get_vehicle;    %units..?: returns speed and turning radius (only speed is relevant)
%%%%Obstacles and radars
obstacles = obstacle_map;
plot_obstacles(obstacles, figure(1));
radars = radar_map;


%%%Set up the grid: note: the start location snaps on to the grid, but the
%%%goal location has no guarantee of doing so.
%X--
[startx, starty] = ginput(1);
start = [startx, starty];
%start = [43.03,   49.71];
grid_x_left = start(1):-dx:domain(1,1);
grid_x_right = start(1):dx:domain(1,2);
grid_x = [fliplr(grid_x_left), grid_x_right(2:end)];
%Y--
grid_y_down = start(2):-dy:domain(2,1);
grid_y_up = start(2):dy:domain(2,2);
grid_y = [fliplr(grid_y_down), grid_y_up(2:end)];
start_index = [length(grid_x_left), length(grid_y_down)];
[grid_mesh_x, grid_mesh_y] = meshgrid(grid_x, grid_y);
figure(1)
plot(grid_mesh_x, grid_mesh_y, 'kx', 'Markersize', 1);
plot_radars(radars, grid_mesh_x, grid_mesh_y, figure(1));

%%%Start & Goal 
[goalx, goaly] = ginput(1);
goal = [goalx, goaly];
%goal = [9.3894,   70.1460];
plot(start(1), start(2), 'go', 'Markersize', 6, 'Markerfacecolor', 'g');
plot(goal(1), goal(2), 'ro', 'Markersize', 6, 'Markerfacecolor', 'r');



%%%Identify Goal index: Goal either lies on the grid or within a cell of
%%%the grid: in either case, it can be assigned an index: we will assign
%%%the index of the node closest to it 
[~, g_ind_x] = min(abs(grid_x - goal(1)));
[~, g_ind_y] = min(abs(grid_y - goal(2)));
goal_index = [g_ind_x, g_ind_y];
figure(1)
plot(grid_x(goal_index(1)), grid_y(goal_index(2)), 'rs', 'Markersize', 6, 'Markerfacecolor', 'r');

%%%Actions: movement + cost
actions = get_actions(dx, dy);

%%%Define Visited list: all "unvisited" (including frontier) nodes are set
%%%to zero cost. The entry is the cost value. Algorithm terminates when the
%%%goal index entry acquires a nonzero value.
visited = zeros(fliplr(size(grid_mesh_x))); %meshgrid dimensions are "backwards"
parent = struct();
pathload = struct();    %this keeps track of all integral constraints for each node
visited(start_index(1), start_index(2)) = EPS;  %identify start node
parent(start_index(1), start_index(2)).p = start_index; %start node is its own parent
pathload(start_index(1), start_index(2)).l = zeros(1, radars.number);   %no path load at start location (accrues over time)

%%%Define Frontier Tuple
frontier.cost = EPS;
frontier.starcost = heuristic_cost(start, goal) + EPS;
frontier.node = start;
frontier.index = start_index;
frontier.icons = 0.0;   %integral constraints: make this a row vector containing all relevant constraints
frontier.t = 0;    %time keeper                        
frontier.parent = start;   %where it came from;
plotswitch = 0;
kickstart = 0;
reachedgoal = 0;
tic
while ~isempty(frontier.cost)
    %%%Extract the optimal node in frontier: this is the very first element
    %%%because it is sorted that way
    %%%
    % check if the top on the frontier list has already been visited. If
    % yes, pop it and skip this iteration
    %%%
    if kickstart && visited(frontier.index(1,1), frontier.index(1,2)) > 0
        frontier = popnodestar(frontier);
        continue;
    else
        kickstart = 1;
        current.node = frontier.node(1,:);
        current.index = frontier.index(1,:);
        current.cost = frontier.cost(1);
        current.startcost = frontier.starcost(1);
        current.icons = frontier.icons(1,:);
        current.t = frontier.t(1);
        current.parent = frontier.parent(1,:);        
    end
    
    %%%append the visited and parent structures:
    visited(current.index(1), current.index(2)) = current.cost;    %cost to visit
    parent(current.index(1), current.index(2)).p = current.parent;  %parent of node
    %pathload(current.index(1), current.index(2)).l = current.icons;    %path load (integral constraints)
    pathload(current.index(1), current.index(2)).t = current.t;        %current time at node

    
    %%%check if this is either exactly the goal or effectively the goal (if
    %%%the goal does not lie on the graph): we match with the goal index
    index_delta = abs(current.index - goal_index);
    if sum(index_delta) == 0    %reached the goal_index
        reachedgoal = 1;
        fprintf('reached goal...breaking search\n');
        break;
    end
    
    %%%remove current node from the Frontier (POP)
    frontier = popnodestar(frontier);

    
    %%%Run through admissible actions to determine new nodes
    %newnode_optcost = inf;
    for aci = 1:actions.num
        newnode = applyaction(current.node, current.index, actions.move(aci, :), actions.index(aci, :));
        node_admissibility = check_admissibility(newnode.pos, current.node, obstacles);
        %node_admissibility = check_admissibility_nolos(newnode.pos, obstacles);
        pathlength = norm(actions.move(aci, :)); DT = pathlength/speed;
        newnode.t = current.t + DT;
        if node_admissibility     %node passes admissibility check: add to frontier
            if visited(newnode.index(1), newnode.index(2)) > 0   %already visited
                continue;
            else     %add to frontier
                new_costtogo = current.cost + actions.cost(aci);
                new_costtogoal = heuristic_cost(newnode.pos, goal);
                newnode.cost = new_costtogo;
                newnode.starcost = new_costtogo + new_costtogoal;
                newnode.icons = 0; %applyiconstraint(current.node, current.index, actions.move(aci, :));       
                newnode.parent = current.index;
                frontier = pushnodestar(frontier, newnode);
            end
        else
           continue; 
        end
    end
%     fprintf('Frontier size: %d\n', length(frontier.cost));
end
toc

fprintf('\nResults from A* Algorithm:: \n');
%%%When finished, publish the cost and path
figure(2)
drawdomain(domain, 'k', N);
hold on;
plot(grid_mesh_x, grid_mesh_y, 'kx', 'Markersize', 1);
plot(start(1), start(2), 'go', 'Markersize', 6, 'Markerfacecolor', 'g');
plot(goal(1), goal(2), 'ro', 'Markersize', 6, 'Markerfacecolor', 'r');
plot_obstacles(obstacles, figure(2));
plot_radars(radars, grid_mesh_x, grid_mesh_y, figure(2));
plot(grid_x(goal_index(1)), grid_y(goal_index(2)), 'rs', 'Markersize', 6, 'Markerfacecolor', 'r');
%%%
% extract the cost to visit goal
%%%
if reachedgoal
    goal_cost = visited(goal_index(1), goal_index(2));
    tracing = 1;
    currentindex = goal_index;
    goal_indexpath = goal_index;
    goal_time = pathload(goal_index(1), goal_index(2)).t;
    while tracing
        goal_indexpath = [goal_indexpath; parent(currentindex(1), currentindex(2)).p];
        goal_time = [goal_time; pathload(currentindex(1), currentindex(2)).t];
        currentindex = parent(currentindex(1), currentindex(2)).p;
        if sum(currentindex == start_index) == N
            tracing = 0;
        end
    end
    forward_indexpath = flipud(goal_indexpath);
    goal_time = flipud(goal_time);
    numpath = size(forward_indexpath, 1);
    fprintf('cost to reach goal: %1.3f units\n\n', goal_cost);    
end
figure(2);
%%%plot the searched area to depict nodes that were "visited"
vnind = find(visited' > 0); %the transpose is used to be consistent with the plotting of grid.
numvisited = numel(vnind);
plot(grid_mesh_x(vnind), grid_mesh_y(vnind), 'gs');
if reachedgoal
    plot(grid_x(forward_indexpath(:,1)), grid_y(forward_indexpath(:,2)), 'k-o', 'Markersize', 4, 'MarkerFaceColor', 'k', 'linewidth', 1);
else
    fprintf('Did not reach goal: see figure 2 for a depiction of explored nodes\n\n');
end
title('A^* Search');


