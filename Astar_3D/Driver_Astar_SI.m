%%%A-star search: Single Index
clear; clc; close all;
EPS = 1e-6;


%%%Vehicle specifics
[speedh, speedc] = get_vehicle_3d;    %units..?: returns speed and turning radius (only speed is relevant)
%speedh = speed in horizontal plane
%speedc = climb speed
DT = .3;
LH = speedh*DT;   %path lengths for motion primitive
LZ = speedc*DT;
actionoption = 2;

%%%Domain
global domain;
[N, domain, ~, ~, ~] = get_domain_3d;
dx = 1.5*LH; dy = 1.5*LH; dz = 15*LZ;
cfig = figure(1);
dp = drawdomain_3d(domain);
hold on;
view(30,30);

%%%%Obstacles and radars
obstacles = obstacle_map;
obstacles3d = obstacle_map_3d(obstacles);
plot_obstacles(obstacles, cfig);
drawobstacles_3d(obstacles3d, cfig);

%%%Set up the grid: note: the start location snaps on to the grid, but the
%%%goal location has no guarantee of doing so.

%%%start and goal
%[startx, starty, startz] = ginput(1);
%start = [startx, starty, startz];
%start = [43.03,  49.71,  1];
start = [80, 30, 3];
%[goalx, goaly, goalz] = ginput(1);
%goal = [goalx, goaly, goalz];
goal = [9.3894,   70.1460, 24];
%goal = [80, 30, 12];

%X--
grid_x_left = start(1):-dx:domain(1,1);
grid_x_right = start(1):dx:domain(1,2);
grid_x = [fliplr(grid_x_left), grid_x_right(2:end)];
LX = length(grid_x);

%Y--
grid_y_down = start(2):-dy:domain(2,1);
grid_y_up = start(2):dy:domain(2,2);
grid_y = [fliplr(grid_y_down), grid_y_up(2:end)];
LY = length(grid_y);

%Z--
grid_z_down = start(3):-dz:domain(3,1);
grid_z_up = start(3):dz:domain(3,2);
grid_z = [fliplr(grid_z_down), grid_z_up(2:end)];
LZ = length(grid_z);

%%%redefine domain
domain = [grid_x(1) grid_x(LX); grid_y(1) grid_y(LY); grid_z(1) grid_z(LZ)];
start_index = [length(grid_x_left), length(grid_y_down), length(grid_z_down)];

%%%plot domain nodes% plot each z-slice
[grid_mesh_x, grid_mesh_y] = meshgrid(grid_x, grid_y);
sxy = size(grid_mesh_x);
figure(1);
for i = 1:LZ
    plot3(grid_mesh_x, grid_mesh_y, grid_z(i)*ones(sxy(1), sxy(2)), 'kx', 'Markersize', 1);
end

%%%Start & Goal 
plot3(start(1), start(2), start(3), 'go', 'Markersize', 6, 'Markerfacecolor', 'g');
plot3(goal(1), goal(2), goal(3), 'ro', 'Markersize', 6, 'Markerfacecolor', 'r');

%%%Identify Goal index: Goal either lies on the grid or within a cell of
%%%the grid: in either case, it can be assigned an index: we will assign
%%%the index of the node closest to it 
[~, g_ind_x] = min(abs(grid_x - goal(1)));
[~, g_ind_y] = min(abs(grid_y - goal(2)));
[~, g_ind_z] = min(abs(grid_z - goal(3)));
goal_index = [g_ind_x, g_ind_y, g_ind_z];
figure(1)
plot3(grid_x(goal_index(1)), grid_y(goal_index(2)), grid_z(goal_index(3)), 'rs', 'Markersize', 6, 'Markerfacecolor', 'r');

%%%Actions: movement + cost
actions = get_actions_3d(dx, dy, dz, actionoption);

%%%Define Visited list: all "unvisited" (including frontier) nodes are set
%%%to zero cost. The entry is the cost value. Algorithm terminates when the
%%%goal index entry acquires a nonzero value.
num_icons = 0;
numgraph = LX*LY*LZ;
visited = zeros(numgraph, 1);   %cost of visitation. A number greater than zero means this node is in the optimal path
visitedtime = -inf(numgraph, 1);    %didn't use inf because need to do a check "> threshold" below
pathload = zeros(numgraph, num_icons);    %this keeps track of all integral constraints for each node
parent_indexc = zeros(numgraph, 1);
parent_node = zeros(numgraph, 3);   %actual node location of parent
parent_act = zeros(numgraph, 1);

start_indexc = (start_index(3) - 1)*LX*LY + (start_index(2) - 1)*LX + start_index(1);
goal_indexc = (goal_index(3) - 1)*LX*LY + (goal_index(2) - 1)*LX + goal_index(1);
visited(start_indexc) = EPS;
visitedtime(start_indexc) = 0;  %if a node is in the "visited column", this data carries a finite time stamp indicating time of arrival
parent_indexc(start_indexc) = start_indexc;
parent_node(start_indexc, :) = start;   %actual location 
parent_act(start_indexc) = 0;


%%%Define Frontier Tuple
frontier.cost = EPS;
frontier.starcost = heuristic_cost_3d(start, goal) + EPS;
frontier.node = start;
frontier.indexc = start_indexc; %"c" in the index stands for "cumulative": transfer (i,j,k) - "I".
frontier.icons = 0; %zeros(1, radars.number);   %integral constraints: make this a row vector containing all relevant constraints
                             %no cost incurred since no time has passed
frontier.t = 0;    %time keeper: this is important for first check on nodes to be popped from frontier                        
frontier.parentindexc = start_indexc;    %index of parent
frontier.parent = start;   %location of parent (where it came from);
frontier.parenticons = 0; %zeros(1, radars.number);  %integral constraint load born by parent node  
frontier.parenttime = 0;    %time stamp at the parent node
frontier.action = 0;    %how it got here.

%%%initialize search
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
    if kickstart && visited(frontier.indexc(1)) > 0
        frontier = popnodestarc(frontier);
        continue;
    else
        kickstart = 1;
        current = assignfront(frontier);     
    end
    
    %%%append the visited and parent structures:
    visited(current.indexc) = current.cost;    %cost to visit
    visitedtime(current.indexc) = current.t;    %time of visitation
    parent_indexc(current.indexc) = current.parentindexc;  %parent of node; must also add what movement led to here
    parent_node(current.indexc, :) = current.parent;
    parent_act(current.indexc) = current.action;
    pathload(current.indexc, :) = current.icons; 

    
    %%%check if this is either exactly the goal or effectively the goal (if
    %%%the goal does not lie on the graph): we match with the goal index
    index_delta = abs(current.indexc - goal_indexc);
    if index_delta == 0    %reached the goal_index
        reachedgoal = 1;
        fprintf('reached goal...breaking search\n');
        break;
    end
    
    %%%remove current node from the Frontier (POP)
    frontier = popnodestarc(frontier);

    
    %%%Run through admissible actions to determine new nodes
    %newnode_optcost = inf;
    for aci = 1:actions.num
        newnode = applyaction_3d(current, actions.move(aci, :), actions.index(aci, :), grid_x, grid_y, grid_z);
        node_admissibility = check_admissibility_3d(newnode.node, current.node, obstacles);
        %node_admissibility = check_admissibility_nolos(newnode.pos, obstacles);
        pathlength = norm(actions.move(aci, :)); 
        DT = pathlength/speedh;    %fix this
        newnode.t = current.t + DT;
        if node_admissibility     %node passes admissibility check: add to frontier
            if visited(newnode.indexc) > 0   %already visited
                continue;
            else     %add to frontier
                new_costtogo = current.cost + actions.cost(aci);
                new_costtogoal = heuristic_cost_3d(newnode.node, goal);
                newnode.cost = new_costtogo;
                newnode.starcost = new_costtogo + new_costtogoal;
                newnode.icons = 0; %applyiconstraint(current.node, current.index, actions.move(aci, :));       
                newnode.parent = current.node;
                newnode.parentindexc = current.indexc;
                newnode.parenticons = current.icons; %%%
                newnode.parenttime = current.t;    %%%
                newnode.action = aci;   %which action led to this new node?                        
                frontier = pushnodestarc(frontier, newnode);
            end
        else
           continue; 
        end
    end
%     fprintf('Frontier size: %d\n', length(frontier.cost));
end
algotime = toc;

fprintf('\nResults from 3D-A* Algorithm:: \n');
if actionoption == 1
    fprintf('Action set: F-B-L-R + Diagonal motion allowed\n');
else
    fprintf('Action set: F-B-L-R only\n');
end
fprintf('Algorithm run time: %1.1f seconds\n', algotime);

%%%When finished, publish the cost and path
cfig  = figure(2);
drawdomain_3d(domain);
hold on;
for i = 1:LZ
    plot3(grid_mesh_x, grid_mesh_y, grid_z(i)*ones(sxy(1), sxy(2)), 'kx', 'Markersize', 1);
end
plot3(start(1), start(2), start(3), 'go', 'Markersize', 6, 'Markerfacecolor', 'g');
plot3(goal(1), goal(2), goal(3), 'ro', 'Markersize', 6, 'Markerfacecolor', 'r');
plot3(grid_x(goal_index(1)), grid_y(goal_index(2)), grid_z(goal_index(3)), 'rs', 'Markersize', 6, 'Markerfacecolor', 'r');
plot_obstacles(obstacles, cfig);
drawobstacles_3d(obstacles3d, cfig);

%%%
% extract the cost to visit goal
%%%
if reachedgoal
    goal_cost = visited(goal_indexc);
    tracing = 1;
    currentindexc = goal_indexc;
    goal_indexpathc = goal_indexc;
    goal_path = current.node;
    goal_actionhis = current.action;
    %goal_load = pathload(goal_indexc);
    goal_time = visitedtime(goal_indexc);

    while tracing
        goal_path = [goal_path; parent_node(currentindexc,:)];
        goal_indexpathc = [goal_indexpathc; parent_indexc(currentindexc)];
        goal_actionhis = [goal_actionhis; parent_act(currentindexc)];
     %   goal_load = [goal_load; pathload(currentindexc)];
        goal_time = [goal_time; visitedtime(currentindexc)];
        currentindexc = parent_indexc(currentindexc);
        if currentindexc == start_indexc
      %      goal_load = [goal_load; pathload(currentindexc)]; %no accural
            goal_time = [goal_time; visitedtime(currentindexc)]; %no accrual
            goal_path = [goal_path; start];
            goal_indexpathc = [goal_indexpathc; start_indexc];
            goal_actionhis = [goal_actionhis; 0];
            tracing = 0;
        end
    end
    forward_indexpath = flipud(goal_indexpathc);
    forward_action = flipud(goal_actionhis);
    forward_path = flipud(goal_path);
    numpath = numel(forward_action);
    %goal_load = flipud(goal_load);
    goal_time = flipud(goal_time);
    fprintf('cost to reach goal: %1.3f units\n\n', goal_cost);    
end
figure(2);
%%%plot the searched area to depict nodes that were "visited"
vindex = find(visited);
vsize = length(vindex);
ix = zeros(vsize, 1); iy = ix; iz = ix;
for i = 1:vsize
    [ixc, iyc, izc] = numtoindex(vindex(i), LX, LY, LZ);
    ix(i) = ixc; iy(i) = iyc; iz(i) = izc;
end
plot3(grid_x(ix), grid_y(iy), grid_z(iz), 'gs', 'Markersize', 4);

if reachedgoal
    figure(2);
    plot3(forward_path(:,1), forward_path(:,2), forward_path(:,3), 'm-o', 'Linewidth', 2, 'Markersize', 4, 'MarkerFaceColor', 'c');
    title('3-D A-star Search');
    figure(1);
    plot3(forward_path(:,1), forward_path(:,2), forward_path(:,3), 'm-o', 'Linewidth', 2, 'Markersize', 4, 'MarkerFaceColor', 'c');
    title('3-D A-star Search');    
else
    fprintf('Did not reach goal: see figure 2 for a depiction of explored nodes\n\n');
end

% % EXTRA:: %%smooth out the path
if reachedgoal
    pnum = size(forward_path, 1);
end

currind = 1;
nextind = 2;
sfpath = zeros(pnum,3);
sfpath(1,:) = forward_path(1,:);
currn = forward_path(currind, :);
nextn = forward_path(nextind, :);
ctr = 2;
while nextind < pnum
    adm_los = check_los_connect(currn(1:2)', nextn(1:2)', obstacles);
    if adm_los
        nextind = nextind + 1;
        nextn = forward_path(nextind, :);
    else
        currind = nextind-1;    %step back.. took too much liberty
        nextind = currind + 1;
        sfpath(ctr, :) = forward_path(currind,:);
        currn = forward_path(currind, :);
        nextn = forward_path(nextind, :);
        ctr = ctr + 1;
    end
end
fzero = find(sfpath(:,1) == 0);
if ~isempty(fzero)
    fzero = fzero(1);
end
sfpath(fzero, :) = forward_path(end,:);
sfpath = sfpath(1:fzero, :);
figure(2);
plot3(sfpath(:,1), sfpath(:, 2), sfpath(:,3), 'r', 'linewidth', 2);
figure(1);
plot3(sfpath(:,1), sfpath(:, 2), sfpath(:,3), 'r', 'linewidth', 2);

%%%Clean plot
cfig = figure(3);
dp = drawdomain_3d(domain);
hold on;
view(30, 30);
drawobstacles_3d(obstacles3d, cfig);
plot3(start(1), start(2), start(3), 'go', 'Markersize', 6, 'Markerfacecolor', 'g');
plot3(goal(1), goal(2), goal(3), 'ro', 'Markersize', 6, 'Markerfacecolor', 'r');
plot3(grid_x(goal_index(1)), grid_y(goal_index(2)), grid_z(goal_index(3)), 'rs', 'Markersize', 6, 'Markerfacecolor', 'r');
plot3(sfpath(:,1), sfpath(:, 2), sfpath(:,3), 'ro', 'Markersize', 4, 'Markerfacecolor', 'r');
plot3(sfpath(:,1), sfpath(:, 2), sfpath(:,3), 'r', 'linewidth', 2);
title('3-D A-star Search');
set(gca,'fontsize', 14, 'fontweight', 'bold')
