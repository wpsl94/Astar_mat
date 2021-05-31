function actions = get_actions(dx, dy)

actions.move = [dx, 0;
    -dx, 0;
    0, dy;
    0, -dy; %;%;
    dx, dy;
    dx, -dy;
    -dx, -dy;
    -dx, dy];
    
actions.index = [1, 0;
    -1, 0;
    0, 1;
    0, -1;
    1, 1;
    1, -1;
    -1, -1;
    -1, 1];

actions.cost = sqrt(actions.move(:,1).^2 + actions.move(:,2).^2);  %[dx; dx; dy; dy; sqrt(dx^2 + dy^2); sqrt(dx^2 + dy^2); sqrt(dx^2 + dy^2); sqrt(dx^2 + dy^2)];
actions.num = length(actions.cost);    %how many new nodes to explore
