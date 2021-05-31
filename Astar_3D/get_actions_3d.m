function actions = get_actions_3d(dx, dy, dz, opt)

if opt == 1
        actions.move = [dx, 0, 0;
        -dx, 0, 0;
        0, dy, 0;
        0, -dy, 0; %;%;
        dx, dy, 0;
        dx, -dy, 0;
        -dx, -dy, 0;
        -dx, dy, 0;
        dx, 0, -dz;
        -dx, 0, -dz;
        0, dy, -dz;
        0, -dy, -dz; %;%;
        dx, dy, -dz;
        dx, -dy, -dz;
        -dx, -dy, -dz;
        -dx, dy, -dz;
        dx, 0, dz;
        -dx, 0, dz;
        0, dy, dz;
        0, -dy, dz; %;%;
        dx, dy, dz;
        dx, -dy, dz;
        -dx, -dy, dz;
        -dx, dy, dz];
    
        actions.index = [1, 0, 0;
        -1, 0, 0;
        0, 1, 0;
        0, -1, 0;
        1, 1, 0;
        1, -1, 0;
        -1, -1, 0;
        -1, 1, 0;
        1, 0, -1;
        -1, 0, -1;
        0, 1, -1;
        0, -1, -1;
        1, 1, -1;
        1, -1, -1;
        -1, -1, -1;
        -1, 1, -1;
        1, 0, 1;
        -1, 0, 1;
        0, 1, 1;
        0, -1, 1;
        1, 1, 1;
        1, -1, 1;
        -1, -1, 1;
        -1, 1, 1];

else
    actions.move = [dx, 0, 0;
        -dx, 0, 0;
        0, dy, 0;
        0, -dy, 0; %;%;
        dx, 0, -dz;
        -dx, 0, -dz;
        0, dy, -dz;
        0, -dy, -dz; %;%;
        dx, 0, dz;
        -dx, 0, dz;
        0, dy, dz;
        0, -dy, dz]; %;%;
    
    actions.index = [1, 0, 0;
        -1, 0, 0;
        0, 1, 0;
        0, -1, 0;
        1, 0, -1;
        -1, 0, -1;
        0, 1, -1;
        0, -1, -1;
        1, 0, 1;
        -1, 0, 1;
        0, 1, 1;
        0, -1, 1];
end

actions.cost = sqrt(actions.move(:,1).^2 + actions.move(:,2).^2 + actions.move(:,3).^2);  %[dx; dx; dy; dy; sqrt(dx^2 + dy^2); sqrt(dx^2 + dy^2); sqrt(dx^2 + dy^2); sqrt(dx^2 + dy^2)];

if opt == 1
    numchange = sum(abs(actions.index), 2);
    allc = find(numchange == 3);
    actions.cost(allc) = 2.0*actions.cost(allc);
else
    numchange = abs(actions.index);
    allc = find(numchange(:,3) > 0);
    actions.cost(allc) = 1.5*actions.cost(allc);
end
actions.num = length(actions.cost);    %how many new nodes to explore