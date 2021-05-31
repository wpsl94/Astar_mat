function current = assignfront(frontier)

current.node = frontier.node(1,:);
current.indexc = frontier.indexc(1);
current.cost = frontier.cost(1);
current.starcost = frontier.starcost(1);
current.icons = frontier.icons(1,:);
current.t = frontier.t(1);
current.parent = frontier.parent(1,:);
current.parentindexc = frontier.parentindexc(1);
current.parenticons = frontier.parenticons(1,:);    %int constraint load on parent
current.parenttime = frontier.parenttime(1);    %time at which parent was visited
current.action = frontier.action(1);    %action == action to take from parent to get to current