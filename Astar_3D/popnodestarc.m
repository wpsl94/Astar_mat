function frontier = popnodestarc(frontier)  

frontier.starcost = frontier.starcost(2:end,:);
frontier.cost = frontier.cost(2:end,:);
frontier.node = frontier.node(2:end,:);
frontier.indexc = frontier.indexc(2:end,:);
frontier.icons = frontier.icons(2:end,:);
frontier.t = frontier.t(2:end,:);
frontier.parent = frontier.parent(2:end,:);
frontier.parentindexc = frontier.parentindexc(2:end);
frontier.parenticons = frontier.parenticons(2:end,:);
frontier.parenttime = frontier.parenttime(2:end);
frontier.action = frontier.action(2:end);