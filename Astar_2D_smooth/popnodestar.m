function frontier = popnodestar(frontier)  

frontier.starcost = frontier.starcost(2:end,:);
frontier.cost = frontier.cost(2:end,:);
frontier.node = frontier.node(2:end,:);
frontier.index = frontier.index(2:end,:);
frontier.icons = frontier.icons(2:end,:);
frontier.t = frontier.t(2:end,:);
frontier.parent = frontier.parent(2:end,:);