function frontier = pushnodestar(frontier, newnode)

frontier_length = numel(frontier.starcost);
entrylocation = find(frontier.starcost > newnode.starcost, 1);
if isempty(entrylocation)
    entrylocation = frontier_length + 1;
end
frontier.starcost = [frontier.starcost(1:entrylocation-1,:); newnode.starcost; frontier.starcost(entrylocation:end,:)];
frontier.cost = [frontier.cost(1:entrylocation-1,:); newnode.cost; frontier.cost(entrylocation:end,:)];
frontier.node = [frontier.node(1:entrylocation-1, :); newnode.pos; frontier.node(entrylocation:end, :)];
frontier.index = [frontier.index(1:entrylocation-1, :); newnode.index; frontier.index(entrylocation:end, :)];
frontier.icons = [frontier.icons(1:entrylocation-1, :); newnode.icons; frontier.icons(entrylocation:end, :)];
frontier.t = [frontier.t(1:entrylocation-1); newnode.t; frontier.t(entrylocation:end)];
frontier.parent = [frontier.parent(1:entrylocation-1, :); newnode.parent; frontier.parent(entrylocation:end, :)];
