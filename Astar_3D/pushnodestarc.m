function frontier = pushnodestarc(frontier, newnode)

frontier_length = numel(frontier.starcost);
entrylocation = find(frontier.starcost > newnode.starcost, 1);
if isempty(entrylocation)
    entrylocation = frontier_length + 1;
end
frontier.starcost = [frontier.starcost(1:entrylocation-1,:); newnode.starcost; frontier.starcost(entrylocation:end,:)];
frontier.cost = [frontier.cost(1:entrylocation-1,:); newnode.cost; frontier.cost(entrylocation:end,:)];
frontier.node = [frontier.node(1:entrylocation-1, :); newnode.node; frontier.node(entrylocation:end, :)];
%frontier.index = [frontier.index(1:entrylocation-1, :); newnode.index; frontier.index(entrylocation:end, :)];
frontier.indexc = [frontier.indexc(1:entrylocation-1, :); newnode.indexc; frontier.indexc(entrylocation:end, :)];
frontier.icons = [frontier.icons(1:entrylocation-1, :); newnode.icons; frontier.icons(entrylocation:end, :)];
frontier.t = [frontier.t(1:entrylocation-1); newnode.t; frontier.t(entrylocation:end)];
frontier.parent = [frontier.parent(1:entrylocation-1, :); newnode.parent; frontier.parent(entrylocation:end, :)];
frontier.parentindexc = [frontier.parentindexc(1:entrylocation-1, :); newnode.parentindexc; frontier.parentindexc(entrylocation:end, :)];
frontier.parenticons = [frontier.parenticons(1:entrylocation-1, :); newnode.parenticons; frontier.parenticons(entrylocation:end, :)];
frontier.parenttime = [frontier.parenttime(1:entrylocation-1, :); newnode.parenttime; frontier.parenttime(entrylocation:end, :)];
frontier.action = [frontier.action(1:entrylocation-1, :); newnode.action; frontier.action(entrylocation:end, :)];