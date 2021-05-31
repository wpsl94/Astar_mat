function newnode = applyaction_3d(current, movement, indexshift, grid_x, grid_y, grid_z)

%%%in this one: there are only three possible actions: move straight by a
%%%fixed length, left or right by the same length at the vehicles min turn
%%%radius


LX = length(grid_x);
LY = length(grid_y);
LZ = length(grid_z);

newnode.node = current.node + movement;
[cix, ciy, ciz] = numtoindex(current.indexc, LX, LY, LZ);

newindex = [cix, ciy, ciz] + indexshift;
newnode.indexc = (newindex(3) - 1)*LX*LY + (newindex(2) - 1)*LX + newindex(1);