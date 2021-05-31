function obstacles3d = obstacle_map_3d(obstacles)


for i = 1:obstacles.number
    vnum(i) = size(obstacles.vertices{i}, 1);   %number of vertices in the footprint
    obstacles3d.vertices{i} = [[obstacles.vertices{i}, zeros(vnum(i), 1)]; [obstacles.vertices{i}, obstacles.hgt(i)*ones(vnum(i), 1)]]; 
end



for i = 1:obstacles.number
    vnumc = vnum(i);
    if vnum(i) < 4
        currfaces = NaN(vnumc + 2, 4);
    else
        currfaces = NaN(vnumc + 2, vnumc);
    end
    %%%first build the lateral faces (around the obstacle)
    for j = 1:vnumc
        if j == vnumc
            currfaces(j,1:4) = [j, 1, 1+vnumc, j+vnumc];
        else
            currfaces(j,1:4) = [j, j+1, j+vnumc+1, j+vnumc];
        end        
    end
    %%%build the bottom face
    currfaces(vnumc+1,1:vnumc) = 1:vnumc;
    %%%build the top face
    currfaces(vnumc+2,1:vnumc) = vnumc+1:2*vnumc;
    obstacles3d.faces{i} = currfaces;
end

obstacles3d.number = obstacles.number;