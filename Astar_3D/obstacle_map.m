function obstacles = obstacle_map


obstacles.type = 1; % AABB - 0, Closed Polygon - 1
obstacles.vertices{1} = [20,30; 40,20; 30,90; 25,85];
obstacles.vertices{2} = [50,30; 70,30; 70,50; 50,40]; %[50, 50; 70, 50; 70,70; 50, 60]; %
obstacles.vertices{3} = [11,2; 11,26; 39 2];
obstacles.vertices{4} = [60,60; 90,90; 96,71; 90, 47; 70, 55];
obstacles.hgt = [10; 13; 15; 17];   %height of 3d structure

%%%a sort-of circular obstacles
cnv = 18; cnc = [44, 70; 50, 20; 12 60; 80 15]; cnr = [7; 5; 8.2; 6.8]; %cnc = [50, 70]; cnr = 5; 
cnh = [19; 10; 5; 13];

cobs_num = length(cnr);
ctr = size(obstacles.vertices, 2);
for j = 1:cobs_num
    for i = 1:cnv
        ang = i*(2*pi)/cnv;
        obstacles.vertices{ctr+j}(i, :) = cnr(j)*[cos(ang), sin(ang)] + cnc(j,:);
    end
end
obstacles.hgt = [obstacles.hgt; cnh];

obstacles.number = size(obstacles.vertices, 2);