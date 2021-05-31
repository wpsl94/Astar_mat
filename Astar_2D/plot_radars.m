function plot_radars(radars, grid_mesh_x, grid_mesh_y, figHandle)

figure(figHandle)
hold on

radar_num = radars.number;

xR = zeros(radar_num,1);
yR = zeros(radar_num,1);

gridnx = size(grid_mesh_x);
gridny = size(grid_mesh_y);
Pd = zeros(gridnx);

if( radars.number > 0 )
    for k = 1:radars.number
        xR(k) = radars.location{k}(1);
        yR(k) = radars.location{k}(2);
        S = radars.covariance{k};
        iS = inv(S);
        NZ = 2*pi*sqrt(det(S));
        for i = 1:gridnx(1)
            for j = 1:gridny(2)
                cc = [grid_mesh_x(i,j) - radars.location{k}(1,1); grid_mesh_y(i,j) - radars.location{k}(1,2)];
                Pd(i,j) = exp(-0.5*(cc'*iS*cc));
            end
        end
        %Pd = Pd/NZ;
        contour(grid_mesh_x, grid_mesh_y, Pd, 50);
    end
    plot(xR, yR, 'k*', 'Linewidth', 2, 'MarkerSize', 12)
end