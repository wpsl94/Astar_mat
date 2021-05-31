% Contributors:
%   Dave Grymin, AFRL/RQ
%   Kyle Matissek, AFIT
%   Austin Rivera, AFIT
% Distribution Statement A: Approved for Public Release; Distribution
% Unlimited; Report Number AFIT-ENY-MS-20-M-271.
% Copyright (c) 2017 Government of the United States of America, as
% represented by the Secretary of the Air Force. No copyright is claimed in
% the United States under Title 17, U.S. Code. All Other Rights Reserved.
function plot_obstacles(obstacles, figHandle)
figure(figHandle)
hold on

xO = [];
yO = [];

if( obstacles.number > 0 )
    for k = 1:obstacles.number
        xO = [xO;obstacles.vertices{k}(:,1); obstacles.vertices{k}(1,1);NaN];
        yO = [yO;obstacles.vertices{k}(:,2); obstacles.vertices{k}(1,2);NaN];
        fill(obstacles.vertices{k}(:,1), obstacles.vertices{k}(:,2), 'k');
    end
    
    plot(xO,yO, 'Color', [0.2, 0.2, 1], 'LineWidth',2)
end