function drawobstacles_3d(ob3d, figHandle)

figure(figHandle)
hold on

for i = 1:ob3d.number
    dp = patch('Faces', ob3d.faces{i}, 'Vertices', ob3d.vertices{i});
    dp.FaceColor = '#A2142F';
    dp.FaceAlpha = 0.1;
    dp.LineStyle = ':';
    dp.LineWidth = 1;
end