function dp = drawdomain_3d(domain)

xmin = domain(1,1);
xmax = domain(1,2);
ymin = domain(2,1);
ymax = domain(2,2);
zmin = domain(3,1);
zmax = domain(3,2);

%%%define vertices: bottom face then top face: ccw
dom_vertices = [xmin ymin zmin;
    xmax ymin zmin;
    xmax ymax zmin;
    xmin ymax zmin;
    xmin ymin zmax;
    xmax ymin zmax;
    xmax ymax zmax;
    xmin ymax zmax];

dom_faces = [1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    1 5 8 4;
    1 4 3 2;
    5 6 7 8];

dp = patch('Faces', dom_faces, 'Vertices', dom_vertices);
dp.FaceColor = '#EDB120'; %'#77AC30'; % 'yellow';
dp.FaceAlpha = 0.1;
%dp.FaceLighting = 'gouraud';
dp.LineWidth = 2;
dp.Marker = '*';