function radars = radar_map

radars.number = 2;

%%%type and location
radars.type{1} = 1; %1 = monostatic
radars.location{1} = [62, 7];
radars.thres{1} = 6;

radars.type{2} = 1; %1 = monostatic
radars.location{2} = [45, 90];
radars.thres{2} = 6;

%%%scan range
dtr = pi/180;
ang = 150*dtr;

%%%assign detection probability a Gaussian shape
S = [cos(ang), sin(ang); -sin(ang) cos(ang)];
radars.covariance{1} = S*[200 0; 0 500]*S';
radars.covariance{2} = [500 0; 0 200];