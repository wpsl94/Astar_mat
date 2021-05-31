function [N, domain, dx, dy, dz] = get_domain_3d

%%%define domain and step size;
N = 3;  %number of dimensions
domain = [0 100; 0, 100; 0 30];
dx = 2; dy = 2; dz = 1;