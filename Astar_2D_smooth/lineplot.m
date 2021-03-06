clear; clc; close all;

p1 = [0; 0.1]; p2 = [3; 4];
%p3 = [1.6; 3.8]; p4 = [2.8; 2.2];
p3 = 2.0*p1; p4 = -1.4*p2;
ll = 100;

lam = linspace(0,1, ll)';
myu = linspace(0,1, ll)';

l1 = zeros(ll, 2);
l2 = zeros(ll, 2);
for i = 1:2
    l1(:,i) = lam*p2(i) +  (1 - lam)*p1(i);
    l2(:,i) = myu*p4(i) +  (1 - myu)*p3(i);
end

figure(1)
plot(p1(1), p1(2), 'go', 'Markersize', 8, 'Markerfacecolor', 'g');
hold on
plot(p2(1), p2(2), 'ro', 'Markersize', 8, 'Markerfacecolor', 'r');
plot(l1(:,1), l1(:,2), 'k', 'linewidth', 2);

plot(p3(1), p3(2), 'gp', 'Markersize', 8, 'Markerfacecolor', 'g');
hold on
plot(p4(1), p4(2), 'rp', 'Markersize', 8, 'Markerfacecolor', 'r');
plot(l2(:,1), l2(:,2), 'k:', 'linewidth', 2);

%%%intersection
A = [(p2 - p1), (p3 - p4)];
cA = cond(A);
b = p3 - p1;
if abs(cA) > 1e12
    lm = [inf; inf];
    fprintf('Parallel or nearly parallel lines! no intersection practical\n');
else
    lm = A\b;
end

lams = lm(1);
myus = lm(2);

if lams >= 0 && lams <= 1
    if myus >= 0 && myus <= 1
        fprintf('intersection point found\n');
        ip = lams*p2 +  (1 - lams)*p1;
        ipc = myus*p4 +  (1 - myus)*p3;
        plot(ip(1), ip(2), 'gs', 'Markersize', 8, 'Markerfacecolor', 'g');
        plot(ipc(1), ipc(2), 'rs', 'Markersize', 3, 'Markerfacecolor', 'r');
    else
        fprintf('no intersection point found\n');
    end
else
    fprintf('no intersection point found\n');
end