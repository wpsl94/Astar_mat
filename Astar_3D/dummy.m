% % EXTRA:: %%smooth out the path
if reachedgoal
    pnum = size(forward_path, 1);
end

currind = 1;
nextind = 2;
sfpath = zeros(pnum,3);
sfpath(1,:) = forward_path(1,:);
currn = forward_path(currind, :);
nextn = forward_path(nextind, :);
ctr = 2;
while nextind < pnum
    adm_los = check_los_connect(currn(1:2)', nextn(1:2)', obstacles);
    if adm_los
        nextind = nextind + 1;
        nextn = forward_path(nextind, :);
    else
        currind = nextind-1;    %step back.. took too much liberty
        nextind = currind + 1;
        sfpath(ctr, :) = forward_path(currind,:);
        currn = forward_path(currind, :);
        nextn = forward_path(nextind, :);
        ctr = ctr + 1;
    end
end
fzero = find(sfpath(:,1) == 0);
if ~isempty(fzero)
    fzero = fzero(1);
end
sfpath(fzero, :) = forward_path(end,:);
sfpath = sfpath(1:fzero, :);
figure(2);
plot3(sfpath(:,1), sfpath(:, 2), sfpath(:,3), 'r', 'linewidth', 2);
figure(1);
plot3(sfpath(:,1), sfpath(:, 2), sfpath(:,3), 'r', 'linewidth', 2);


%%%Clean plot
figure(3)
