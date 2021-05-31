currind = 1;
nextind = 2;
sfpath = zeros(pnum,2);
sfpath(1,:) = fpath(1,:);
currn = fpath(currind, :);
nextn = fpath(nextind, :);
ctr = 2;
while nextind < pnum
    adm_los = check_los_connect(currn', nextn', obstacles);
    if adm_los
        nextind = nextind + 1;
        nextn = fpath(nextind, :);
    else
        currind = nextind-1;    %step back.. took too much liberty
        nextind = currind + 1;
        sfpath(ctr, :) = fpath(currind,:);
        currn = fpath(currind, :);
        nextn = fpath(nextind, :);
        ctr = ctr + 1;
    end
end
fzero = find(sfpath(:,1) == 0);
if ~isempty(fzero)
    fzero = fzero(1);
end
sfpath(fzero, :) = fpath(end,:);
sfpath = sfpath(1:fzero, :);
plot(sfpath(:,1), sfpath(:, 2), 'r', 'linewidth', 2);