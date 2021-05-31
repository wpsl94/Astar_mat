function [ix, iy, it] = numtoindex(ind, LX, LY, ~)

LXY = LX*LY;
rem1 = mod(ind, LXY);

if rem1
    it = fix(ind/(LXY)) + 1;
    if mod(rem1, LX) == 0
        ix = LX;
        iy = fix(rem1/LX);
    else
        ix = mod(rem1, LX);
        iy = fix(rem1/LX)+1;
    end
else
    it = fix(ind/LXY);
    ix = LX;
    iy = LY;
end