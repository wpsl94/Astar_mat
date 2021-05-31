function newnode = applyaction(currnode, currind, actionmove, actionindex)

newnode.pos = currnode + actionmove;
newnode.index = currind + actionindex;

