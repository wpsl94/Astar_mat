function adm = check_los_connect(nodepos, parentnode, obstacles)
%%%checks if the line joining input nodes are intersects one or several of
%%%obstacles


%%%check obstacle LOS admissibility
obsnum = obstacles.number;
admissobst = true(1);

for j = 1:obsnum
    if admissobst == 0
        break;
    end
    obc = [obstacles.vertices{j}; obstacles.vertices{j}(1,:)]; %current obstacle (close it up on itself)
    obvn = size(obc, 1);    %number of vertices + 1 (repeated the 1st vertex)
    for k = 1:obvn-1
        if admissobst == 0
            break;
        else
            %%line 1: parent-node pair
            p1 = parentnode; p2 = nodepos;
            %%line 2: obstacle edge
            p3 = obc(k,:)'; p4 = obc(k+1,:)';
            Amat = [(p2 - p1), (p3 - p4)];
            cA = cond(Amat);
            if cA < 1e12
                bvec = p3 - p1;
                lamyu = Amat\bvec;
                if ((lamyu(1) < 0 || lamyu(1) > 1) || (lamyu(2) < 0 || lamyu(2) > 1)) %does not intersect
                    continue;
                else   %intersects, and so, in-admissible
                    admissobst = 0;
                end
            end
        end
    end
end
adm = admissobst;