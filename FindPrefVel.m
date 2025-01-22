%% Find prefer velocity using relative position
function prefv = FindPrefVel(pos, tarpos, maxv)
    rep = tarpos - pos;
    if norm(rep) < maxv
        prefv = rep;
    else
        prefv = normalize(rep,2,'norm') * maxv;
    end
end