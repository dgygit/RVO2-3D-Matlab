%% set veloctiy range, custom
function [lb,ub] = VelRange(curv, acc, omega, dt)
    vel = norm(curv);
    if vel==0
        nvel = [0.1 0.1 0.1];
    else
        nvel = abs(curv) ./ vel;
    end
    nveld = dt * abs(acc) .* nvel;
    wveld = sin(dt * abs(omega)) * 2 * vel;
    lb = curv - nveld - wveld;
    ub = curv + nveld + wveld;
end