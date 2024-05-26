function out = rho_h(z)
global  l_alpha ;

if ((z>=0) && (z<l_alpha))
    out = 1;
elseif ((z>=l_alpha) && (z<=1))
    out = 0.5*[1+cos(pi*(z-l_alpha)/(1-l_alpha))];
else
    out = 0;
end
end

