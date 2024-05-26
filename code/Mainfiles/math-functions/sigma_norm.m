function out = sigma_norm(z)
global  l ;


out = (1/l)*(sqrt(1+l*norm(z))-1);
end