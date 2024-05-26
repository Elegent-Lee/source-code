function out = gradient_sigma_norm(z)
global  l ;

out = z/(sqrt(1+l*norm(z)));% zij in the paper
end