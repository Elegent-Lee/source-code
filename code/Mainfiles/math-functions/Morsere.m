function out = Morsere(x,d,r)
if (x>=0)&&(x<d)
      out=2*0.2*4*(1-exp(-0.2.*(x-d))).*exp(-0.2.*(x-d));
else
    out=0;
end
end