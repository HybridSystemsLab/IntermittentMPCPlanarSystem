function [uout,init] = timererror(u,theta_c,omega_c)
    
    if(theta_c>0)
        init = randi(theta_c);
    else
        init = 1;
    end
    u = u(init:end);    

    lgth = length(u);
    
    l = lgth;
    
    i = 0;
    if(omega_c>0)
        while(l>0)
            r = randi(l);
            h = randi(omega_c);
            u((r+h+1):end) = u((r+1):(lgth-h));
            u(r:r+h) = u(r);
            l = l-(r+h);
        end
        clear r
        clear h
    end
    
    clear lgth
    clear l
    
    uout = u;
end