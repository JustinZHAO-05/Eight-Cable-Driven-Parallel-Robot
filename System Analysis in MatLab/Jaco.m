function J = Jaco(x0,  delta)

    if nargin<4 || isempty(delta)
        delta = 1e-6;
    end

    J  = zeros(8,6);

    for i = 1:3
        x0(i) = x0(i);
    end

    for i = 1:6
        x1_temp    = x0;
        
            x1_temp(i) = x1_temp(i) + delta;
        
        s1_temp    = calculateCableLengths( ...
                      x1_temp(1), x1_temp(2), x1_temp(3), ...
                      x1_temp(4), x1_temp(5), x1_temp(6) )  ;

        x2_temp    = x0;
         
            x2_temp(i) = x2_temp(i) - delta;
        
        s2_temp    = calculateCableLengths( ...
                      x2_temp(1), x2_temp(2), x2_temp(3), ...
                      x2_temp(4), x2_temp(5), x2_temp(6) )  ;

        J(:,i)    = (s1_temp - s2_temp) / (2*delta);
    end
