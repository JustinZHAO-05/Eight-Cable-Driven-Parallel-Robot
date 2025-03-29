function T = trvec2tform(vect)
    % TRVEC2TFORM Convert a translation vector to a homogeneous transformation.
    %   T = TRVEC2TFORM(vect) returns a 4x4 homogeneous transformation matrix
    %   with the translation given by vect (which can be symbolic or numeric).
    
    if isa(vect, 'sym')
        T = sym(eye(4));
    else
        T = eye(4);
    end
    T(1:3,4) = vect;
end

