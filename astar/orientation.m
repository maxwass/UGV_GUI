function output = orientation( p, q, r )
%// To find orientation of ordered triplet (p, q, r).
%// The function returns following values
%// 0 --> p, q and r are colinear
%// 1 --> Clockwise
%// 2 --> Counterclockwise
%   Detailed explanation goes here
    val = (q(2) - p(2)) * (r(1) - q(1)) - (q(1) - p(1)) * (r(2) - q(2));
 
    if (val == 0) 
        output = 0;
        return;
    end
    if (val > 0)
        output = 1;
        return;
    end
    if (val < 0)
        output = 2;
        return;
    end

end
