function output = doIntersect( p1, q1, p2, q2 )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
    o1 = orientation(p1, q1, p2);
    o2 = orientation(p1, q1, q2);
    o3 = orientation(p2, q2, p1);
    o4 = orientation(p2, q2, q1);
    
    if (o1 ~= o2 && o3 ~= o4)
        output = 1;
        return;
    end
    
    if (o1 == 0 && onSegment(p1, p2, q1)) 
        output = 1;
        return;        
    end

    if (o2 == 0 && onSegment(p1, q2, q1)) 
        output = 1;
        return;
    end

    if (o3 == 0 && onSegment(p2, p1, q2)) 
        output = 1;
        return;
    end

    if (o4 == 0 && onSegment(p2, q1, q2))
        output = 1;
        return;
    end
    
    output = 0;

end
