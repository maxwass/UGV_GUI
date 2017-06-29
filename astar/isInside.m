function output = isInside( obs_r, n, p )
%Returns true if the point p lies inside the polygon[] with n vertices
%   Detailed explanation goes here
    obs = obs_r';
    if (n<3) 
       output = 0;
       return;
    end

    p_inf = [1000, p(2)];
    count = 0;

    for i=1:n
        
        j = i+1;
        if (i == n)
           j = 1;
        end
        if (doIntersect(obs(:,i), obs(:,j), p, p_inf))
            % If the point 'p' is colinear with line segment 'i-next',
            % then check if it lies on segment. If it lies, return true,
            % otherwise false
            if (orientation(obs(:,i), p, obs(:,j)) == 0)
               output = onSegment(obs(:,i), p, obs(:,j));
               return;
            end
 
            count = count + 1;
        end
    end
    output = rem(count,2);

end

