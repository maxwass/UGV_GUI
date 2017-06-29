function map_out = findHeuristicMap( sMap,goal )
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

    for i=1:sMap(1)
        for j=1:sMap(2)
            map_out(i,j) = findDistance([i,j],goal);
        end
    end


end

