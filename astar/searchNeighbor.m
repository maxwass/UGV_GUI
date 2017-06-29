function [isSearchedMap_update, costmap_update, arrayFrontier_update] = searchNeighbor( costmap, mapHeuristic, isSearchedMap, node, arrayFrontier)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    costmap_update = costmap;
    isSearchedMap_update = isSearchedMap;
    new_node = [];
    arrayFrontier_new = [];
    p = [node(3),node(4)];
    for i=-1:1
        for j=-1:1
           if ((p(1) + i > 0) && (p(2) + j > 0))
                if ( isSearchedMap(p(1)+i, p(2)+j) == 0) 
                    isSearchedMap_update(p(1)+i, p(2)+j) = 1; 
                    cost_prev = node(6) + findDistance([p(1)+i, p(2)+j], [p(1), p(2)]);
                    value = findDistance([p(1)+i, p(2)+j],p_goal) + mapHeuristic(p(1)+i, p(2)+j);
                    if(value < costmap(p(1)+i, p(2)+j))
                        costmap_update(p(1)+i, p(2)+j) = costmap(p(1)+i, p(2)+j);
                    end


                    map_out(p(1)+i, p(2)+j) = value;
                    n = n+1;
                    new_node = [new_node; n, value, p(1)+i, p(2)+j, node(1), 1, cost_prev];
                    arrayFrontier_new = [arrayFrontier_new; n, value, p(1)+i, p(2)+j];
                
                end
           end
        end
    end


end



