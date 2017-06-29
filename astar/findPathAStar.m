arfunction path = findPathAStar( obs, x_map, y_map, start, goal )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%% bias
x_bias = min(x_map)-1;
y_bias = min(y_map)-1;
x_map = x_map - x_bias;
y_map = y_map - y_bias;

start(1) = start(1) - x_bias;
start(2) = start(2) - y_bias;

goal(1) = goal(1) - x_bias;
goal(2) = goal(2) - y_bias;
if(length(obs)~=0)
    for i=1:length(obs)
       obs{i}(:,1) =  obs{i}(:,1) - x_bias;
       obs{i}(:,2) =  obs{i}(:,2) - y_bias;
    end
end

%% debug plot
% figure(99)
% plot(goal(1), goal(2),'x','linewidth',2,'color','r')
% hold on
% plot(start(1), start(2),'o','linewidth',2,'color','r')
% for i=1:length(obs)
%     h4 = fill(obs{i}(:,1),obs{i}(:,2),'r');
%     set(h4,'facealpha',.5)
% end
%%

initialCost = 999;   
costmap_init = ones(max(x_map),max(y_map))*initialCost;  %  initialCost should be bigger than possible cost in the map
mapSize = size(costmap_init);
isSearchedMap = zeros(size(costmap_init)); % 0: not visited node, 1: visited node
mapHeuristic = findHeuristicMap(size(costmap_init),goal); % heuristic map with euclidian distance
searchList = [];

%costmap=occupyObsMap(costmap_init,obs1); % occupy the map with obstacle
%costmap=occupyObsMap(costmap,obs2);
if (length(obs) == 0)
    costmap = costmap_init;
end
for i=1:length(obs)
    costmap = occupyObsMap(costmap_init,obs{i});
    costmap_init = costmap;
end


%% Initialization
initVal = findDistance(start,goal); 
isSearchedMap(start(1), start(2))=1;
costmap(start(1),start(2)) = initVal;
node=[1,initVal,start(1),start(2),0,0]; % [ID, cost, x, y, ID_prev,  cost from start position to this position]
[costmap_update, searchListNew] = updateCostMap( costmap, mapHeuristic, isSearchedMap, node);
costmap = costmap_update;
searchList=[searchList;searchListNew]; % [cost, x, y, node ID, cost from start position to this position]
cnt=0;
%% Main Loop
while(1)
    [r,c]=find(searchList==min(searchList(:,1))); % find an index (r) of a not-visited node that has lowest cost 
    cnt=cnt+1;

    node = [node; size(node,1)+1, searchList(r(1),1), searchList(r(1),2), searchList(r(1),3), searchList(r(1),4),  searchList(r(1),5)]; % update node
    isSearchedMap(searchList(r(1),2), searchList(r(1),3)) = 1; % update isSearchedMap
    searchList(r(1),:)=[];

    if((node(size(node,1),3) == goal(1)) && (node(size(node,1),4) == goal(2)))
        break;
    end
    [costmap_update, searchListNew] = updateCostMap( costmap, mapHeuristic, isSearchedMap, node(size(node,1),:));
    costmap = costmap_update;
    searchList=[searchList;searchListNew]; % update searchList

end

%% Find Optimal Path
path = [node(size(node,1),3),node(size(node,1),4)]; 
index_next = node(size(node,1),5);
while(1)
    path=[node(index_next,3),node(index_next,4);path];
    index_next = node(index_next,5);
    if(index_next == 0)
        break;
    end
end
%% Unbias

x_map = x_map + x_bias;
y_map = y_map + y_bias;

start(1) = start(1) + x_bias;
start(2) = start(2) + y_bias;

goal(1) = goal(1) + x_bias;
goal(2) = goal(2) + y_bias;

path(:,1) = path(:,1) + x_bias;
path(:,2) = path(:,2) + y_bias;

for i=1:length(obs)
   obs{i}(:,1) =  obs{i}(:,1) + x_bias;
   obs{i}(:,2) =  obs{i}(:,2) + y_bias;
end


end

