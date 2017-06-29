function map_out = occupyObsMap( map, obs )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
    s_map = size(map);
    n = size(obs,1);
    cnt = 0;
    map_out = map;
    
    %%
%     figure(98)
%     for i=1:length(obs)
%         h4 = fill(obs(:,1),obs(:,2),'r');
%         set(h4,'facealpha',.5)
%     end
%%
    for i = 1:s_map(1)
       for j = 1:s_map(2)
           p = [i,j];
 %          if(isInside(obs, n,p))
            if(inpolygon(i,j,obs(:,1),obs(:,2)))
              cnt = cnt + 1;
              map_out(i,j) = 1000;
           end
       end
    end
    
end

