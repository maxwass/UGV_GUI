classdef waypointGenerator
    %waypointGenerator class stores a 2d array called path (
        %path = [ 43.12342, 76.234234; 45.345345,65.34535; .... ]
        %nextPointer = index in path of waypoint we are currently going to
    %Its methods return the updated waypoint to go to
    properties
        path
        nextPointer
        noPath
    end
    
    methods
        function obj = waypointGenerator(newPath)
            if (isnan(newPath))
                obj.noPath = true;
            else
                obj.noPath = false;
                obj.path = newPath;
            end
            obj.nextPointer = 1;
        end
%         function obj = waypointGenerator()
%             obj.noPath = true;
%         end
        
        
        function nextWaypoint = getNextWaypoint(this, currentGPS)
            if (this.noPath == true)
               disp("WayPoint Generator: No Path Entered!!")
               nextWaypoint = NaN;
               return;
           end
            
            %reached next waypoint?
            if this.reachedWaypoint(this.path(this.nextPointer, :), currentGPS, 1)
                %if yes increment nextPointer as long as not at end of path
               if(objnextPointer < this.path.length)
                   this.nextPointer = this.nextPointer + 1; 
               else
                   %TRIGGER END OF PATH
               end   
            end
            nextWaypoint = this.path(this.nextPointer,:);
        end
        
        function b = reachedWaypoint(this, currentGPS, nextWaypointGPS, distanceThreshold)
           %% GPS comes in as [long, lattitude]
           
           if (this.noPath == true)
               disp("WayPoint Generator: No Path Entered!!")
               b = false;
               return;
           end
           
           
           %%if we are within threshold (m) then increment pointer
           
           %needs to be tested! - flip input gps for correct long/lat order
           %[distKM] = lldistkm(fliplr(currentGPS), fliplr(nextWaypointGPS));
           %distM = distKM*1000;
           
           xy_waypoint = llToMeters(nextWaypointGPS(1),nextWaypointGPS(2));
           xy_jackal = llToMeters(currentGPS(1), currentGPS(2));
           
           distM  = pdist([xy_waypoint; xy_jackal], 'euclidean');
           
           
           if(distM < distanceThreshold)
               b = true;
           else
               b = false;
           end
          
        end

        function changePath(this, newPath)
            this.path = newPath;
            this.nextPointer = 1;
        end
        
    end
    
end

