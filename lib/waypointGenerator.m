classdef waypointGenerator
    %waypointGenerator class stores a 2d array called path (
        %path = [ 43.12342, 76.234234; 45.345345,65.34535; .... ]
        %nextPointer = index in path of waypoint we are currently going to
    %Its methods return the updated waypoint to go to
    properties (SetAccess = private)
        path = []
        nextPointer
        noPath

    end
    
    methods
        %Constructor: feed in a path. Path can be NaN upon initialization.
        %If path is NaN set noPath flag, if path is not NaN regular init
        function obj = waypointGenerator(newPath)
            if (isnan(newPath))
                obj.noPath = true;
            else
                obj.noPath = false;
                obj.path = newPath;
            end
            obj.nextPointer = 1;
        end
        
        %getNextWaypoint: takes in currentGPS and compares it to
        %nextWaypoint. If they are withing some threshold, then increment
        %the index in the path to output the next waypoint
        function [obj, nextWaypoint, endOfPath] = getNextWaypoint(obj, currentGPS)
            
           endOfPath = false;
           
           if (obj.noPath == true)
               nextWaypoint = NaN;
               return;
           end
            
            %reached next waypoint?
            %if yes increment nextPointer as long as not at end of path
            %if we have reached the end of the path, tell user
            next_waypoint = obj.path(obj.nextPointer, :);
            reached_waypoint = obj.reachedWaypoint(currentGPS, next_waypoint, 1);
            
            if reached_waypoint
               if(obj.nextPointer < size(obj.path,1))
                   obj.nextPointer = (obj.nextPointer + 1); 
               else
                   %TRIGGER END OF PATH
                   endOfPath = true;
               end   
            end
            nextWaypoint = obj.path(obj.nextPointer,:);
        end
        
        %reachedWaypoint: compare the distance between current and waypoint
        %GPS coordinates. If we are within distThreshold then increment 
        %pointer in path
        %GPS comes in as [long, lattitude]
        function reached = reachedWaypoint(this, currentGPS, nextWaypointGPS, distanceThreshold)
           
           if (this.noPath == true)
               disp("WayPoint Generator: No Path Entered!!")
               reached = false;
               return;
           end

           xy_waypoint = llToMeters(nextWaypointGPS(1),nextWaypointGPS(2));
           xy_jackal = llToMeters(currentGPS(1), currentGPS(2));
           %distM  = pdist([xy_waypoint; xy_jackal], 'euclidean');
           distM = findDistance( xy_waypoint, xy_jackal );
           
           if(distM < distanceThreshold)
               reached = true;
           else
               reached = false;
           end
          
        end

        function obj = setPath(obj, newPath)
            obj.path = newPath;
            obj.nextPointer = 1;
            obj.noPath = false;
        end
        
        function index = getIndex(obj)
            index = obj.nextPointer;
        end
        
        
    end
    
end

