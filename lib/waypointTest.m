%%testcode for waypoint generator

%% test distance function

%gps coordinate middle interstion 33rd and walnut
latt_33 = 39.952553;
long_33 = -75.190033;
latt_long_33 = [latt_33, long_33];

%gps coordinate next to middle interstion 33rd and walnut
latt_33_next = 39.952553;
long_33_next = -75.190025;
latt_long_33_next = [latt_33_next, long_33_next];

%gps coordinates red block by moore and intersection
latt_red = 39.952509;
long_red = -75.190209;
latt_long_red = [latt_red, long_red];

%gps coordinate of 42nd and walnut
latt_42 = 39.954576;
long_42 = -75.206484;
latt_long_42 = [latt_42, long_42];

%assert(lldistkm(*1000 == 0);
assert(lldistkm(latt_long_33, latt_long_33_next)*1000 < 1);
assert(lldistkm(latt_long_33, latt_long_red)*1000 > 1);
assert(lldistkm(latt_long_33, latt_long_42)*1000 > 1);

%% test waypoint class

%constructor
%create fake path _ long/latt
path_1 = [-75.183154, 39.951751;
          -75.185515, 39.952008;
          -75.187189, 39.952172;
          -75.189046, 39.952416];

waypoint_generator = waypointGenerator(path_1);

waypoint_generator.reachedWaypoint(latt_long_33,latt_long_42, 1)


%% test get waypoint

waypoint_generator = waypoint_generator.getNextWaypoint(path_1(1,:) - .001)

%% test Change 

path_flipped = fliplr(path_1);
waypoint_generator = waypoint_generator.setPath(path_flipped)

