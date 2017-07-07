function ros_comm_callback(hObject, eventdata, handles)

global pub_state;
global pub_waypoint;
global waypoint_generator;
global msg_gps
global jackal_state;

%%Use the global gps coordinate of jackal and feed it into the
%%getNextWaypoint method. This will check if we've reached next waypoint.
%%If we have, it increment so we will travel to the subsequent waypoint. if
%%not we continute to track current waypoint
msg_waypoint = rosmessage('std_msgs/Float64MultiArray');
[waypoint_generator, msg_waypoint.Data, endOfPath] = waypoint_generator.getNextWaypoint([msg_gps.Longitude, msg_gps.Latitude]);

if endOfPath 
end

msg_state = rosmessage('std_msgs/Int8');
msg_state.Data = jackal_state;

send(pub_state,msg_state);

if ~isnan(msg_waypoint.Data)
    send(pub_waypoint,msg_waypoint);
end

if true
    if (endOfPath == true)
        printf("  Reached end of path!!!\n");
    end

    fprintf(' current gps: %f, %f \n', msg_gps.Longitude, msg_gps.Latitude);

    if isnan(msg_waypoint.Data)
        fprintf(' next_waypoint: NAN \n');
    else
       fprintf(' next_waypoint: %f, %f, \n Index in Path: %i \n\n', msg_waypoint.Data(1), msg_waypoint.Data(2), waypoint_generator.getIndex());
    end
end


end