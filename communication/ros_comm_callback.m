function ros_comm_callback(hObject, eventdata, handles)

global sub_gps;
global pub_state;
global pub_waypoint;
global waypoint_generator;
global currentGPS;
global msg_gps
global jackal_state;

msg_waypoint = rosmessage('std_msgs/Float64MultiArray');
[waypoint_generator, msg_waypoint.Data] = waypoint_generator.getNextWaypoint([msg_gps.Longitude, msg_gps.Latitude]);
fprintf(' current gps: %f, %f \n', msg_gps.Longitude, msg_gps.Latitude);


if isnan(msg_waypoint.Data)
    fprintf(' next_waypoint: NAN \n');
else
    fprintf(' next_waypoint: %f, %f, index %i \n', msg_waypoint.Data(1), msg_waypoint.Data(2), waypoint_generator.getNextPointer());
end

msg_state = rosmessage('std_msgs/Int8');
msg_state.Data = jackal_state;

send(pub_state,msg_state);
%disp("published state");
if ~isnan(msg_waypoint.Data)
    send(pub_waypoint,msg_waypoint);
else
    %disp("IN ros_comm_callback: waypoint is NAN");
end

end