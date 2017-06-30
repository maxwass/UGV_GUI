function ros_comm_callback(hObject, eventdata, handles)

global sub_gps;
global pub_state;
global pub_waypoint;
global waypoint_generator;
global currentGPS;
global jackal_state;

msg_gps = rosmessage('sensor_msgs/NavSatFix');
msg_gps = receive(sub_gps,.1);
currentGPS = [msg_gps.Longitude, msg_gps.Latitude];

msg_waypoint = rosmessage('std_msgs/Float64MultiArray');
msg_waypoint.Data = waypoint_generator.getNextWaypoint(currentGPS);

msg_state = rosmessage('std_msgs/Int8');
msg_state.Data = jackal_state;

send(pub_state,msg_state);
if ~isnan(msg_waypoint.Data)
    send(pub_waypoint,msg_waypoint);
end

end