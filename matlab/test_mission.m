% test_mission.m — odpowiednik Twojego Python main().
% WYMAGA: drone_handler_px4 dziala (wystawia actions/services),
%         ros2genmsg('/home/hozaz/matlab_interfaces') wykonane,
%         FASTRTPS profil UDP ustawiony (inaczej cisza na DDS).

% --- wymus UDP transport (inaczej subscribery/actions moga nie dostawac danych) ---
setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/home/hozaz/fastdds_udp.xml');
setenv('ROS_DOMAIN_ID', '0');

% --- kontroler ---
d = DroneControllerMATLAB();

% --- prosta misja (jak Python main) ---
d.arm();
d.takeoff(10.0);
pause(5);                       % stabilizacja po takeoff

% kwadrat 25 m (odkomentuj gdy podstawowe dziala):
% base_lat = 47.398136; base_lon = 8.549139; alt = 20.0; meters = 25.0;
% dlat = meters / 111320;
% dlon = meters / (111320 * cosd(base_lat));
% d.goto_global(base_lat,        base_lon,        alt);
% d.goto_global(base_lat,        base_lon+dlon,   alt);
% d.goto_global(base_lat+dlat,   base_lon+dlon,   alt);
% d.goto_global(base_lat+dlat,   base_lon,        alt);

d.land();
d.stop();
