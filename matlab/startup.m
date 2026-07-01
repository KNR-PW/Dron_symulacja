% startup.m -> skopiuj do userpath (sprawdz: userpath w MATLAB).
% Laduje sie automatycznie przy KAZDYM starcie MATLAB.
% Rejestruje typy ROS2 + wymusza UDP transport ([WSL], inaczej DDS cisza).

home = getenv('HOME');
if isempty(home), home = fullfile('/home', getenv('USER')); end

addpath(fullfile(home, 'matlab_msgs/matlab_msg_gen_R2026a/glnxa64/install/m'));
addpath(fullfile(home, 'matlab_interfaces/matlab_msg_gen_R2026a/glnxa64/install/m'));
setenv('FASTRTPS_DEFAULT_PROFILES_FILE', fullfile(home, 'fastdds_udp.xml'));  % [WSL]
setenv('ROS_DOMAIN_ID', '0');
