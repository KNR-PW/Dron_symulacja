% matlab_ros2_setup_ONCE.m
% Uruchom RAZ, po instalacji MATLAB / na nowej maszynie.
% Buduje typy px4_msgs + drone_interfaces dla ROS Toolbox.
% WYMAGANIA WSTEPNE (zrobic w WSL PRZED tym skryptem):
%   - Python 3.10 (deadsnakes) + python3.10-dev + python3.10-venv
%   - gcc-12 g++-12
%   - foldery ~/matlab_msgs/px4_msgs i ~/matlab_interfaces/drone_interfaces
%     (skopiowane z src/, drone_interfaces przepuszczone przez iconv->ASCII)
%   - MATLAB odpalony z:  export CC=/usr/bin/gcc-12; export CXX=/usr/bin/g++-12; matlab

home = getenv('HOME');
if isempty(home), home = fullfile('/home', getenv('USER')); end

pyenv('Version', '/usr/bin/python3.10');   % ROS Toolbox wymaga 3.9-3.10 do buildu

disp('=== Buduje px4_msgs (kilka minut) ===');
ros2genmsg(fullfile(home, 'matlab_msgs'));

disp('=== Buduje drone_interfaces (kilka minut) ===');
ros2genmsg(fullfile(home, 'matlab_interfaces'));

addpath(fullfile(home, 'matlab_msgs/matlab_msg_gen_R2026a/glnxa64/install/m'));
addpath(fullfile(home, 'matlab_interfaces/matlab_msg_gen_R2026a/glnxa64/install/m'));
clear classes;          %#ok<CLCLS>
rehash toolboxcache;

disp('=== Gotowe. Sprawdz: ros2 msg list ===');
disp('=== Skopiuj startup.m do userpath. ===');
