classdef DroneControllerMATLAB < handle
    % Odwzorowanie Python DroneController (drone_interfaces) w MATLAB.
    % Actions/services handlera sa pod namespace /knr_hardware/.
    % Wymaga: ros2genmsg('/home/hozaz/matlab_interfaces') wykonane.
    %
    % Uzycie:
    %   d = DroneControllerMATLAB();
    %   d.arm(); d.takeoff(10); d.goto_global(47.398,8.549,20); d.land(); d.stop();

    properties
        Node
        NS = "/knr_hardware"          % namespace handlera
        ArmClient
        TakeoffClient
        GotoGlobalClient
        GotoRelativeClient
        YawClient
        ModeClient
        TelemetrySub
        Telemetry = struct('battery_voltage',NaN,'lat',NaN,'lon',NaN,'alt',NaN, ...
                           'global_lat',NaN,'global_lon',NaN,'flight_mode','', ...
                           'roll',NaN,'pitch',NaN,'yaw',NaN,'speed',NaN);
        Alarm = false
        VoltageThreshold = 12.0
        VoltageSpikes = 0
    end

    methods
        function obj = DroneControllerMATLAB(nodeName, ns)
            if nargin < 1 || isempty(nodeName), nodeName = "/matlab_drone_controller"; end
            if nargin >= 2 && ~isempty(ns), obj.NS = ns; end
            obj.Node = ros2node(nodeName);

            % --- action clients (pod namespace) ---
            obj.ArmClient          = ros2actionclient(obj.Node, obj.NS+"/Arm",          "drone_interfaces/Arm");
            obj.TakeoffClient      = ros2actionclient(obj.Node, obj.NS+"/takeoff",      "drone_interfaces/Takeoff");
            obj.GotoGlobalClient   = ros2actionclient(obj.Node, obj.NS+"/goto_global",  "drone_interfaces/GotoGlobal");
            obj.GotoRelativeClient = ros2actionclient(obj.Node, obj.NS+"/goto_relative","drone_interfaces/GotoRelative");
            obj.YawClient          = ros2actionclient(obj.Node, obj.NS+"/Set_yaw",      "drone_interfaces/SetYawAction");

            % --- service client ---
            obj.ModeClient = ros2svcclient(obj.Node, obj.NS+"/set_mode", "drone_interfaces/SetMode");

            % --- telemetria (sprawdz nazwe topiku jesli inna) ---
            try
                obj.TelemetrySub = ros2subscriber(obj.Node, obj.NS+"/telemetry", ...
                    "drone_interfaces/Telemetry", @(m) obj.telemetryCb(m), "Reliability","besteffort");
            catch
                warning("Nie podpieto telemetrii (%s/telemetry) — sprawdz nazwe topiku.", obj.NS);
            end

            fprintf("DroneControllerMATLAB: czekam na serwery akcji (%s/*)...\n", obj.NS);
            obj.waitForServers();
            fprintf("DroneControllerMATLAB: gotowy.\n");
        end

        function waitForServers(obj)
            clients = {obj.ArmClient, obj.TakeoffClient, obj.GotoGlobalClient, ...
                       obj.GotoRelativeClient, obj.YawClient};
            names = {"Arm","takeoff","goto_global","goto_relative","Set_yaw"};
            for i = 1:numel(clients)
                ok = waitForServer(clients{i}, "Timeout", 10);
                if ~ok
                    warning("Serwer '%s/%s' niedostepny (10s). Czy drone_handler_px4 dziala?", obj.NS, names{i});
                end
            end
        end

        function ok = set_mode(obj, mode)
            if ~isServerAvailable(obj.ModeClient)
                waitForServer(obj.ModeClient, "Timeout", 10);
            end
            req = ros2message(obj.ModeClient);
            req.mode = mode;
            try
                call(obj.ModeClient, req, "Timeout", 10);
                fprintf("Mode -> %s\n", mode); ok = true;
            catch e
                % SetMode.srv ma pusty response -> call() rzuca CallError mimo ze
                % handler wykonal komende. Traktuj CallError jako sukces.
                if contains(e.identifier, "CallError")
                    fprintf("Mode -> %s (pusty response, OK)\n", mode); ok = true;
                else
                    warning("set_mode '%s' failed [%s]: %s", mode, e.identifier, e.message); ok = false;
                end
            end
        end
        function ok = arm(obj)
            % handler mapuje 'GUIDED' -> PX4 offboard (DO_SET_MODE param2=6)
            fprintf("Offboard (GUIDED)...\n");
            if ~obj.set_mode("GUIDED"), ok = false; return; end
            pause(2);   % PX4 potrzebuje chwili by wejsc w offboard (nav_state=14)
            fprintf("Arming...\n");
            goal = ros2message(obj.ArmClient);
            ok = obj.sendAndWait(obj.ArmClient, goal, "Arm");
        end

        function ok = takeoff(obj, altitude)
            fprintf("Takeoff %.1f m\n", altitude);
            goal = ros2message(obj.TakeoffClient);
            goal.altitude = single(altitude);
            ok = obj.sendAndWait(obj.TakeoffClient, goal, "Takeoff");
        end

        function ok = goto_global(obj, lat, lon, alt)
            fprintf("Goto LAT:%.6f LON:%.6f ALT:%.1f\n", lat, lon, alt);
            goal = ros2message(obj.GotoGlobalClient);
            goal.lat = single(lat); goal.lon = single(lon); goal.alt = single(alt);
            ok = obj.sendAndWait(obj.GotoGlobalClient, goal, "GotoGlobal");
        end

        function ok = goto_relative(obj, north, east, down)
            fprintf("Goto rel N:%.1f E:%.1f D:%.1f\n", north, east, down);
            goal = ros2message(obj.GotoRelativeClient);
            goal.north = single(north); goal.east = single(east); goal.down = single(down);
            ok = obj.sendAndWait(obj.GotoRelativeClient, goal, "GotoRelative");
        end

        function ok = set_yaw(obj, yaw, relative)
            if nargin < 3, relative = true; end
            goal = ros2message(obj.YawClient);
            goal.yaw = single(yaw); goal.relative = logical(relative);
            ok = obj.sendAndWait(obj.YawClient, goal, "SetYaw");
        end

        function ok = land(obj)
            ok = obj.set_mode("LAND"); if ok, fprintf("Landing...\n"); end
        end

        function ok = rtl(obj)
            ok = obj.set_mode("RTL"); if ok, fprintf("RTL...\n"); end
        end

        function ok = sendAndWait(obj, client, goal, label)
            if obj.Alarm
                warning("ALARM — akcja '%s' zablokowana.", label); ok = false; return;
            end
            try
                gh = sendGoal(client, goal);
                [result, status] = getResult(gh);
                fprintf("Akcja %s: status=%d result=%d\n", label, status, result.result);
                ok = (status == 1);
            catch e
                warning("Akcja '%s' failed: %s", label, e.message); ok = false;
            end
        end

        function telemetryCb(obj, msg)
            obj.Telemetry.battery_voltage = msg.battery_voltage;
            obj.Telemetry.lat = msg.lat; obj.Telemetry.lon = msg.lon; obj.Telemetry.alt = msg.alt;
            obj.Telemetry.global_lat = msg.global_lat; obj.Telemetry.global_lon = msg.global_lon;
            obj.Telemetry.flight_mode = char(msg.flight_mode);
            obj.Telemetry.roll = msg.roll; obj.Telemetry.pitch = msg.pitch; obj.Telemetry.yaw = msg.yaw;
            obj.Telemetry.speed = msg.speed;
            if msg.battery_voltage < obj.VoltageThreshold
                obj.VoltageSpikes = obj.VoltageSpikes + 1;
            else
                obj.VoltageSpikes = 0;
            end
            if obj.VoltageSpikes >= 5 && ~obj.Alarm
                warning("Niskie napiecie — ALARM."); obj.Alarm = true;
            end
        end

        function stop(obj)
            clear obj.TelemetrySub obj.ArmClient obj.TakeoffClient ...
                  obj.GotoGlobalClient obj.GotoRelativeClient obj.YawClient ...
                  obj.ModeClient obj.Node;
        end
    end
end
