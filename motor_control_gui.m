function motor_control_gui
    % MOTOR_CONTROL_GUI
    % GUI for simulating a DC motor control system with PID controllers.
    % Allows user input of motor and PID parameters, runs simulations,
    % loads experimental data, and saves simulation results.
    %
    % The GUI displays:
    %  - Step response plots for open-loop, P, PI, and PID controllers
    %  - PID disturbance rejection plot

    % Center the figure window
    screenSize = get(0, 'ScreenSize');  
    figWidth = 820;
    figHeight = 700;
    figLeft = (screenSize(3) - figWidth) / 2;
    figBottom = (screenSize(4) - figHeight) / 2;

    % Create main figure window with fixed size and title
    fig = figure('Name', 'DC Motor Control Simulator', ...
        'Position', [figLeft, figBottom, figWidth, figHeight], ...
        'Resize', 'off');

    % Define default motor and PID parameters
    defaults.R = 1;     % Resistance (Ohms)
    defaults.L = 0.5;   % Inductance (Henries)
    defaults.Ke = 0.01; % Back EMF constant
    defaults.Kt = 0.01; % Torque constant
    defaults.J = 0.01;  % Inertia
    defaults.b = 0.1;   % Damping coefficient
    defaults.Kp = 100;  % PID Proportional gain
    defaults.Ki = 200;  % PID Integral gain
    defaults.Kd = 1;    % PID Derivative gain
    defaults.t = 0:0.01:2; % Simulation time vector

    % Initialize structure to hold experimental and simulation data
    data.expTime = [];
    data.expSpeed = [];
    data.sim = [];

    % Create UI controls for motor parameters input
    uicontrol('Style', 'text', 'Position', [20 630 150 20], ...
        'String', 'Resistance R (Ohms):', 'HorizontalAlignment', 'left');
    hR = uicontrol('Style', 'edit', 'Position', [180 630 60 25], ...
        'String', num2str(defaults.R));

    uicontrol('Style', 'text', 'Position', [20 595 150 20], ...
        'String', 'Inductance L (H):', 'HorizontalAlignment', 'left');
    hL = uicontrol('Style', 'edit', 'Position', [180 595 60 25], ...
        'String', num2str(defaults.L));

    uicontrol('Style', 'text', 'Position', [20 560 150 20], ...
        'String', 'Back EMF constant Ke:', 'HorizontalAlignment', 'left');
    hKe = uicontrol('Style', 'edit', 'Position', [180 560 60 25], ...
        'String', num2str(defaults.Ke));

    uicontrol('Style', 'text', 'Position', [20 525 150 20], ...
        'String', 'Torque constant Kt:', 'HorizontalAlignment', 'left');
    hKt = uicontrol('Style', 'edit', 'Position', [180 525 60 25], ...
        'String', num2str(defaults.Kt));

    uicontrol('Style', 'text', 'Position', [20 490 150 20], ...
        'String', 'Inertia J:', 'HorizontalAlignment', 'left');
    hJ = uicontrol('Style', 'edit', 'Position', [180 490 60 25], ...
        'String', num2str(defaults.J));

    uicontrol('Style', 'text', 'Position', [20 455 150 20], ...
        'String', 'Damping b:', 'HorizontalAlignment', 'left');
    hb = uicontrol('Style', 'edit', 'Position', [180 455 60 25], ...
        'String', num2str(defaults.b));

    % Create UI controls for PID parameters input
    uicontrol('Style', 'text', 'Position', [20 410 150 20], ...
        'String', 'P gain Kp:', 'HorizontalAlignment', 'left');
    hKp = uicontrol('Style', 'edit', 'Position', [180 410 60 25], ...
        'String', num2str(defaults.Kp));

    uicontrol('Style', 'text', 'Position', [20 375 150 20], ...
        'String', 'I gain Ki:', 'HorizontalAlignment', 'left');
    hKi = uicontrol('Style', 'edit', 'Position', [180 375 60 25], ...
        'String', num2str(defaults.Ki));

    uicontrol('Style', 'text', 'Position', [20 340 150 20], ...
        'String', 'D gain Kd:', 'HorizontalAlignment', 'left');
    hKd = uicontrol('Style', 'edit', 'Position', [180 340 60 25], ...
        'String', num2str(defaults.Kd));

    % Create buttons for user actions
    btnRun = uicontrol('Style', 'pushbutton', 'String', 'Run Simulation', ...
        'Position', [50 250 140 30], 'Callback', @runSimulation);

    btnLoad = uicontrol('Style', 'pushbutton', 'String', 'Load Experimental Data', ...
        'Position', [50 210 140 30], 'Callback', @loadData);

    btnReset = uicontrol('Style', 'pushbutton', 'String', 'Reset to Defaults', ...
        'Position', [50 170 140 30], 'Callback', @resetDefaults);

    btnSave = uicontrol('Style', 'pushbutton', 'String', 'Save Simulation Data', ...
        'Position', [50 130 140 30], 'Callback', @saveData);

    % Status message text box to show user feedback and errors
    hStatus = uicontrol('Style', 'text', 'Position', [20 50 220 25], ...
        'String', 'Ready', 'ForegroundColor', [0 0.5 0], 'HorizontalAlignment', 'left');

    % Create axes for plotting simulation results
    ax1 = axes('Units', 'pixels', 'Position', [300 400 480 250]);
    title(ax1, 'Step Response with Controllers');
    xlabel(ax1, 'Time (s)');
    ylabel(ax1, 'Motor Speed (rad/s)');
    grid(ax1, 'on');

    ax2 = axes('Units', 'pixels', 'Position', [300 70 480 250]);
    title(ax2, 'PID Disturbance Rejection');
    xlabel(ax2, 'Time (s)');
    ylabel(ax2, 'Motor Speed (rad/s)');
    grid(ax2, 'on');

    % Nested helper function to read parameters from UI
    function [params, errMsg] = readParams()
        % READPARAMS reads and validates motor and PID parameters
        % Returns a struct 'params' with numeric values
        % Returns error message string if invalid inputs are detected
        params = struct();
        errMsg = '';

        params.R = str2double(hR.String);
        params.L = str2double(hL.String);
        params.Ke = str2double(hKe.String);
        params.Kt = str2double(hKt.String);
        params.J = str2double(hJ.String);
        params.b = str2double(hb.String);
        params.Kp = str2double(hKp.String);
        params.Ki = str2double(hKi.String);
        params.Kd = str2double(hKd.String);

        % Validate all parameters are numeric
        if any(structfun(@(x) isnan(x), params))
            errMsg = 'All parameters must be numeric.';
            return;
        end

        % Validate positive physical parameters
        if params.R <= 0 || params.L <= 0 || params.J <= 0
            errMsg = 'Resistance (R), Inductance (L), and Inertia (J) must be positive.';
            return;
        end

        % Validate PID gains non-negative
        if params.Kp < 0 || params.Ki < 0 || params.Kd < 0
            errMsg = 'PID gains (Kp, Ki, Kd) must be non-negative.';
            return;
        end
    end

    % Nested function to run the motor simulation
    function runSimulation(~,~)
        % RUNSIMULATION executes the motor control simulation using the
        % current UI parameters and plots the results

        [params, errMsg] = readParams();
        if ~isempty(errMsg)
            set(hStatus, 'String', ['Error: ' errMsg], 'ForegroundColor', 'r');
            return;
        end

        set(hStatus, 'String', 'Running simulation...', 'ForegroundColor', [0 0.5 0]);

        t = defaults.t;

        % Construct the motor transfer function G(s) = Kt / (J*L*s^2 + (J*R + b*L)*s + b*R + Kt*Ke)
        num = params.Kt;
        den = [params.J*params.L, params.J*params.R + params.b*params.L, ...
               params.b*params.R + params.Kt*params.Ke];
        G = tf(num, den);

        % Calculate open loop step response
        y_open = step(G, t);

        % Define controllers and closed-loop systems
        C_P = pid(params.Kp, 0, 0);
        T_P = feedback(C_P * G, 1);
        y_p = step(T_P, t);

        C_PI = pid(params.Kp*0.6, params.Ki*0.6, 0);
        T_PI = feedback(C_PI * G, 1);
        y_pi = step(T_PI, t);

        C_PID = pid(params.Kp, params.Ki, params.Kd);
        T_PID = feedback(C_PID * G, 1);
        y_pid = step(T_PID, t);

        % Store simulation results in data struct for saving or plotting
        data.sim.t = t;
        data.sim.open = y_open;
        data.sim.p = y_p;
        data.sim.pi = y_pi;
        data.sim.pid = y_pid;

        % Plot step response results
        axes(ax1);
        cla(ax1);
        plot(t, y_open, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Open Loop'); hold on;
        plot(t, y_p, 'g-.', 'LineWidth', 1.5, 'DisplayName', 'P Control');
        plot(t, y_pi, 'm-', 'LineWidth', 1.5, 'DisplayName', 'PI Control');
        plot(t, y_pid, 'b-', 'LineWidth', 1.5, 'DisplayName', 'PID Control');

        % Plot experimental data points if available
        if ~isempty(data.expTime)
            plot(data.expTime, data.expSpeed, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k', ...
                'DisplayName', 'Experimental Data');
        end

        legend('Location', 'northeastoutside');
        xlabel('Time (s)');
        ylabel('Motor Speed (rad/s)');
        title('Step Response with Controllers');
        grid on;
        hold off;

        % Simulate disturbance rejection for PID controller
        d = [zeros(1, 100), 0.2 * ones(1, length(t) - 100)];
        y_dist = lsim(T_PID, d, t);

        % Plot disturbance rejection results
        axes(ax2);
        cla(ax2);
        plot(t, y_pid, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Step Input Only'); hold on;
        plot(t, y_dist, 'k--', 'LineWidth', 1.5, 'DisplayName', 'With Disturbance');
        legend('Location', 'best');
        xlabel('Time (s)');
        ylabel('Motor Speed (rad/s)');
        title('PID Controller: Disturbance Rejection');
        grid on;
        hold off;

        set(hStatus, 'String', 'Simulation completed successfully.', 'ForegroundColor', [0 0.5 0]);
    end

    % Nested function to load experimental data from CSV
    function loadData(~,~)
        % LOADDATA opens a file dialog for the user to select a CSV file
        % containing experimental data with time and speed columns
        [file, path] = uigetfile('*.csv', 'Select Experimental Data CSV');
        if isequal(file, 0)
            set(hStatus, 'String', 'Load cancelled.', 'ForegroundColor', [0 0.5 0]);
            return;
        end
        filename = fullfile(path, file);
        try
            raw = readmatrix(filename);
            if size(raw,2) < 2
                error('Data file must have at least two columns: time and speed.');
            end
            data.expTime = raw(:,1);
            data.expSpeed = raw(:,2);
            set(hStatus, 'String', ['Loaded experimental data from ' file], 'ForegroundColor', [0 0.5 0]);
        catch ME
            set(hStatus, 'String', ['Error loading data: ' ME.message], 'ForegroundColor', 'r');
        end
    end

    % Nested function to reset all UI inputs to default values
    function resetDefaults(~,~)
        % RESETDEFAULTS sets all input fields back to the initial default parameters
        hR.String = num2str(defaults.R);
        hL.String = num2str(defaults.L);
        hKe.String = num2str(defaults.Ke);
        hKt.String = num2str(defaults.Kt);
        hJ.String = num2str(defaults.J);
        hb.String = num2str(defaults.b);
        hKp.String = num2str(defaults.Kp);
        hKi.String = num2str(defaults.Ki);
        hKd.String = num2str(defaults.Kd);
    
        data.expTime = [];
        data.expSpeed = [];
        data.sim = [];
    
        % Clear axes and turn off legends
        cla(ax1);
        legend(ax1, 'off');
    
        cla(ax2);
        legend(ax2, 'off');
    
        set(hStatus, 'String', 'Inputs reset to default values.', 'ForegroundColor', [0 0.5 0]);
    end


    % Nested function to save the latest simulation data
    function saveData(~,~)
        % SAVEDATA allows the user to save the latest simulation results to a CSV file
        if isempty(data.sim)
            set(hStatus, 'String', 'No simulation data to save.', 'ForegroundColor', 'r');
            return;
        end
        [file, path] = uiputfile('simulation_results.csv', 'Save Simulation Data As');
        if isequal(file,0)
            set(hStatus, 'String', 'Save cancelled.', 'ForegroundColor', [0 0.5 0]);
            return;
        end
        filename = fullfile(path, file);
        % Prepare data matrix for saving: columns are time, open, P, PI, PID
        toSave = [data.sim.t(:), data.sim.open(:), data.sim.p(:), data.sim.pi(:), data.sim.pid(:)];
        header = {'Time','OpenLoop','P_Control','PI_Control','PID_Control'};
        try
            % Write header
            fid = fopen(filename, 'w');
            fprintf(fid, '%s,%s,%s,%s,%s\n', header{:});
            fclose(fid);
            % Append data
            dlmwrite(filename, toSave, '-append');
            set(hStatus, 'String', ['Simulation data saved to ' file], 'ForegroundColor', [0 0.5 0]);
        catch ME
            set(hStatus, 'String', ['Error saving data: ' ME.message], 'ForegroundColor', 'r');
        end
    end
end