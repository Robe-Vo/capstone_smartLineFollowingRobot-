function line_following_program()
    % Load network

    % Load config

    % Change robot mode to OPERATION 

    % Line following loop
    while true % might use some flag to stop
        % Get signals

        % Line signal process

        % Logic control
        % 1. Determine road
        % 2. Determine action:
        % 2.1    Get controller + tune paramters (for drive and steer)
        % 2.2    Stop
        % 2.3    Keep running + steering at angle 
        %        (when there is special line path or robot go outline)
        % 2.4    Run base on app control

        % 3. Control drive motor
        
        % 4. Control steer servor

        % 5. Send signal control to robot

        % 6. Wait
    end
        
    % End program -> Send signal to change robot mode to IDLE
    % This option may be stop
end
