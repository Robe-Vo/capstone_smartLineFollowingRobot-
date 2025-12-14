% Register controller
function reg = ControllerRegistry()
    reg = struct();
    reg.PID = @capstone.control.pidController;
    reg.ONOFF = @capstone.control.onoffController;
end
