function appState = initAppState()
    appState.mode           = "IDLE";
    appState.roadId         = "UNKNOWN";
    appState.prevRoadId     = "UNKNOWN";
    appState.condition      = "NORMAL";
    appState.lastLineMask   = [0 0 0 0 0];
    appState.controllerType = "PID";
end
