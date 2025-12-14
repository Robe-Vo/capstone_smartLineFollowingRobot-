function [ok, meas] = rxSensorFrame(net, cfg)
    [ok, meas] = net.recvSensorFrame(cfg.timing.rxTimeoutS);
end
