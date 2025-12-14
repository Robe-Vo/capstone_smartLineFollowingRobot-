function txControl(net, cfg, cmd)
    net.sendControl(cmd.speedU8, cmd.angleU16, cmd.cmd);
end
