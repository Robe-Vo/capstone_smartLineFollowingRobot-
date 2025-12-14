function G = GainTable_default(cfg)

    G = struct();

    G.GLOBAL.PID = struct("Kp",cfg.pidDefault.Kp,"Ki",cfg.pidDefault.Ki,"Kd",cfg.pidDefault.Kd,"intMax",cfg.pidDefault.intMax);
    G.GLOBAL.ONOFF = struct("threshold",0.15,"angleStep",12);

    G.UNSTABLE.PID = struct("Kp",0.6*cfg.pidDefault.Kp,"Ki",cfg.pidDefault.Ki,"Kd",0.6*cfg.pidDefault.Kd,"intMax",cfg.pidDefault.intMax);
    G.UNSTABLE.ONOFF = struct("threshold",0.20,"angleStep",10);

    G.LOST_LINE.PID = struct("Kp",0.0,"Ki",0.0,"Kd",0.0,"intMax",0.0);
    G.LOST_LINE.ONOFF = struct("threshold",0.05,"angleStep",25);
end
