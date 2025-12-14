function rt = makeRuntimeFrame()
    rt = struct();
    rt.t = 0.0;
    rt.meas = struct("lineRaw",zeros(1,5),"ultra",0,"mpu",zeros(1,6),"encCount",0,"encSpeed",0);
    rt.proc = struct("lineNorm",zeros(1,5),"mask",false(1,5),"error",0.0);
    rt.state = struct("roadId",uint16(0),"condition","NORMAL","controller","PID","useGlobalTune",true);
    rt.tune = struct("steer",struct());
    rt.cmd = struct("steer",struct("cmd",uint8(0),"speedU8",uint8(0),"angleU16",uint16(0)));
end
