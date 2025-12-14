function st = makeAppStateDefault(cfg) %#ok<INUSD>
st = struct();
st.run = struct("mode","AUTO");
st.control = struct("type","PID");
st.tune = struct("useGlobal", true);
st.road = struct("id",uint16(0),"forceState",false,"forcedStateId",uint16(0));
st.run.manual = struct("enable",false,"speedU8",uint8(0),"angleU16",uint16(0),"cmd",uint8(hex2dec('F1')));
end
