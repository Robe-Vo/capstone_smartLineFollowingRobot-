function encoder_viewer_ui_continuous()
% UI compare encoder continuous + filters + overlays + PRINT METRICS (sync with UI)
% Expected file: encoderStream_continuous.mat
%   encoderStream.allEvents: struct array with fields t_us (uint32), flags (uint8)
%   encoderStream.cmdLog   : optional struct array with fields t_pc (double), pwm (uint8/double)

%% ========================= USER-EDITABLE LAYOUT =========================
UI = struct();

UI.figUnits = "normalized";
UI.figPos   = [0.04 0.06 0.92 0.86];

UI.ctrlPanelPos = [0.01 0.915 0.98 0.08];
UI.row1Y = 0.58;
UI.row2Y = 0.15;
UI.ctrlH = 0.32;

% Base control sizes (normalized to control panel)
UI.wLbl   = 0.050;
UI.wPop   = 0.070;
UI.wNum   = 0.035;
UI.wChk   = 0.030;
UI.wGap   = 0.008;

UI.sliderHeight = 0.26;
UI.wVal         = 0.032;

% IIR block
UI.iirLabelW  = 0.045;
UI.iirSlW     = 0.105;    % alpha slider width (reduced to avoid overlap)
UI.iirBetaSlW = 0.085;    % beta slider width (reduced to avoid overlap)
UI.iirGap     = 0.010;

% PRINT button
UI.btnW       = 0.080;
UI.btnH       = UI.ctrlH;

% Plot panels
UI.rawPanelPos  = [0.01 0.49 0.49 0.41];
UI.medPanelPos  = [0.50 0.49 0.49 0.41];
UI.iir1PanelPos = [0.01 0.03 0.49 0.41];
UI.iir2PanelPos = [0.50 0.03 0.49 0.41];

UI.axPos = [0.08 0.12 0.90 0.82];

%% ========================= LOAD DATA =========================
S = load("encoderStream_continuous.mat");
assert(isfield(S,"encoderStream"), "Missing encoderStream in encoderStream_continuous.mat");
ES = S.encoderStream;

events = ES.allEvents;
assert(~isempty(events), "encoderStream.allEvents is empty.");

hasCMD = isfield(ES,"cmdLog") && ~isempty(ES.cmdLog);

% Decode events
evt_t  = double([events.t_us]) * 1e-6;   % seconds
evt_f  = uint8([events.flags]);
evt_ch = bitand(evt_f, 1);              % 0:A, 1:B

tA = evt_t(evt_ch==0);
tB = evt_t(evt_ch==1);

t0dev = min(evt_t);
tA = tA - t0dev;
tB = tB - t0dev;

% Command series (PC time)
if hasCMD
    tCMD = [ES.cmdLog.t_pc]; tCMD = tCMD(:);
    uCMD = double([ES.cmdLog.pwm]); uCMD = uCMD(:);
    tCMD = tCMD - tCMD(1);
else
    tCMD = []; uCMD = [];
end

tEndDev = max([tA(:); tB(:)]);
if hasCMD
    tEnd = min(tEndDev, max(tCMD));
else
    tEnd = tEndDev;
end
if ~isfinite(tEnd) || tEnd<=0
    tEnd = tEndDev;
end

%% ========================= FIGURE + UI =========================
H = struct();
H.cache = struct(); % last computed data (sync to UI)

H.fig = figure("Name","Encoder Viewer (continuous) RAW / MEDIAN / IIR1 / IIR2", ...
    "NumberTitle","off", "Units",UI.figUnits, "Position",UI.figPos);

H.pnl = uipanel(H.fig, "Units","normalized", "Position",UI.ctrlPanelPos, "Title","Tuning");

x = 0.005; y1 = UI.row1Y; y2 = UI.row2Y; h = UI.ctrlH; gap = UI.wGap;

% -------- Column 1: mode / channel --------
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y1 UI.wLbl h], ...
    "String","mode","HorizontalAlignment","left");
H.popMode = uicontrol(H.pnl,"Style","popupmenu","Units","normalized","Position",[x+UI.wLbl y1 UI.wPop h], ...
    "String",{"accumulate","period","count_dt","compare"});

uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y2 UI.wLbl h], ...
    "String","CH","HorizontalAlignment","left");
H.popCH = uicontrol(H.pnl,"Style","popupmenu","Units","normalized","Position",[x+UI.wLbl y2 UI.wPop h], ...
    "String",{"A","B","BOTH"});

x = x + UI.wLbl + UI.wPop + gap;

% -------- Column 2: time window --------
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y1 UI.wLbl h], ...
    "String","tStart","HorizontalAlignment","left");
H.edT0 = uicontrol(H.pnl,"Style","edit","Units","normalized","Position",[x+UI.wLbl y1 UI.wNum h], ...
    "String","0");

uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y2 UI.wLbl h], ...
    "String","tSpan","HorizontalAlignment","left");
span0 = min(30, max(2, floor(tEnd)));
H.edSpan = uicontrol(H.pnl,"Style","edit","Units","normalized","Position",[x+UI.wLbl y2 UI.wNum h], ...
    "String",num2str(span0));

x = x + UI.wLbl + UI.wNum + gap;

% -------- Column 3: Tw / dtC --------
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y1 UI.wLbl h], ...
    "String","Tw","HorizontalAlignment","left");
H.edTw = uicontrol(H.pnl,"Style","edit","Units","normalized","Position",[x+UI.wLbl y1 UI.wNum h], ...
    "String","0.01");

uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y2 UI.wLbl h], ...
    "String","dtC","HorizontalAlignment","left");
H.edDtC = uicontrol(H.pnl,"Style","edit","Units","normalized","Position",[x+UI.wLbl y2 UI.wNum h], ...
    "String","0.01");

x = x + UI.wLbl + UI.wNum + gap;

% -------- Column 4: MedN / Lag --------
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y1 UI.wLbl h], ...
    "String","MedN","HorizontalAlignment","left");
H.edMedN = uicontrol(H.pnl,"Style","edit","Units","normalized","Position",[x+UI.wLbl y1 UI.wNum h], ...
    "String","11");

uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y2 UI.wLbl h], ...
    "String","Lag","HorizontalAlignment","left");
H.edLag = uicontrol(H.pnl,"Style","edit","Units","normalized","Position",[x+UI.wLbl y2 UI.wNum h], ...
    "String","0.04");

x = x + UI.wLbl + UI.wNum + gap;

% -------- Column 5: CMD / ONOFF --------
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y1 UI.wLbl h], ...
    "String","CMD","HorizontalAlignment","left");
H.chkCMD = uicontrol(H.pnl,"Style","checkbox","Units","normalized", ...
    "Position",[x+UI.wLbl y1 UI.wChk h], "Value", double(hasCMD));

uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y2 UI.wLbl h], ...
    "String","ON/OFF","HorizontalAlignment","left");
H.chkOnOff = uicontrol(H.pnl,"Style","checkbox","Units","normalized", ...
    "Position",[x+UI.wLbl y2 UI.wChk h], "Value", double(hasCMD));

x = x + UI.wLbl + UI.wChk + gap;

% -------- Column 6: Mean / Var (kept LEFT of IIR to avoid overlap) --------
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y1 UI.wLbl h], ...
    "String","Mean","HorizontalAlignment","left");
H.chkMean = uicontrol(H.pnl,"Style","checkbox","Units","normalized", ...
    "Position",[x+UI.wLbl y1 UI.wChk h], "Value", 0);

uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y2 UI.wLbl h], ...
    "String","Var","HorizontalAlignment","left");
H.chkVar = uicontrol(H.pnl,"Style","checkbox","Units","normalized", ...
    "Position",[x+UI.wLbl y2 UI.wChk h], "Value", 0);

x = x + UI.wLbl + UI.wChk + gap;

% -------- IIR block starts AFTER Mean/Var to guarantee no overlap --------
xIIR = x;

% Keep everything inside panel (hard right margin)
rightMargin = 0.99;

% Row1: IIR2 alpha + IIR2 beta (aligned horizontally)
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[xIIR y1 UI.iirLabelW h], ...
    "String","IIR2 α","HorizontalAlignment","left");
H.slA2 = uicontrol(H.pnl,"Style","slider","Units","normalized", ...
    "Position",[xIIR+UI.iirLabelW y1+0.05 UI.iirSlW UI.sliderHeight], ...
    "Min",0,"Max",1,"Value",0.05);
H.txtA2 = uicontrol(H.pnl,"Style","text","Units","normalized", ...
    "Position",[xIIR+UI.iirLabelW+UI.iirSlW+0.004 y1 UI.wVal h], ...
    "String","0.050","HorizontalAlignment","left");

xBeta = xIIR + UI.iirLabelW + UI.iirSlW + 0.004 + UI.wVal + UI.iirGap;

uicontrol(H.pnl,"Style","text","Units","normalized","Position",[xBeta y1 UI.iirLabelW h], ...
    "String","IIR2 β","HorizontalAlignment","left");
H.slB2 = uicontrol(H.pnl,"Style","slider","Units","normalized", ...
    "Position",[xBeta+UI.iirLabelW y1+0.05 UI.iirBetaSlW UI.sliderHeight], ...
    "Min",0,"Max",1,"Value",0.0);
H.txtB2 = uicontrol(H.pnl,"Style","text","Units","normalized", ...
    "Position",[xBeta+UI.iirLabelW+UI.iirBetaSlW+0.004 y1 UI.wVal h], ...
    "String","0.000","HorizontalAlignment","left");

% Row2: IIR1 alpha
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[xIIR y2 UI.iirLabelW h], ...
    "String","IIR1 α","HorizontalAlignment","left");
H.slA1 = uicontrol(H.pnl,"Style","slider","Units","normalized", ...
    "Position",[xIIR+UI.iirLabelW y2+0.05 UI.iirSlW UI.sliderHeight], ...
    "Min",0,"Max",1,"Value",0.05);
H.txtA1 = uicontrol(H.pnl,"Style","text","Units","normalized", ...
    "Position",[xIIR+UI.iirLabelW+UI.iirSlW+0.004 y2 UI.wVal h], ...
    "String","0.050","HorizontalAlignment","left");

% PRINT button: pinned to far right (always visible / clickable)
xBtn = rightMargin - UI.btnW;
H.btnPrint = uicontrol(H.pnl,"Style","pushbutton","Units","normalized", ...
    "Position",[xBtn y2 UI.btnW h], "String","PRINT", ...
    "Callback",@(~,~)printMetrics());

% If IIR controls would collide with the PRINT button, shift beta group slightly left by shrinking widths.
% (Simple safety: do nothing dynamic here; fixed widths above are selected to fit typical panels.)

%% ========================= PLOTS =========================
H.pnlRaw  = uipanel(H.fig,"Units","normalized","Position",UI.rawPanelPos,  "Title","RAW");
H.pnlMed  = uipanel(H.fig,"Units","normalized","Position",UI.medPanelPos,  "Title","MEDIAN");
H.pnlIIR1 = uipanel(H.fig,"Units","normalized","Position",UI.iir1PanelPos, "Title","IIR1");
H.pnlIIR2 = uipanel(H.fig,"Units","normalized","Position",UI.iir2PanelPos, "Title","IIR2");

H.axRaw  = axes("Parent",H.pnlRaw,  "Units","normalized","Position",UI.axPos);
H.axMed  = axes("Parent",H.pnlMed,  "Units","normalized","Position",UI.axPos);
H.axIIR1 = axes("Parent",H.pnlIIR1, "Units","normalized","Position",UI.axPos);
H.axIIR2 = axes("Parent",H.pnlIIR2, "Units","normalized","Position",UI.axPos);

grid(H.axRaw,"on"); grid(H.axMed,"on"); grid(H.axIIR1,"on"); grid(H.axIIR2,"on");

%% ========================= CALLBACKS =========================
set(H.popMode,"Callback",@(~,~)updateAll());
set(H.popCH,  "Callback",@(~,~)updateAll());
set(H.edT0,   "Callback",@(~,~)updateAll());
set(H.edSpan, "Callback",@(~,~)updateAll());
set(H.edTw,   "Callback",@(~,~)updateAll());
set(H.edDtC,  "Callback",@(~,~)updateAll());
set(H.edMedN, "Callback",@(~,~)updateAll());
set(H.chkCMD, "Callback",@(~,~)updateAll());
set(H.chkOnOff,"Callback",@(~,~)updateAll());
set(H.edLag,  "Callback",@(~,~)updateAll());
set(H.chkMean,"Callback",@(~,~)updateAll());
set(H.chkVar, "Callback",@(~,~)updateAll());
set(H.slA1,   "Callback",@(~,~)updateAll());
set(H.slA2,   "Callback",@(~,~)updateAll());
set(H.slB2,   "Callback",@(~,~)updateAll());

updateAll();

%% ========================= NESTED =========================
    function updateAll()
        modeSel = get(H.popMode,"Value"); % 1..4
        chSel   = get(H.popCH,"Value");   % 1..3

        t0 = str2double(get(H.edT0,"String"));
        if ~(isfinite(t0)), t0 = 0; end

        tSpan = str2double(get(H.edSpan,"String"));
        if ~(isfinite(tSpan) && tSpan>0), tSpan = 10; end

        t1 = t0 + tSpan;

        if t0 < 0, t0 = 0; end
        if t1 > tEnd
            t1 = tEnd;
            tSpan = max(eps, t1 - t0);
        end
        if tSpan <= 0
            tSpan = 1;
            t1 = min(tEnd, t0 + tSpan);
        end

        set(H.edT0,"String",num2str(t0));
        set(H.edSpan,"String",num2str(tSpan));

        Tw = str2double(get(H.edTw,"String"));
        if ~(isfinite(Tw) && Tw>0), Tw = 0.01; end
        set(H.edTw,"String",num2str(Tw));

        dtC = str2double(get(H.edDtC,"String"));
        if ~(isfinite(dtC) && dtC>0), dtC = 0.01; end
        set(H.edDtC,"String",num2str(dtC));

        medN = round(str2double(get(H.edMedN,"String")));
        if ~isfinite(medN), medN = 11; end
        if medN < 1, medN = 1; end
        if mod(medN,2)==0, medN = medN + 1; end
        set(H.edMedN,"String",num2str(medN));

        lag = str2double(get(H.edLag,"String"));
        if ~(isfinite(lag)), lag = 0.04; end
        set(H.edLag,"String",num2str(lag));

        a1 = get(H.slA1,"Value");
        a2 = get(H.slA2,"Value");
        b2 = get(H.slB2,"Value");
        set(H.txtA1,"String",sprintf("%.3f",a1));
        set(H.txtA2,"String",sprintf("%.3f",a2));
        set(H.txtB2,"String",sprintf("%.3f",b2));

        showCMD   = (get(H.chkCMD,"Value") ~= 0) && hasCMD;
        showOnOff = (get(H.chkOnOff,"Value") ~= 0) && hasCMD;
        showMean  = (get(H.chkMean,"Value") ~= 0);
        showVar   = (get(H.chkVar,"Value")  ~= 0);

        % events (selected channel)
        t_evt = pickEvents(tA, tB, chSel);
        if numel(t_evt) < 3
            drawEmptyAll("NO EVENTS");
            H.cache = struct();
            return;
        end

        % device time -> plot time by manual lag
        t_evt_view = t_evt - lag;

        % clip to window
        inViewEvt = (t_evt_view >= t0) & (t_evt_view <= t1);
        t_evt_view = t_evt_view(inViewEvt);
        if numel(t_evt_view) < 3
            drawEmptyAll("NO EVENTS IN WINDOW");
            H.cache = struct();
            return;
        end

        % measures
        [tW,fW] = windowFreq(t_evt_view, Tw);

        dt = diff(t_evt_view); dt = max(dt, eps);
        tP = t_evt_view(2:end);
        fP = 1 ./ dt;

        [tC,fC] = windowFreq(t_evt_view, dtC);

        % filters
        [fW_med,fW_iir1,fW_iir2] = filterPack(fW, medN, a1, a2, b2);
        [fP_med,fP_iir1,fP_iir2] = filterPack(fP, medN, a1, a2, b2);
        [fC_med,fC_iir1,fC_iir2] = filterPack(fC, medN, a1, a2, b2);

        % command overlays in view window
        [tCMDv, uCMDv, onv] = deal([],[],[]);
        if hasCMD && (showCMD || showOnOff)
            inCMD = (tCMD >= t0) & (tCMD <= t1);
            tCMDv = tCMD(inCMD);
            uCMDv = uCMD(inCMD);
            if isempty(tCMDv), onv = [];
            else, onv = double(uCMDv > 0);
            end
        end

        % render
        chInfo = chStr(chSel);

        switch modeSel
            case 1 % accumulate
                render(H.axRaw,  "RAW",   chInfo, t0,t1, "acc raw",  tW, fW,      showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
                render(H.axMed,  "MEDIAN",chInfo, t0,t1, "acc med",  tW, fW_med,  showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
                render(H.axIIR1, "IIR1",  chInfo, t0,t1, "acc iir1", tW, fW_iir1, showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
                render(H.axIIR2, "IIR2",  chInfo, t0,t1, "acc iir2", tW, fW_iir2, showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);

            case 2 % period
                render(H.axRaw,  "RAW",   chInfo, t0,t1, "per raw",  tP, fP,      showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
                render(H.axMed,  "MEDIAN",chInfo, t0,t1, "per med",  tP, fP_med,  showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
                render(H.axIIR1, "IIR1",  chInfo, t0,t1, "per iir1", tP, fP_iir1, showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
                render(H.axIIR2, "IIR2",  chInfo, t0,t1, "per iir2", tP, fP_iir2, showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);

            case 3 % count_dt
                render(H.axRaw,  "RAW",   chInfo, t0,t1, "cnt raw",  tC, fC,      showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
                render(H.axMed,  "MEDIAN",chInfo, t0,t1, "cnt med",  tC, fC_med,  showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
                render(H.axIIR1, "IIR1",  chInfo, t0,t1, "cnt iir1", tC, fC_iir1, showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
                render(H.axIIR2, "IIR2",  chInfo, t0,t1, "cnt iir2", tC, fC_iir2, showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);

            case 4 % compare: acc vs per
                renderCompare(H.axRaw,  "RAW compare",  chInfo, t0,t1, "acc", tW, fW,       "per", tP, fP,       showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
                renderCompare(H.axMed,  "MED compare",  chInfo, t0,t1, "acc", tW, fW_med,   "per", tP, fP_med,   showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
                renderCompare(H.axIIR1, "IIR1 compare", chInfo, t0,t1, "acc", tW, fW_iir1,  "per", tP, fP_iir1,  showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
                renderCompare(H.axIIR2, "IIR2 compare", chInfo, t0,t1, "acc", tW, fW_iir2,  "per", tP, fP_iir2,  showCMD, tCMDv, uCMDv, showOnOff, tCMDv, onv, showMean, showVar);
        end

        % ---- CACHE (everything needed for PRINT is synchronized to current UI) ----
        H.cache = struct();
        H.cache.modeSel = modeSel;
        H.cache.modeStr = modeName(modeSel);
        H.cache.chStr   = chInfo;

        H.cache.t0 = t0; H.cache.t1 = t1;
        H.cache.Tw = Tw; H.cache.dtC = dtC; H.cache.medN = medN;
        H.cache.a1 = a1; H.cache.a2 = a2; H.cache.b2 = b2;
        H.cache.lag = lag;

        H.cache.hasCMD = hasCMD;
        H.cache.tCMD = tCMDv;
        H.cache.uCMD = uCMDv;
        H.cache.onCMD = onv;

        H.cache.t_evt_view = t_evt_view(:);

        % Store all four panel series (RAW/MED/IIR1/IIR2) for the chosen measure(s)
        % This allows PRINT to output full report regardless of which subplot is inspected.
        if modeSel == 4
            % compare: store acc/per for each panel level
            H.cache.series = struct();

            H.cache.series.RAW  = {packSeries("acc",tW,fW),         packSeries("per",tP,fP)};
            H.cache.series.MED  = {packSeries("acc",tW,fW_med),     packSeries("per",tP,fP_med)};
            H.cache.series.IIR1 = {packSeries("acc",tW,fW_iir1),    packSeries("per",tP,fP_iir1)};
            H.cache.series.IIR2 = {packSeries("acc",tW,fW_iir2),    packSeries("per",tP,fP_iir2)};
        else
            % single-mode: store one series for each panel level
            H.cache.series = struct();
            switch modeSel
                case 1 % accumulate
                    H.cache.series.RAW  = {packSeries("acc",tW,fW)};
                    H.cache.series.MED  = {packSeries("acc",tW,fW_med)};
                    H.cache.series.IIR1 = {packSeries("acc",tW,fW_iir1)};
                    H.cache.series.IIR2 = {packSeries("acc",tW,fW_iir2)};
                case 2 % period
                    H.cache.series.RAW  = {packSeries("per",tP,fP)};
                    H.cache.series.MED  = {packSeries("per",tP,fP_med)};
                    H.cache.series.IIR1 = {packSeries("per",tP,fP_iir1)};
                    H.cache.series.IIR2 = {packSeries("per",tP,fP_iir2)};
                case 3 % count_dt
                    H.cache.series.RAW  = {packSeries("cnt",tC,fC)};
                    H.cache.series.MED  = {packSeries("cnt",tC,fC_med)};
                    H.cache.series.IIR1 = {packSeries("cnt",tC,fC_iir1)};
                    H.cache.series.IIR2 = {packSeries("cnt",tC,fC_iir2)};
            end
        end
    end

    function printMetrics()
        if ~isfield(H,"cache") || ~isstruct(H.cache) || ~isfield(H.cache,"series")
            fprintf("[PRINT] No cached data available.\n");
            return;
        end

        C = H.cache;

        fprintf("\n==================== ENCODER REPORT (SYNC UI) ====================\n");
        fprintf("mode = %s | %s\n", C.modeStr, C.chStr);
        fprintf("window = [%.6f, %.6f] s | span = %.6f s\n", C.t0, C.t1, (C.t1-C.t0));
        fprintf("Tw = %.6g | dtC = %.6g | MedN = %d\n", C.Tw, C.dtC, C.medN);
        fprintf("IIR1 alpha = %.6g | IIR2 alpha = %.6g | IIR2 beta = %.6g\n", C.a1, C.a2, C.b2);
        fprintf("Lag (encoder shift) = %.6g s\n", C.lag);
        fprintf("Events in window = %d\n", numel(C.t_evt_view));

        % Basic event-rate diagnostics (purely from event timestamps)
        if numel(C.t_evt_view) >= 3
            dte = diff(C.t_evt_view);
            dte = dte(isfinite(dte) & dte>0);
            if ~isempty(dte)
                fprintf("Event dt: median = %.6g s | p10 = %.6g s | p90 = %.6g s\n", ...
                    median(dte), prctile(dte,10), prctile(dte,90));
            end
        end

        if C.hasCMD && ~isempty(C.tCMD)
            fprintf("CMD samples in window = %d | CMD mean = %.6g | CMD var = %.6g\n", ...
                numel(C.tCMD), mean(C.uCMD,"omitnan"), var(C.uCMD,"omitnan"));
            fprintf("ON fraction (u>0) = %.6g\n", mean(double(C.uCMD>0),"omitnan"));
        else
            fprintf("CMD data: not available (lag/corr metrics will be NaN).\n");
        end

        fprintf("-------------------------------------------------------------------\n");

        % Report for each panel level
        levels = {"RAW","MED","IIR1","IIR2"};
        for L = 1:numel(levels)
            lvl = levels{L};
            Sset = C.series.(lvl);

            fprintf("[%s]\n", lvl);

            % Per-series metrics
            for i = 1:numel(Sset)
                s = Sset{i};
                if isempty(s.t) || isempty(s.y)
                    fprintf("  - %s: empty\n", s.name);
                    continue;
                end

                % Resample to uniform grid for stable comparisons
                [tG, yG] = resampleToGrid(s.t, s.y, 0.002); % 2 ms grid (internal for metrics only)
                if isempty(tG)
                    fprintf("  - %s: insufficient points\n", s.name);
                    continue;
                end

                % Core metrics (mathematical)
                [var_r, J, lagOpt, corrMax] = metrics_math(tG, yG, C.tCMD, C.uCMD);

                % Additional descriptive stats
                mu  = mean(yG,"omitnan");
                va  = var(yG,"omitnan");

                % Outlier rate (robust MAD z-score on residual)
                outRate = outlierRateResidual(tG, yG);

                % Print
                fprintf("  - %s: N=%d | mean=%.6g | var=%.6g\n", s.name, numel(yG), mu, va);
                fprintf("        var_res=%.6g | jerkE=%.6g | outRate=%.6g\n", var_r, J, outRate);
                fprintf("        lag*=%.6g s | corr_max=%.6g\n", lagOpt, corrMax);

                % Store back for conclusion scoring
                Sset{i}.m_mu = mu;
                Sset{i}.m_var = va;
                Sset{i}.m_varres = var_r;
                Sset{i}.m_jerk = J;
                Sset{i}.m_out = outRate;
                Sset{i}.m_lag = lagOpt;
                Sset{i}.m_corr = corrMax;
                Sset{i}.tG = tG; Sset{i}.yG = yG;
            end

            % Pairwise compare if exactly two series (acc vs per)
            if numel(Sset) == 2 && isfield(Sset{1},"tG") && isfield(Sset{2},"tG") ...
                    && ~isempty(Sset{1}.tG) && ~isempty(Sset{2}.tG)

                [rmse12, rho12, overlap] = pairwiseCompare(Sset{1}.tG,Sset{1}.yG,Sset{2}.tG,Sset{2}.yG);
                fprintf("    Pair(%s vs %s): overlap=%.6g s | RMSE=%.6g | corr=%.6g\n", ...
                    Sset{1}.name, Sset{2}.name, overlap, rmse12, rho12);

                % Simple decision score (lower is better)
                % Score components:
                %   - smaller residual variance (var_res)
                %   - smaller jerk energy (jerkE)
                %   - smaller outlier rate
                %   - smaller absolute lag* (if CMD exists)
                %   - higher corr_max (if CMD exists) -> subtract
                w = struct("varres",1.0,"jerk",0.5,"out",0.8,"lag",0.5,"corr",0.8);

                [sBest, sLine] = chooseBestByScore(Sset, w, C.hasCMD && ~isempty(C.tCMD));
                fprintf("    Conclusion(%s): %s\n", lvl, sLine);
            end

            fprintf("-------------------------------------------------------------------\n");
        end

        fprintf("GLOBAL CONCLUSION:\n");
        fprintf("  If CMD exists, prioritize small |lag*| and high corr_max.\n");
        fprintf("  If CMD does not exist, prioritize low var_res + low jerkE + low outRate.\n");
        fprintf("===================================================================\n\n");
    end

    function s = packSeries(name, t, y)
        s = struct("name",string(name),"t",t(:),"y",y(:));
    end

    function [rmse12, rho12, overlap] = pairwiseCompare(t1,y1,t2,y2)
        tMin = max(min(t1), min(t2));
        tMax = min(max(t1), max(t2));
        overlap = max(0, tMax - tMin);
        if overlap <= 0
            rmse12 = NaN; rho12 = NaN; return;
        end
        dt = 0.002;
        tG = (tMin:dt:tMax).';
        y1g = interp1(t1,y1,tG,"linear","extrap");
        y2g = interp1(t2,y2,tG,"linear","extrap");
        rmse12 = sqrt(mean((y1g-y2g).^2,"omitnan"));
        rho12  = corr(y1g,y2g,"rows","complete");
    end

function [bestName, line, scores, detail] = chooseBestByScore(Sset, w, hasCmdNow)
% chooseBestByScore  Robust scoring (omit NaN) + metric normalization.
%
% INPUT
%   Sset      : cell array of struct, each must contain:
%               .name (string/char)
%               .m_varres, .m_jerk, .m_out, .m_lag, .m_corr
%   w         : struct with weights:
%               w.varres, w.jerk, w.out, w.lag, w.corr
%   hasCmdNow : logical, true if CMD exists and correlation/lag are valid targets
%
% OUTPUT
%   bestName  : name of best series
%   line      : human-readable summary
%   scores    : numeric score per series (lower is better)
%   detail    : struct with normalized components for debugging/logging
%
% Notes
%   - Uses 'omitnan' for min()
%   - Normalizes metrics with robust scaling (median + MAD)
%   - Uses log10 for jerk energy to avoid extreme dynamic range domination
%   - If hasCmdNow==false, ignores lag/corr terms by design
%
% Example weights:
%   w = struct("varres",1.0,"jerk",0.5,"out",0.8,"lag",0.5,"corr",0.8);

    n = numel(Sset);
    scores = nan(n,1);

    % --- Pull vectors (NaN-safe) ---
    name = strings(n,1);
    v_varres = nan(n,1);
    v_jerk   = nan(n,1);
    v_out    = nan(n,1);
    v_lag    = nan(n,1);
    v_corr   = nan(n,1);

    for i = 1:n
        if isfield(Sset{i},"name"), name(i) = string(Sset{i}.name); else, name(i) = "S"+i; end
        if isfield(Sset{i},"m_varres"), v_varres(i) = Sset{i}.m_varres; end
        if isfield(Sset{i},"m_jerk"),   v_jerk(i)   = Sset{i}.m_jerk;   end
        if isfield(Sset{i},"m_out"),    v_out(i)    = Sset{i}.m_out;    end
        if isfield(Sset{i},"m_lag"),    v_lag(i)    = Sset{i}.m_lag;    end
        if isfield(Sset{i},"m_corr"),   v_corr(i)   = Sset{i}.m_corr;   end
    end

    % --- Pre-transform ---
    % jerkE dynamic range is huge -> log compress for stable scoring
    v_jerk = log10(v_jerk + eps);

    % corr: larger is better, but we will subtract it in score after normalization
    % lag: smaller |lag| is better -> use abs(lag)
    v_lag = abs(v_lag);

    % --- Robust normalize each metric to comparable scale ---
    z_varres = robustZ(v_varres);
    z_jerk   = robustZ(v_jerk);
    z_out    = robustZ(v_out);

    if hasCmdNow
        z_lag  = robustZ(v_lag);
        z_corr = robustZ(v_corr); % larger is better
    else
        z_lag  = nan(n,1);
        z_corr = nan(n,1);
    end

    % --- Compute score (lower is better) ---
    % For corr: subtract normalized corr so higher corr reduces score.
    for i = 1:n
        if ~isfinite(z_varres(i)) || ~isfinite(z_jerk(i)) || ~isfinite(z_out(i))
            % If core metrics missing, cannot score
            scores(i) = NaN;
            continue;
        end

        s = 0;
        s = s + w.varres * z_varres(i);
        s = s + w.jerk   * z_jerk(i);
        s = s + w.out    * z_out(i);

        if hasCmdNow
            if isfinite(z_lag(i))
                s = s + w.lag * z_lag(i);
            end
            if isfinite(z_corr(i))
                s = s - w.corr * z_corr(i);
            end
        end

        scores(i) = s;
    end

    % --- Pick best (omit NaN) ---
    if all(isnan(scores))
        bestName = "";
        line = "best=NONE (all scores are NaN)";
        detail = makeDetail();
        return;
    end

    [~, idx] = min(scores, [], 'omitnan');
    bestName = name(idx);

    % --- Build verbose line ---
    parts = strings(0,1);
    for i = 1:n
        if ~isfinite(scores(i)), continue; end
        parts(end+1) = sprintf("%s score=%.6g", name(i), scores(i)); %#ok<AGROW>
    end
    line = sprintf("best=%s | %s", bestName, strjoin(parts," | "));

    % --- Debug detail for printing if needed ---
    detail = makeDetail();

    % ========== nested helpers ==========
    function z = robustZ(x)
        % Robust z-score using median & MAD (scaled to std approx)
        z = nan(size(x));
        ok = isfinite(x);
        if nnz(ok) < 2
            return;
        end
        xm = median(x(ok));
        madv = median(abs(x(ok) - xm));
        if madv <= 0
            % fallback to std
            xs = std(x(ok));
            if xs <= 0, return; end
            z(ok) = (x(ok) - xm) / xs;
        else
            z(ok) = 0.6745 * (x(ok) - xm) / madv;
        end
    end

    function d = makeDetail()
        d = struct();
        d.name = name;

        d.raw = struct();
        d.raw.varres = v_varres;
        d.raw.jerk_log10 = v_jerk;
        d.raw.out = v_out;
        d.raw.absLag = v_lag;
        d.raw.corr = v_corr;

        d.norm = struct();
        d.norm.varres = z_varres;
        d.norm.jerk_log10 = z_jerk;
        d.norm.out = z_out;
        d.norm.absLag = z_lag;
        d.norm.corr = z_corr;

        d.weights = w;
        d.hasCmdNow = hasCmdNow;
    end
end

    function name = modeName(modeSel)
        switch modeSel
            case 1, name = "accumulate";
            case 2, name = "period";
            case 3, name = "count_dt";
            case 4, name = "compare";
            otherwise, name = "unknown";
        end
    end

    function t_evt = pickEvents(tA_, tB_, sel)
        if sel==1
            t_evt = tA_(:);
        elseif sel==2
            t_evt = tB_(:);
        else
            t_evt = sort([tA_(:); tB_(:)]);
        end
    end

    function s = chStr(sel)
        if sel==1, s="CH=A";
        elseif sel==2, s="CH=B";
        else, s="CH=BOTH";
        end
    end

    function [f_med,f_iir1,f_iir2] = filterPack(f, medN_, a1_, a2_, b2_)
        if isempty(f)
            f_med=[]; f_iir1=[]; f_iir2=[]; return;
        end
        f_med  = medfilt1(f, medN_, "truncate");
        f_iir1 = iir1(f, a1_);
        f_iir2 = iir2_alphaBeta(f, a2_, b2_);
    end

    function [mu, va] = statsOf(y)
        y = y(:);
        y = y(isfinite(y));
        if isempty(y), mu = NaN; va = NaN;
        else, mu = mean(y); va = var(y);
        end
    end

    function ttl = makeTitle(tag, chInfo, name1, y, showMean_, showVar_)
        ttl = sprintf("%s | %s | %s", tag, chInfo, name1);
        if showMean_ || showVar_
            [mu, va] = statsOf(y);
            if showMean_ && showVar_
                ttl = sprintf("%s | mean=%.6g | var=%.6g", ttl, mu, va);
            elseif showMean_
                ttl = sprintf("%s | mean=%.6g", ttl, mu);
            else
                ttl = sprintf("%s | var=%.6g", ttl, va);
            end
        end
    end

    function render(ax, tag, chInfo, t0_, t1_, name1, tx, yx, ...
            showCMD_, tCMDv_, uCMDv_, showOnOff_, tON_, onv_, showMean_, showVar_)
        cla(ax,"reset"); hold(ax,"on"); grid(ax,"on");

        h = gobjects(0); lab = {};
        if ~isempty(tx)
            h(end+1)=plot(ax, tx, yx, "-", "LineWidth", 1); %#ok<AGROW>
            lab{end+1}=name1; %#ok<AGROW>
        end

        xlim(ax,[t0_ t1_]);
        xlabel(ax,"t (s)");
        ylabel(ax,"frequency (Hz)");

        if (showCMD_ && ~isempty(tCMDv_)) || (showOnOff_ && ~isempty(tON_))
            yyaxis(ax,"right");
            hold(ax,"on");
            if showCMD_ && ~isempty(tCMDv_)
                plot(ax, tCMDv_, uCMDv_, "-", "LineWidth", 1);
            end
            if showOnOff_ && ~isempty(tON_)
                stairs(ax, tON_, onv_, "-", "LineWidth", 1);
            end
            if showCMD_ && showOnOff_
                ylabel(ax,"CMD / ONOFF");
            elseif showCMD_
                ylabel(ax,"CMD");
            else
                ylabel(ax,"ONOFF (0/1)");
            end
            yyaxis(ax,"left");
        end

        title(ax, makeTitle(tag, chInfo, name1, yx, showMean_, showVar_));

        if ~isempty(h)
            legend(ax, h, lab, "Location","northeast");
        end
    end

    function renderCompare(ax, tag, chInfo, t0_, t1_, n1, t1v, y1v, n2, t2v, y2v, ...
            showCMD_, tCMDv_, uCMDv_, showOnOff_, tON_, onv_, showMean_, showVar_)
        cla(ax,"reset"); hold(ax,"on"); grid(ax,"on");

        h = gobjects(0); lab = {};
        if ~isempty(t1v)
            h(end+1)=plot(ax, t1v, y1v, "-", "LineWidth", 1); %#ok<AGROW>
            lab{end+1}=n1; %#ok<AGROW>
        end
        if ~isempty(t2v)
            h(end+1)=plot(ax, t2v, y2v, "-", "LineWidth", 1); %#ok<AGROW>
            lab{end+1}=n2; %#ok<AGROW>
        end

        xlim(ax,[t0_ t1_]);
        xlabel(ax,"t (s)");
        ylabel(ax,"frequency (Hz)");

        if (showCMD_ && ~isempty(tCMDv_)) || (showOnOff_ && ~isempty(tON_))
            yyaxis(ax,"right");
            hold(ax,"on");
            if showCMD_ && ~isempty(tCMDv_)
                plot(ax, tCMDv_, uCMDv_, "-", "LineWidth", 1);
            end
            if showOnOff_ && ~isempty(tON_)
                stairs(ax, tON_, onv_, "-", "LineWidth", 1);
            end
            if showCMD_ && showOnOff_
                ylabel(ax,"CMD / ONOFF");
            elseif showCMD_
                ylabel(ax,"CMD");
            else
                ylabel(ax,"ONOFF (0/1)");
            end
            yyaxis(ax,"left");
        end

        ttl = sprintf("%s | %s", tag, chInfo);
        if showMean_ || showVar_
            [m1,v1] = statsOf(y1v);
            [m2,v2] = statsOf(y2v);
            if showMean_ && showVar_
                ttl = sprintf("%s | %s(mean=%.6g,var=%.6g) | %s(mean=%.6g,var=%.6g)", ...
                    ttl, n1, m1, v1, n2, m2, v2);
            elseif showMean_
                ttl = sprintf("%s | %s(mean=%.6g) | %s(mean=%.6g)", ttl, n1, m1, n2, m2);
            else
                ttl = sprintf("%s | %s(var=%.6g) | %s(var=%.6g)", ttl, n1, v1, n2, v2);
            end
        end
        title(ax, ttl);

        if ~isempty(h)
            legend(ax, h, lab, "Location","northeast");
        end
    end

    function drawEmptyAll(msg)
        drawEmpty(H.axRaw,  "RAW", msg);
        drawEmpty(H.axMed,  "MEDIAN", msg);
        drawEmpty(H.axIIR1, "IIR1", msg);
        drawEmpty(H.axIIR2, "IIR2", msg);
    end

    function drawEmpty(ax, tag, msg)
        cla(ax,"reset"); grid(ax,"on");
        title(ax, sprintf("%s | %s", tag, msg));
        xlabel(ax,"t (s)");
        ylabel(ax,"frequency (Hz)");
    end

    function y = iir1(x, alpha)
        y = zeros(size(x)); y(1)=x(1);
        for k=2:numel(x)
            y(k)=y(k-1)+alpha*(x(k)-y(k-1));
        end
    end

    function y = iir2_alphaBeta(x, alpha, beta)
        y = zeros(size(x)); y(1)=x(1);
        if numel(x)>=2
            y(2)=(1-alpha)*y(1)+alpha*x(2);
        end
        for k=3:numel(x)
            y(k)=(1-alpha)*y(k-1)+alpha*x(k)+beta*(y(k-1)-y(k-2));
        end
    end

    function [t_mid, f_win] = windowFreq(t_evt, Tw_)
        t_evt = t_evt(:);
        edges = t_evt(1):Tw_:(t_evt(end)+Tw_);
        if numel(edges)<2
            t_mid=t_evt(1); f_win=0; return;
        end
        c = histcounts(t_evt, edges);
        t_mid = edges(1:end-1) + Tw_/2;
        f_win = c(:)'/Tw_;
    end

    % ===== METRICS HELPERS (MATHEMATICAL) =====
    function [tG, yG] = resampleToGrid(tIn, yIn, dtG)
        tIn = tIn(:); yIn = yIn(:);
        ok = isfinite(tIn) & isfinite(yIn);
        tIn = tIn(ok); yIn = yIn(ok);
        if numel(tIn) < 3
            tG = []; yG = []; return;
        end
        tMin = min(tIn); tMax = max(tIn);
        if tMax <= tMin
            tG = []; yG = []; return;
        end
        tG = (tMin:dtG:tMax).';
        yG = interp1(tIn, yIn, tG, "linear", "extrap");
    end

    function [var_r, J, lagOpt, corrMax] = metrics_math(t, y, tcmd, ucmd)
        % Residual variance via moving-mean baseline (0.2 s)
        dt = median(diff(t));
        W = max(5, round(0.2/dt));
        yhat = movmean(y, W, "omitnan");
        r = y - yhat;
        var_r = var(r, "omitnan");

        % Jerk energy (2nd derivative energy)
        if numel(y) >= 3
            d2y = diff(y,2) / (dt^2);
            J = mean(d2y.^2, "omitnan");
        else
            J = NaN;
        end

        % Lag/corr vs command (optional)
        lagOpt = NaN; corrMax = NaN;
        if ~isempty(tcmd) && ~isempty(ucmd) && numel(tcmd)>=2
            u = interp1(tcmd, ucmd, t, "previous", "extrap");
            u = u - mean(u, "omitnan");
            yc = y - mean(y, "omitnan");
            [c,lags] = xcorr(yc, u, "coeff");
            [corrMax, idx] = max(c);
            lagOpt = lags(idx) * dt;
        end
    end

    function outRate = outlierRateResidual(t, y)
        dt = median(diff(t));
        W = max(5, round(0.2/dt));
        yhat = movmean(y, W, "omitnan");
        r = y - yhat;
        r = r(isfinite(r));
        if numel(r) < 10
            outRate = NaN; return;
        end
        medr = median(r);
        madv = median(abs(r - medr));
        if madv <= 0
            outRate = 0; return;
        end
        z = 0.6745 * (r - medr) / madv;   % robust z-score
        outRate = mean(abs(z) > 3.5);
    end
end
