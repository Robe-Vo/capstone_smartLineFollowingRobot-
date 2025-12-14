function encoder_viewer_ui_m()
% encoder_viewer_ui_m.m
% Requested changes:
% - Popups: 5x smaller
% - Numeric edit boxes: 2x smaller
% - Push ALL objects left (so IIR2 beta is visible)
% - IIR2 alpha slider length = IIR1 alpha slider length
% - All 3 sliders range: 0..1

%% ========================= USER-EDITABLE LAYOUT =========================
UI = struct();

% Figure
UI.figUnits = "normalized";
UI.figPos   = [0.04 0.06 0.92 0.86];

% Control panel
UI.ctrlPanelPos = [0.01 0.915 0.98 0.08];

% Rows inside control panel
UI.row1Y = 0.58;
UI.row2Y = 0.15;
UI.ctrlH = 0.32;

% Base widths (normalized to control panel)
UI.wLbl   = 0.045;     % label width (slightly smaller)
UI.wPop   = 0.040;     % popup width (5x smaller vs ~0.20)
UI.wNum   = 0.0225;    % numeric edit width (2x smaller vs ~0.045)
UI.wGap   = 0.008;     % tighter gaps to push left

% Sliders (thicker)
UI.sliderHeight = 0.26;  % thicker (increase if needed 0.28..0.32)
UI.wVal         = 0.030; % value readout width (smaller)

% IIR block layout (make compact so beta not hidden)
UI.iirLabelW  = 0.040;   % "IIRx α/β" label width
UI.iirSlW     = 0.110;   % slider width (IIR1 and IIR2 alpha MUST match)
UI.iirBetaSlW = 0.085;   % beta slider width (compact)
UI.iirGap     = 0.010;   % spacing within IIR block
UI.xIIR       = 0.36;    % MOVE LEFT hard so beta is visible (adjust if needed)

% Plot panels
UI.rawPanelPos  = [0.01 0.49 0.49 0.41];
UI.medPanelPos  = [0.50 0.49 0.49 0.41];
UI.iir1PanelPos = [0.01 0.03 0.49 0.41];
UI.iir2PanelPos = [0.50 0.03 0.49 0.41];

% Axes inside panel
UI.axPos = [0.08 0.12 0.90 0.82];

%% ========================= LOAD DATA =========================
S = load("encoderSignalsPerSteps.mat");
assert(isfield(S,"encoderSignalsPerSteps"), "Missing encoderSignalsPerSteps in encoderSignalsPerSteps.mat");
steps = S.encoderSignalsPerSteps;
assert(~isempty(steps), "encoderSignalsPerSteps is empty.");
uniqPwm = unique(uint8([steps.pwm]), "stable");

%% ========================= FIGURE + UI =========================
H = struct();
H.fig = figure("Name","Encoder Viewer (RAW / MEDIAN / IIR1 / IIR2)", ...
    "NumberTitle","off", "Units",UI.figUnits, "Position",UI.figPos);

H.pnl = uipanel(H.fig, "Units","normalized", "Position",UI.ctrlPanelPos, "Title","Tuning");

% Convenience
x = 0.005; y1 = UI.row1Y; y2 = UI.row2Y; h = UI.ctrlH; gap = UI.wGap;

% -------- Column 1: mode / PWM --------
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y1 UI.wLbl h], ...
    "String","mode","HorizontalAlignment","left");
H.popMode = uicontrol(H.pnl,"Style","popupmenu","Units","normalized","Position",[x+UI.wLbl y1 UI.wPop h], ...
    "String",{"accumulate","period","compare"});

uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y2 UI.wLbl h], ...
    "String","PWM","HorizontalAlignment","left");
H.popPwm = uicontrol(H.pnl,"Style","popupmenu","Units","normalized","Position",[x+UI.wLbl y2 UI.wPop h], ...
    "String",cellstr(string(uniqPwm)));

x = x + UI.wLbl + UI.wPop + gap;

% -------- Column 2: CHacc / CHper --------
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y1 UI.wLbl h], ...
    "String","CHacc","HorizontalAlignment","left");
H.popChAcc = uicontrol(H.pnl,"Style","popupmenu","Units","normalized","Position",[x+UI.wLbl y1 UI.wPop h], ...
    "String",{"A","B","BOTH"});

uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y2 UI.wLbl h], ...
    "String","CHper","HorizontalAlignment","left");
H.popChPer = uicontrol(H.pnl,"Style","popupmenu","Units","normalized","Position",[x+UI.wLbl y2 UI.wPop h], ...
    "String",{"A","B","BOTH"});

x = x + UI.wLbl + UI.wPop + gap;

% -------- Column 3: tStart / tEnd --------
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y1 UI.wLbl h], ...
    "String","tStart","HorizontalAlignment","left");
H.edT0 = uicontrol(H.pnl,"Style","edit","Units","normalized","Position",[x+UI.wLbl y1 UI.wNum h], ...
    "String","0");

uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y2 UI.wLbl h], ...
    "String","tEnd","HorizontalAlignment","left");
H.edT1 = uicontrol(H.pnl,"Style","edit","Units","normalized","Position",[x+UI.wLbl y2 UI.wNum h], ...
    "String","6");

x = x + UI.wLbl + UI.wNum + gap;

% -------- Column 4: Tw / MedN --------
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y1 UI.wLbl h], ...
    "String","Tw","HorizontalAlignment","left");
H.edTw = uicontrol(H.pnl,"Style","edit","Units","normalized","Position",[x+UI.wLbl y1 UI.wNum h], ...
    "String","0.01");

uicontrol(H.pnl,"Style","text","Units","normalized","Position",[x y2 UI.wLbl h], ...
    "String","MedN","HorizontalAlignment","left");
H.edMedN = uicontrol(H.pnl,"Style","edit","Units","normalized","Position",[x+UI.wLbl y2 UI.wNum h], ...
    "String","11");

% -------- IIR block (forced LEFT + compact, beta visible) --------
xIIR = UI.xIIR;

% Row1: IIR2 alpha (slider width = IIR1 slider width)
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[xIIR y1 UI.iirLabelW h], ...
    "String","IIR2 α","HorizontalAlignment","left");
H.slA2 = uicontrol(H.pnl,"Style","slider","Units","normalized", ...
    "Position",[xIIR+UI.iirLabelW y1+0.05 UI.iirSlW UI.sliderHeight], ...
    "Min",0,"Max",1,"Value",0.05);
H.txtA2 = uicontrol(H.pnl,"Style","text","Units","normalized", ...
    "Position",[xIIR+UI.iirLabelW+UI.iirSlW+0.004 y1 UI.wVal h], ...
    "String","0.050","HorizontalAlignment","left");

% Row2: IIR1 alpha
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[xIIR y2 UI.iirLabelW h], ...
    "String","IIR1 α","HorizontalAlignment","left");
H.slA1 = uicontrol(H.pnl,"Style","slider","Units","normalized", ...
    "Position",[xIIR+UI.iirLabelW y2+0.05 UI.iirSlW UI.sliderHeight], ...
    "Min",0,"Max",1,"Value",0.05);
H.txtA1 = uicontrol(H.pnl,"Style","text","Units","normalized", ...
    "Position",[xIIR+UI.iirLabelW+UI.iirSlW+0.004 y2 UI.wVal h], ...
    "String","0.050","HorizontalAlignment","left");

% Row2: IIR2 beta (to the right of IIR1 alpha) — ensure visible
xBeta = xIIR + UI.iirLabelW + UI.iirSlW + 0.004 + UI.wVal + UI.iirGap;
uicontrol(H.pnl,"Style","text","Units","normalized","Position",[xBeta y2 UI.iirLabelW h], ...
    "String","IIR2 β","HorizontalAlignment","left");
H.slB2 = uicontrol(H.pnl,"Style","slider","Units","normalized", ...
    "Position",[xBeta+UI.iirLabelW y2+0.05 UI.iirBetaSlW UI.sliderHeight], ...
    "Min",0,"Max",1,"Value",0.0);
H.txtB2 = uicontrol(H.pnl,"Style","text","Units","normalized", ...
    "Position",[xBeta+UI.iirLabelW+UI.iirBetaSlW+0.004 y2 UI.wVal h], ...
    "String","0.000","HorizontalAlignment","left");

%% ========================= PLOTS =========================
H.pnlRaw  = uipanel(H.fig,"Units","normalized","Position",UI.rawPanelPos, "Title","RAW");
H.pnlMed  = uipanel(H.fig,"Units","normalized","Position",UI.medPanelPos, "Title","MEDIAN");
H.pnlIIR1 = uipanel(H.fig,"Units","normalized","Position",UI.iir1PanelPos,"Title","IIR1");
H.pnlIIR2 = uipanel(H.fig,"Units","normalized","Position",UI.iir2PanelPos,"Title","IIR2");

H.axRaw  = axes("Parent",H.pnlRaw, "Units","normalized","Position",UI.axPos);
H.axMed  = axes("Parent",H.pnlMed, "Units","normalized","Position",UI.axPos);
H.axIIR1 = axes("Parent",H.pnlIIR1,"Units","normalized","Position",UI.axPos);
H.axIIR2 = axes("Parent",H.pnlIIR2,"Units","normalized","Position",UI.axPos);

grid(H.axRaw,"on"); grid(H.axMed,"on"); grid(H.axIIR1,"on"); grid(H.axIIR2,"on");

%% ========================= CALLBACKS =========================
set(H.popPwm,  "Callback",@(~,~)updateAll());
set(H.popMode, "Callback",@(~,~)updateAll());
set(H.popChAcc,"Callback",@(~,~)updateAll());
set(H.popChPer,"Callback",@(~,~)updateAll());
set(H.edT0,    "Callback",@(~,~)updateAll());
set(H.edT1,    "Callback",@(~,~)updateAll());
set(H.edTw,    "Callback",@(~,~)updateAll());
set(H.edMedN,  "Callback",@(~,~)updateAll());
set(H.slA1,    "Callback",@(~,~)updateAll());
set(H.slA2,    "Callback",@(~,~)updateAll());
set(H.slB2,    "Callback",@(~,~)updateAll());

updateAll();

%% ========================= NESTED =========================
    function updateAll()
        pwmSel   = uniqPwm(get(H.popPwm,"Value"));
        modeSel  = get(H.popMode,"Value"); % 1..3
        chAccSel = get(H.popChAcc,"Value"); % 1..3
        chPerSel = get(H.popChPer,"Value"); % 1..3

        t0 = str2double(get(H.edT0,"String"));
        t1 = str2double(get(H.edT1,"String"));
        if ~(isfinite(t0)), t0 = 0; end
        if ~(isfinite(t1)), t1 = 6; end
        if t1 <= t0, t1 = t0 + 1; end
        set(H.edT0,"String",num2str(t0));
        set(H.edT1,"String",num2str(t1));

        Tw = str2double(get(H.edTw,"String"));
        if ~(isfinite(Tw) && Tw>0), Tw = 0.01; end
        set(H.edTw,"String",num2str(Tw));

        medN = round(str2double(get(H.edMedN,"String")));
        if ~isfinite(medN), medN = 11; end
        if medN < 1, medN = 1; end
        if mod(medN,2)==0, medN = medN + 1; end
        set(H.edMedN,"String",num2str(medN));

        a1 = get(H.slA1,"Value");
        a2 = get(H.slA2,"Value");
        b2 = get(H.slB2,"Value");
        set(H.txtA1,"String",sprintf("%.3f",a1));
        set(H.txtA2,"String",sprintf("%.3f",a2));
        set(H.txtB2,"String",sprintf("%.3f",b2));

        idx = find(uint8([steps.pwm]) == uint8(pwmSel), 1, "first");
        if isempty(idx)
            drawEmptyAll("NO DATA");
            return;
        end

        tAll = double(steps(idx).t_us(:)) * 1e-6;
        chId = double(steps(idx).ch(:));
        tA = tAll(chId==0);
        tB = tAll(chId==1);

        t_evt_acc = pickEvents(tA,tB,chAccSel);
        t_evt_per = pickEvents(tA,tB,chPerSel);

        [tW,fW] = deal([],[]);
        if numel(t_evt_acc) >= 3
            [tW,fW] = windowFreq(t_evt_acc, Tw);
        end

        [tP,fP] = deal([],[]);
        if numel(t_evt_per) >= 3
            dt = diff(t_evt_per); dt = max(dt, eps);
            tP = t_evt_per(2:end);
            fP = 1./dt;
        end

        [fW_med,fW_iir1,fW_iir2] = deal([],[],[]);
        if ~isempty(fW)
            fW_med  = medfilt1(fW, medN, "truncate");
            fW_iir1 = iir1(fW, a1);
            fW_iir2 = iir2_alphaBeta(fW, a2, b2);
        end

        [fP_med,fP_iir1,fP_iir2] = deal([],[],[]);
        if ~isempty(fP)
            fP_med  = medfilt1(fP, medN, "truncate");
            fP_iir1 = iir1(fP, a1);
            fP_iir2 = iir2_alphaBeta(fP, a2, b2);
        end

        switch modeSel
            case 1 % accumulate
                render(H.axRaw,  "RAW",  pwmSel, chStr(chAccSel,"CHacc"), t0,t1, ...
                    "acc raw", tW, fW, "",[],[], "",[],[]);
                render(H.axMed,  "MEDIAN",pwmSel, chStr(chAccSel,"CHacc"), t0,t1, ...
                    "acc raw", tW, fW, "acc proc", tW, fW_med, "",[],[]);
                render(H.axIIR1, "IIR1", pwmSel, chStr(chAccSel,"CHacc"), t0,t1, ...
                    "acc raw", tW, fW, "acc proc", tW, fW_iir1, "",[],[]);
                render(H.axIIR2, "IIR2", pwmSel, chStr(chAccSel,"CHacc"), t0,t1, ...
                    "acc raw", tW, fW, "acc proc", tW, fW_iir2, "",[],[]);

            case 2 % period
                render(H.axRaw,  "RAW",  pwmSel, chStr(chPerSel,"CHper"), t0,t1, ...
                    "per raw", tP, fP, "",[],[], "",[],[]);
                render(H.axMed,  "MEDIAN",pwmSel, chStr(chPerSel,"CHper"), t0,t1, ...
                    "per raw", tP, fP, "per proc", tP, fP_med, "",[],[]);
                render(H.axIIR1, "IIR1", pwmSel, chStr(chPerSel,"CHper"), t0,t1, ...
                    "per raw", tP, fP, "per proc", tP, fP_iir1, "",[],[]);
                render(H.axIIR2, "IIR2", pwmSel, chStr(chPerSel,"CHper"), t0,t1, ...
                    "per raw", tP, fP, "per proc", tP, fP_iir2, "",[],[]);

            case 3 % compare
                render(H.axRaw, "RAW (compare RAW)", pwmSel, ...
                    sprintf("CHacc=%s | CHper=%s", chOnly(chAccSel), chOnly(chPerSel)), t0,t1, ...
                    "acc raw", tW, fW, "per raw", tP, fP, "",[],[]);
                render(H.axMed, "MEDIAN (compare PROC)", pwmSel, ...
                    sprintf("CHacc=%s | CHper=%s", chOnly(chAccSel), chOnly(chPerSel)), t0,t1, ...
                    "acc proc", tW, fW_med, "per proc", tP, fP_med, "",[],[]);
                render(H.axIIR1,"IIR1 (compare PROC)", pwmSel, ...
                    sprintf("CHacc=%s | CHper=%s", chOnly(chAccSel), chOnly(chPerSel)), t0,t1, ...
                    "acc proc", tW, fW_iir1, "per proc", tP, fP_iir1, "",[],[]);
                render(H.axIIR2,"IIR2 (compare PROC)", pwmSel, ...
                    sprintf("CHacc=%s | CHper=%s", chOnly(chAccSel), chOnly(chPerSel)), t0,t1, ...
                    "acc proc", tW, fW_iir2, "per proc", tP, fP_iir2, "",[],[]);
        end
    end

    function t_evt = pickEvents(tA,tB,chSel)
        if chSel==1
            t_evt = tA(:);
        elseif chSel==2
            t_evt = tB(:);
        else
            t_evt = sort([tA(:); tB(:)]);
        end
    end

    function render(ax, tag, pwm, chInfo, t0,t1, ...
            name1, t1v, y1v, name2, t2v, y2v, name3, t3v, y3v)
        cla(ax,"reset"); hold(ax,"on"); grid(ax,"on");
        h = gobjects(0); lab = {};
        if ~isempty(t1v), h(end+1)=plot(ax,t1v,y1v,"-","LineWidth",1); lab{end+1}=name1; end %#ok<AGROW>
        if ~isempty(t2v), h(end+1)=plot(ax,t2v,y2v,"-","LineWidth",1); lab{end+1}=name2; end %#ok<AGROW>
        if ~isempty(t3v), h(end+1)=plot(ax,t3v,y3v,"-","LineWidth",1); lab{end+1}=name3; end %#ok<AGROW>
        xlim(ax,[t0 t1]);
        xlabel(ax,"t (s)"); ylabel(ax,"frequency (Hz)");
        title(ax, sprintf("%s | PWM=%d | %s", tag, pwm, chInfo));
        if ~isempty(h), legend(ax,h,lab,"Location","northeast"); end
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
        xlabel(ax,"t (s)"); ylabel(ax,"frequency (Hz)");
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

    function [t_mid, f_win] = windowFreq(t_evt, Tw)
        t_evt = t_evt(:);
        edges = t_evt(1):Tw:(t_evt(end)+Tw);
        if numel(edges)<2
            t_mid=t_evt(1); f_win=0; return;
        end
        c = histcounts(t_evt, edges);
        t_mid = edges(1:end-1) + Tw/2;
        f_win = c(:)'/Tw;
    end

    function s = chOnly(sel)
        if sel==1, s="A"; elseif sel==2, s="B"; else, s="BOTH"; end
    end

    function s = chStr(sel, prefix)
        s = sprintf("%s=%s", prefix, chOnly(sel));
    end
end
