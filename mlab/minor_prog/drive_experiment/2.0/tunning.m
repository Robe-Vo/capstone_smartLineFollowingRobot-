function tunning_ui_final()
% tunning_ui_final.m
% Requirement: Put ALL positions/sizes of the UI objects (the top bar in screenshot)
% into ONE place at the top (struct L.ui.*), so you can adjust easily.

S = load("encoderSignalsPerSteps.mat");
assert(isfield(S,"encoderSignalsPerSteps"), "Missing encoderSignalsPerSteps in encoderSignalsPerSteps.mat");
steps = S.encoderSignalsPerSteps;
assert(~isempty(steps), "encoderSignalsPerSteps is empty.");
uniqPwm = unique(uint8([steps.pwm]), "stable");

% ======================================================================
% ===================== ALL POSITIONS / SIZES HERE ======================
% ======================================================================
L = struct();

% ---- Figure ----
L.fig.pos = [0.05 0.05 0.92 0.86];

% ---- UI panel (top bar container) ----
L.ui.panel.pos = [0.005 0.90 0.99 0.095];
L.ui.panel.title = "Tuning";

% ---- Row geometry in panel ----
L.ui.row1.yCtl = 0.55;  L.ui.row1.yLbl = 0.52;
L.ui.row2.yCtl = 0.02;  L.ui.row2.yLbl = 0.5;
L.ui.rowH      = 0.40;

% ---- Column anchors (left to right) ----
L.ui.x0   = 0.005;     % left start
L.ui.gap  = 0.0005;     % gap between blocks

% ---- Common label widths ----
L.ui.lblPWM.w   = 0.035;
L.ui.lblCH.w    = 0.025;
L.ui.lblMode.w  = 0.05;
L.ui.lblT0.w    = 0.05;
L.ui.lblT1.w    = 0.05;
L.ui.lblTw.w    = 0.02;

% ---- Control widths ----
L.ui.popPWM.w   = 0.07;
L.ui.popCH.w    = 0.04;
L.ui.popMode.w  = 0.07;

L.ui.edT0.w     = 0.04;
L.ui.edT1.w     = 0.04;
L.ui.edTw.w     = 0.04;

% ---- IIR block anchors (SHIFT RIGHT HERE) ----
% IIR2 alpha row1; IIR1 alpha row2 left of IIR2 beta; IIR2 beta row2
L.ui.iir.sliderW = 0.15;
L.ui.iir.txtW    = L.ui.iir.sliderW;
L.ui.iir.hSlider = 0.4;
L.ui.iir.hTxt    = 0.2;

L.ui.iir.xIIR1 = 0.62;   % row2: IIR1 alpha anchor
L.ui.iir.xIIR2 = 0.80;   % row1/row2: IIR2 alpha/beta anchor

% ---- Fine offsets (if you want to shift whole utility block etc.) ----
L.ui.util.x = 0.30;       % mode, tStart, tEnd, Tw block anchor

% ---- Plot panels + axes ----
L.pan.raw  = [0.01 0.49 0.49 0.39];
L.pan.med  = [0.50 0.49 0.49 0.39];
L.pan.iir1 = [0.01 0.03 0.49 0.39];
L.pan.iir2 = [0.50 0.03 0.49 0.39];
L.ax.pos   = [0.07 0.17 0.92 0.78];

% ======================================================================
% ============================ BUILD UI ================================
% ======================================================================
app.fig = figure("Name","Encoder Viewer (RAW / MEDIAN / IIR1 / IIR2)", ...
  "NumberTitle","off","Units","normalized","Position",L.fig.pos);

app.pnlUI = uipanel(app.fig, "Units","normalized", "Position",L.ui.panel.pos, "Title",L.ui.panel.title);

% ---- Left block: PWM / CH ----
x = L.ui.x0;

app.lblPWM = uicontrol(app.pnlUI,"Style","text","Units","normalized", ...
  "Position",[x L.ui.row1.yLbl L.ui.lblPWM.w L.ui.rowH], ...
  "String","PWM","HorizontalAlignment","left");
x = x + L.ui.lblPWM.w;

app.popPwm = uicontrol(app.pnlUI,"Style","popupmenu","Units","normalized", ...
  "Position",[x L.ui.row1.yCtl L.ui.popPWM.w L.ui.rowH], ...
  "String",cellstr(string(uniqPwm)));
x = x + L.ui.popPWM.w + L.ui.gap;

app.lblCH = uicontrol(app.pnlUI,"Style","text","Units","normalized", ...
  "Position",[x L.ui.row1.yLbl L.ui.lblCH.w L.ui.rowH], ...
  "String","CH","HorizontalAlignment","left");
x = x + L.ui.lblCH.w;

app.popCh = uicontrol(app.pnlUI,"Style","popupmenu","Units","normalized", ...
  "Position",[x L.ui.row1.yCtl L.ui.popCH.w L.ui.rowH], ...
  "String",{"A","B","BOTH"});

% ---- Utility block: mode / tStart / tEnd / Tw ----
xu = L.ui.util.x;

app.lblMode = uicontrol(app.pnlUI,"Style","text","Units","normalized", ...
  "Position",[xu L.ui.row1.yLbl L.ui.lblMode.w L.ui.rowH], ...
  "String","mode","HorizontalAlignment","left");

app.popMode = uicontrol(app.pnlUI,"Style","popupmenu","Units","normalized", ...
  "Position",[xu+L.ui.lblMode.w L.ui.row1.yCtl L.ui.popMode.w L.ui.rowH], ...
  "String",{"accumulate","period","compare"});

x2 = xu + L.ui.lblMode.w + L.ui.popMode.w + L.ui.gap;

app.lblT0 = uicontrol(app.pnlUI,"Style","text","Units","normalized", ...
  "Position",[x2 L.ui.row1.yLbl L.ui.lblT0.w L.ui.rowH], ...
  "String","tStart","HorizontalAlignment","left");
app.edT0 = uicontrol(app.pnlUI,"Style","edit","Units","normalized", ...
  "Position",[x2+L.ui.lblT0.w L.ui.row1.yCtl L.ui.edT0.w L.ui.rowH], ...
  "String","0");

x3 = x2 + L.ui.lblT0.w + L.ui.edT0.w + L.ui.gap;

app.lblT1 = uicontrol(app.pnlUI,"Style","text","Units","normalized", ...
  "Position",[x3 L.ui.row1.yLbl L.ui.lblT1.w L.ui.rowH], ...
  "String","tEnd","HorizontalAlignment","left");
app.edT1 = uicontrol(app.pnlUI,"Style","edit","Units","normalized", ...
  "Position",[x3+L.ui.lblT1.w L.ui.row1.yCtl L.ui.edT1.w L.ui.rowH], ...
  "String","6");

x4 = x3 + L.ui.lblT1.w + L.ui.edT1.w + L.ui.gap;

app.lblTw = uicontrol(app.pnlUI,"Style","text","Units","normalized", ...
  "Position",[x4 L.ui.row1.yLbl L.ui.lblTw.w L.ui.rowH], ...
  "String","Tw","HorizontalAlignment","left");
app.edTw = uicontrol(app.pnlUI,"Style","edit","Units","normalized", ...
  "Position",[x4+L.ui.lblTw.w L.ui.row1.yCtl L.ui.edTw.w L.ui.rowH], ...
  "String","0.01");

% ---- IIR2 alpha (row1) ----
xI2 = L.ui.iir.xIIR2;
app.lblA2 = uicontrol(app.pnlUI,"Style","text","Units","normalized", ...
  "Position",[xI2 L.ui.row1.yLbl 0.05 L.ui.rowH], ...
  "String","IIR2 α","HorizontalAlignment","left");

app.slA2 = uicontrol(app.pnlUI,"Style","slider","Units","normalized", ...
  "Position",[xI2+0.05 L.ui.row1.yCtl L.ui.iir.sliderW L.ui.iir.hSlider], ...
  "Min",0.001,"Max",0.5,"Value",0.05);

app.txtA2 = uicontrol(app.pnlUI,"Style","text","Units","normalized", ...
  "Position",[xI2+0.05 L.ui.row1.yLbl L.ui.iir.txtW L.ui.iir.hTxt], ...
  "String","0.050","HorizontalAlignment","center");

% ---- IIR1 alpha (row2) ----
xI1 = L.ui.iir.xIIR1;
app.lblA1 = uicontrol(app.pnlUI,"Style","text","Units","normalized", ...
  "Position",[xI1 L.ui.row2.yLbl 0.1 L.ui.rowH], ...
  "String","IIR1 α","HorizontalAlignment","left");

app.slA1 = uicontrol(app.pnlUI,"Style","slider","Units","normalized", ...
  "Position",[xI1+0.05 L.ui.row2.yCtl L.ui.iir.sliderW L.ui.iir.hSlider], ...
  "Min",0.001,"Max",0.5,"Value",0.05);

app.txtA1 = uicontrol(app.pnlUI,"Style","text","Units","normalized", ...
  "Position",[xI1+0.05 L.ui.row2.yLbl L.ui.iir.txtW L.ui.iir.hTxt], ...
  "String","0.050","HorizontalAlignment","center");

% ---- IIR2 beta (row2) ----
app.lblB2 = uicontrol(app.pnlUI,"Style","text","Units","normalized", ...
  "Position",[xI2 L.ui.row2.yLbl 0.05 L.ui.rowH], ...
  "String","IIR2 β","HorizontalAlignment","left");

app.slB2 = uicontrol(app.pnlUI,"Style","slider","Units","normalized", ...
  "Position",[xI2+0.05 L.ui.row2.yCtl L.ui.iir.sliderW L.ui.iir.hSlider], ...
  "Min",0.0,"Max",0.95,"Value",0.0);

app.txtB2 = uicontrol(app.pnlUI,"Style","text","Units","normalized", ...
  "Position",[xI2+0.05 L.ui.row2.yLbl L.ui.iir.txtW L.ui.iir.hTxt], ...
  "String","0.000","HorizontalAlignment","center");

% ---- Panels / axes ----
app.pnlRaw  = uipanel(app.fig,"Units","normalized","Position",L.pan.raw, "Title","RAW");
app.pnlMed  = uipanel(app.fig,"Units","normalized","Position",L.pan.med, "Title","MEDIAN");
app.pnlIIR1 = uipanel(app.fig,"Units","normalized","Position",L.pan.iir1,"Title","IIR1");
app.pnlIIR2 = uipanel(app.fig,"Units","normalized","Position",L.pan.iir2,"Title","IIR2");

app.axRaw  = axes("Parent",app.pnlRaw, "Units","normalized","Position",L.ax.pos);
app.axMed  = axes("Parent",app.pnlMed, "Units","normalized","Position",L.ax.pos);
app.axIIR1 = axes("Parent",app.pnlIIR1,"Units","normalized","Position",L.ax.pos);
app.axIIR2 = axes("Parent",app.pnlIIR2,"Units","normalized","Position",L.ax.pos);

% ---- Callbacks ----
set(app.popPwm,"Callback",@(~,~)updateAll());
set(app.popCh,"Callback",@(~,~)updateAll());
set(app.popMode,"Callback",@(~,~)updateAll());
set(app.edT0,"Callback",@(~,~)updateAll());
set(app.edT1,"Callback",@(~,~)updateAll());
set(app.edTw,"Callback",@(~,~)updateAll());
set(app.slA1,"Callback",@(~,~)updateAll());
set(app.slA2,"Callback",@(~,~)updateAll());
set(app.slB2,"Callback",@(~,~)updateAll());

updateAll();

% ======================================================================
function updateAll()
  pwmSel  = uniqPwm(get(app.popPwm,"Value"));
  chMode  = get(app.popCh,"Value");
  modeSel = get(app.popMode,"Value"); % 1 accumulate, 2 period, 3 compare

  t0 = str2double(get(app.edT0,"String"));
  t1 = str2double(get(app.edT1,"String"));
  if ~isfinite(t0), t0 = 0; end
  if ~isfinite(t1), t1 = 6; end
  if t1 <= t0, t1 = t0 + 1; end
  set(app.edT0,"String",num2str(t0));
  set(app.edT1,"String",num2str(t1));

  Tw = str2double(get(app.edTw,"String"));
  if ~(isfinite(Tw) && Tw > 0), Tw = 0.01; end
  set(app.edTw,"String",num2str(Tw));

  a1 = get(app.slA1,"Value");
  a2 = get(app.slA2,"Value");
  b2 = get(app.slB2,"Value");
  set(app.txtA1,"String",sprintf("%.3f",a1));
  set(app.txtA2,"String",sprintf("%.3f",a2));
  set(app.txtB2,"String",sprintf("%.3f",b2));

  idx = find(uint8([steps.pwm]) == uint8(pwmSel), 1, "first");
  if isempty(idx), drawEmptyAll(pwmSel, chMode); return; end

  tAll = double(steps(idx).t_us(:)) * 1e-6;
  chId = double(steps(idx).ch(:));
  tA = tAll(chId==0);
  tB = tAll(chId==1);

  if chMode==1
    t_evt = tA;
  elseif chMode==2
    t_evt = tB;
  else
    t_evt = sort([tA; tB]);
  end
  if numel(t_evt) < 3, drawEmptyAll(pwmSel, chMode); return; end

  dt = diff(t_evt); dt = max(dt, eps);
  tP = t_evt(2:end);
  fP = 1./dt;

  [tW, fW] = windowFreq(t_evt, Tw);

  % "Processed" (panel-specific)
  medN = 11;
  fP_med  = medfilt1(fP, medN, "truncate");
  fW_med  = medfilt1(fW, medN, "truncate");
  fP_iir1 = iir1(fP, a1);
  fW_iir1 = iir1(fW, a1);
  fP_iir2 = iir2_alphaBeta(fP, a2, b2);
  fW_iir2 = iir2_alphaBeta(fW, a2, b2);

  switch modeSel
    case 1 % accumulate
      renderLines(app.axRaw, "RAW", pwmSel, chMode, [t0 t1], ...
        "acc raw", tW, fW, "", [], [], "", [], [], "", [], []);
      renderLines(app.axMed, "MEDIAN", pwmSel, chMode, [t0 t1], ...
        "acc raw", tW, fW, "acc proc", tW, fW_med, "", [], [], "", [], []);
      renderLines(app.axIIR1,"IIR1", pwmSel, chMode, [t0 t1], ...
        "acc raw", tW, fW, "acc proc", tW, fW_iir1, "", [], [], "", [], []);
      renderLines(app.axIIR2,"IIR2", pwmSel, chMode, [t0 t1], ...
        "acc raw", tW, fW, "acc proc", tW, fW_iir2, "", [], [], "", [], []);

    case 2 % period
      renderLines(app.axRaw, "RAW", pwmSel, chMode, [t0 t1], ...
        "per raw", tP, fP, "", [], [], "", [], [], "", [], []);
      renderLines(app.axMed, "MEDIAN", pwmSel, chMode, [t0 t1], ...
        "per raw", tP, fP, "per proc", tP, fP_med, "", [], [], "", [], []);
      renderLines(app.axIIR1,"IIR1", pwmSel, chMode, [t0 t1], ...
        "per raw", tP, fP, "per proc", tP, fP_iir1, "", [], [], "", [], []);
      renderLines(app.axIIR2,"IIR2", pwmSel, chMode, [t0 t1], ...
        "per raw", tP, fP, "per proc", tP, fP_iir2, "", [], [], "", [], []);

    case 3 % compare
      renderLines(app.axRaw, "RAW (compare raw)", pwmSel, chMode, [t0 t1], ...
        "acc raw", tW, fW, "per raw", tP, fP, "", [], [], "", [], []);
      renderLines(app.axMed, "MEDIAN (compare proc)", pwmSel, chMode, [t0 t1], ...
        "acc proc", tW, fW_med, "per proc", tP, fP_med, "", [], [], "", [], []);
      renderLines(app.axIIR1,"IIR1 (compare proc)", pwmSel, chMode, [t0 t1], ...
        "acc proc", tW, fW_iir1, "per proc", tP, fP_iir1, "", [], [], "", [], []);
      renderLines(app.axIIR2,"IIR2 (compare proc)", pwmSel, chMode, [t0 t1], ...
        "acc proc", tW, fW_iir2, "per proc", tP, fP_iir2, "", [], [], "", [], []);
  end
end

function renderLines(ax, tag, pwm, chMode, tRange, ...
  name1, t1, y1, name2, t2, y2, name3, t3, y3, name4, t4, y4)

  cla(ax,"reset"); grid(ax,"on"); hold(ax,"on");
  h = gobjects(0); lab = {};

  if ~isempty(t1)
    h(end+1)=plot(ax,t1,y1,"-","Color",[0 0.6 0],"LineWidth",1); lab{end+1}=name1; %#ok<AGROW>
  end
  if ~isempty(t2)
    h(end+1)=plot(ax,t2,y2,"-","Color",[0.85 0 0],"LineWidth",1); lab{end+1}=name2; %#ok<AGROW>
  end
  if ~isempty(t3)
    h(end+1)=plot(ax,t3,y3,"-","Color",[0 0.7 0.7],"LineWidth",1); lab{end+1}=name3; %#ok<AGROW>
  end
  if ~isempty(t4)
    h(end+1)=plot(ax,t4,y4,"-","Color",[0.8 0 0.8],"LineWidth",1); lab{end+1}=name4; %#ok<AGROW>
  end

  xlim(ax, tRange);
  xlabel(ax,"t (s)"); ylabel(ax,"frequency (Hz)");
  title(ax, sprintf("%s | PWM=%d | CH=%s", tag, pwm, chName(chMode)));
  if ~isempty(h), legend(ax,h,lab,"Location","northeast"); end
end

function drawEmptyAll(pwm, chMode)
  drawEmpty(app.axRaw,  "RAW",   pwm, chMode);
  drawEmpty(app.axMed,  "MEDIAN",pwm, chMode);
  drawEmpty(app.axIIR1, "IIR1",  pwm, chMode);
  drawEmpty(app.axIIR2, "IIR2",  pwm, chMode);
end

function drawEmpty(ax, tag, pwm, chMode)
  cla(ax,"reset"); grid(ax,"on");
  title(ax, sprintf("%s | PWM=%d | CH=%s | NO DATA", tag, pwm, chName(chMode)));
  xlabel(ax,"t (s)");
end

function y = iir1(x, alpha)
  y = zeros(size(x)); y(1)=x(1);
  for k=2:numel(x)
    y(k) = y(k-1) + alpha*(x(k)-y(k-1));
  end
end

function y = iir2_alphaBeta(x, alpha, beta)
  y = zeros(size(x));
  y(1) = x(1);
  if numel(x) >= 2
    y(2) = (1-alpha)*y(1) + alpha*x(2);
  end
  for k=3:numel(x)
    y(k) = (1-alpha)*y(k-1) + alpha*x(k) + beta*(y(k-1)-y(k-2));
  end
end

function [t_mid, f_win] = windowFreq(t_evt, Tw)
  t_evt = t_evt(:);
  edges = t_evt(1):Tw:(t_evt(end)+Tw);
  if numel(edges) < 2
    t_mid = t_evt(1); f_win = 0; return;
  end
  c = histcounts(t_evt, edges);
  t_mid = edges(1:end-1) + Tw/2;
  f_win = c(:)' / Tw;
end

function s = chName(m)
  if m==1, s="A"; elseif m==2, s="B"; else, s="BOTH"; end
end

end
