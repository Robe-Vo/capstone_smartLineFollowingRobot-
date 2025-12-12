%% ============================================================
% analyzeSteadyState.m
% Phân tích steady-state từng bậc PWM cho các alpha khác nhau
% Input: file .mat từ stepFilterExperiment (EMA test)
% Output: bảng thống kê per-step, per-alpha và các biểu đồ đánh giá
%% ============================================================

clear; clc; close all;

%% ===== Chọn file .mat chứa stepResults =====
[fn, fp] = uigetfile('*.mat', 'Chọn file MAT stepResults...');
if isequal(fn,0)
    error("Không chọn file");
end

S = load(fullfile(fp,fn));
if ~isfield(S,'stepResults')
    error("File không có biến stepResults");
end
stepResults = S.stepResults;

%% ===== Các mức PWM trong profile =====
% Trích từ speedCmd của lần chạy đầu (giả sử giống nhau)
u = stepResults(1).speedCmd;
uLevels = unique(u(u>0));   % các mức PWM khác 0
nLvl = numel(uLevels);

fprintf("Các mức PWM: ");
disp(uLevels');

%% ===== Tham số steady-state =====
steady_ignore_start = 0.5;   % bỏ 0.5s đầu mỗi step
steady_ignore_end   = 0.5;   % bỏ 0.5s cuối mỗi step

%% ===== Chuẩn bị kết quả =====
nAlpha = numel(stepResults);

stats = struct([]);

for iA = 1:nAlpha
    t    = stepResults(iA).t;
    rpm  = stepResults(iA).rpm;
    ucmd = stepResults(iA).speedCmd;
    alpha = stepResults(iA).alpha;

    fprintf("\n=== Alpha = %.3f ===\n", alpha);

    for iLvl = 1:nLvl
        pwmVal = uLevels(iLvl);

        % Tìm các đoạn mà PWM == pwmVal
        idx = find(ucmd == pwmVal);

        if isempty(idx)
            stats(iA).perStep(iLvl).pwm = pwmVal;
            stats(iA).perStep(iLvl).mean_rpm = NaN;
            stats(iA).perStep(iLvl).std_rpm  = NaN;
            stats(iA).perStep(iLvl).n = 0;
            continue;
        end

        % Phân đoạn liên tục
        d = diff(idx);
        blockStart = [idx(1); idx(find(d>1)+1)];
        blockEnd   = [idx(find(d>1)); idx(end)];

        rpm_all = [];

        for b = 1:numel(blockStart)
            i1 = blockStart(b);
            i2 = blockEnd(b);

            t1 = t(i1);
            t2 = t(i2);
            Tblock = t2 - t1;

            if Tblock < (steady_ignore_start + steady_ignore_end)
                continue;
            end

            % steady-state region
            steady_idx = find( t >= (t1 + steady_ignore_start) & ...
                                t <= (t2 - steady_ignore_end) );

            rpm_all = [rpm_all; rpm(steady_idx)];
        end

        if isempty(rpm_all)
            stats(iA).perStep(iLvl).pwm = pwmVal;
            stats(iA).perStep(iLvl).mean_rpm = NaN;
            stats(iA).perStep(iLvl).std_rpm  = NaN;
            stats(iA).perStep(iLvl).n = 0;
        else
            stats(iA).perStep(iLvl).pwm = pwmVal;
            stats(iA).perStep(iLvl).mean_rpm = mean(rpm_all);
            stats(iA).perStep(iLvl).std_rpm  = std(rpm_all);
            stats(iA).perStep(iLvl).n = numel(rpm_all);
        end

        fprintf("PWM=%3d → mean=%8.2f | std=%8.2f | n=%d\n", ...
            pwmVal, stats(iA).perStep(iLvl).mean_rpm, ...
            stats(iA).perStep(iLvl).std_rpm, stats(iA).perStep(iLvl).n);
    end
end

%% ===== Lưu kèm file =====
save(fullfile(fp, fn), 'stats', '-append');
fprintf("\nĐã ghi 'stats' vào file: %s\n", fn);

%% ===== Plot heatmap Std theo alpha và mỗi PWM =====
alphaVals = arrayfun(@(s) s.alpha, stepResults);

stdMat = nan(nAlpha, nLvl);
for iA = 1:nAlpha
    for iLvl = 1:nLvl
        stdMat(iA,iLvl) = stats(iA).perStep(iLvl).std_rpm;
    end
end

figure;
imagesc(uLevels, alphaVals, stdMat);
colorbar;
xlabel('PWM level');
ylabel('\alpha (EMA)');
title('Steady-state std(rpm) per alpha & per PWM');
set(gca,'YDir','normal');

%% ===== Plot mean rpm theo alpha =====
figure;
for iLvl = 1:nLvl
    meanVals = arrayfun(@(s) s.perStep(iLvl).mean_rpm, stats);
    plot(alphaVals, meanVals, '-o', 'LineWidth',1.2); hold on;
end
grid on;
xlabel('\alpha');
ylabel('Mean rpm (steady state)');
title('Mean rpm per PWM level');
legend(arrayfun(@(x) sprintf('PWM %d',x), uLevels,'UniformOutput',false));
