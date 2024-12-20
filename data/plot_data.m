clear; clc; close all;

% MATLAB Script for Visualizing All Columns (Column-wise Plot)

% 1. 파일 경로 설정
filename = 'extended_output.csv'; % CSV 파일 경로 설정

% 2. 데이터 읽기 (줄 단위 처리)
fileID = fopen(filename, 'r');
rawData = textscan(fileID, '%s', 'Delimiter', '\n'); % 줄 단위로 읽기
fclose(fileID);

% 3. 데이터 가공
rawData = rawData{1}; % textscan 결과에서 셀 배열 추출
parsedData = cellfun(@(x) sscanf(x, '%f,'), rawData, 'UniformOutput', false); % 각 줄을 숫자로 변환

% 4. 데이터를 행렬로 변환 (각 행을 동일한 열 개수로 맞춰야 함)
maxCols = max(cellfun(@length, parsedData)); % 가장 긴 행의 길이 계산
matrixData = cell2mat(cellfun(@(row) padarray(row', [0 maxCols-length(row)], NaN, 'post'), parsedData, 'UniformOutput', false)); 

% 5. 데이터 확인
%disp('Matrix Data:');
%disp(matrixData);

% 6. 열(column) 단위 플롯
figure(1);
hold on;
for col = 1:size(matrixData, 2) % 열 개수만큼 반복
    plot(matrixData(:, col), '-o', 'LineWidth', 1.5, 'DisplayName', ['Column ' num2str(col)]);
end

hold off;

% 7. 플롯 설정
title('input force');
xlabel('Row Index');
ylabel('Value');
legend show;
grid on;


%% 

% 1. 파일 경로 설정
filename = 'state.csv'; % CSV 파일 경로 설정

% 2. 데이터 읽기 (줄 단위 처리)
fileID = fopen(filename, 'r');
rawData = textscan(fileID, '%s', 'Delimiter', '\n'); % 줄 단위로 읽기
fclose(fileID);

% 3. 데이터 가공
rawData = rawData{1}; % textscan 결과에서 셀 배열 추출
parsedData = cellfun(@(x) sscanf(x, '%f,'), rawData, 'UniformOutput', false); % 각 줄을 숫자로 변환

% 4. 데이터를 행렬로 변환 (각 행을 동일한 열 개수로 맞춰야 함)
maxCols = max(cellfun(@length, parsedData)); % 가장 긴 행의 길이 계산
matrixData = cell2mat(cellfun(@(row) padarray(row', [0 maxCols-length(row)], NaN, 'post'), parsedData, 'UniformOutput', false)); 

% 5. 데이터 확인
%disp('pos z:');
%disp(matrixData(:,6));
%disp('vel z:');
%disp(matrixData(:,12));


% 6. 열(column) 단위 플롯
figure(2);
hold on;
%for col = 1:size(matrixData, 2) % 열 개수만큼 반복
%    plot(matrixData(:, col), '-o', 'LineWidth', 1.5, 'DisplayName', ['Column ' num2str(col)]);
%end
plot(matrixData(:, 1), '.', 'LineWidth', 1.5, 'DisplayName', 'x');
plot(matrixData(:, 2), '.', 'LineWidth', 1.5, 'DisplayName', 'x vel');

hold off;

% 7. 플롯 설정
title('state');
xlabel('Row Index');
ylabel('Value');
legend show;
grid on;


%% 

% 1. 파일 경로 설정
%filename = 'cost.csv'; % CSV 파일 경로 설정
%fileID = fopen(filename, 'r');
%rawData = textscan(fileID, '%s', 'Delimiter', '\n'); % 줄 단위로 읽기
%fclose(fileID);
% 3. 데이터 가공
%rawData = rawData{1}; % textscan 결과에서 셀 배열 추출
%parsedData = cellfun(@(x) sscanf(x, '%f,'), rawData, 'UniformOutput', false) % 각 줄을 숫자로 변환

% 4. 데이터를 행렬로 변환 (각 행을 동일한 열 개수로 맞춰야 함)
%maxCols = max(cellfun(@length, parsedData)); % 가장 긴 행의 길이 계산
%matrixData = cell2mat(cellfun(@(row) padarray(row', [0 maxCols-length(row)], NaN, 'post'), parsedData, 'UniformOutput', false));
