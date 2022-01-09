% 清空
clear all
close all
clc

% 定义一些常量
global CenterFrequency
global PlayTime

SampleRate = 228e3; % 信号的采样率
FrequencyDeviation = 75e3; % 最大频偏
AudioSampleRate = 45.6e3; % 最终播放的音频的采样率
SamplesPerFrame = 4095; % 接收机发来的每一帧里包含多少组数据
FrameTime = SamplesPerFrame/SampleRate; % 由前面的数据计算出每一帧所需的时间,用于计时
PlayTime = 10; % 总播放时间,单位:s
CenterFrequency = 92.1e6; % 接收机的频率,单位:Hz.这里92.1MHz为天津相声广播:-)

% 去加重滤波器的设计，为了更好的性能表现，将设计好的滤波器直接作为常量储存
%function [sosMatrix, ScaleValues] = deemphFilterDesigner...
%  (pFs,FilterTimeConstant)
%bs = [0 1];
%as = [FilterTimeConstant 1];
%[bz,az] = bilinear(bs, as, pFs, 1/(2*pi*FilterTimeConstant));
%[Az,Fz] = freqz(bz , az, 2048, pFs);
%filtDesigner = fdesign.arbmag('N,B,F,A', 5, 1, Fz, abs(Az), pFs);
%DeemphFilter = design(filtDesigner,'iirlpnorm', 'SystemObject', true);
%sosMatrix = DeemphFilter.SOSMatrix;
%ScaleValues = DeemphFilter.ScaleValues;
%end
% [sosMatrixDeemp,ScaleValuesDeemp] = coder.const...
%    (@deemphFilterDesigner, 228000, 75e-6);
sosMatrixDeemph = coder.const(...
        [ 1.7272    0.9539    0.1820    1.0000    0.5459    0.1021
        0.0592   -0.0020    0.0021    1.0000   -0.0275    0.0347
        0.5002    0.4997         0    1.0000   -0.9432         0 ]);
ScaleValuesDeemph = coder.const([0.5562    1.0000    1.0000    1.0000]');
% 低通滤波器，用于得到228kHz的准音频信号，设计好的滤波器作为常量储存
% D = fdesign.lowpass('Fp,Fst,Ap,Ast', 0.133, 0.166, 1, 80);
% f = design(D, 'equiripple', 'systemobject', true);
% numerator = f.Numerator;
LowPassNumerator = coder.const(10^-4*[ ...
     1 1 0 -1 -3 -6 -10 -15 -22 -29 -35 -41 -43 -43 -39 -32 -21 -8 5 17 ...
     26 30 29 22 11 -3 -16 -27 -34 -34 -27 -15 2 19 34 43 44 36 20 -1 -24 ...
     -45 -57 -59 -49 -28 0 32  60 77  81 68 40 0 -43 -82 -108 -114 -97 -57 ...
     -1  62 120 160 172 148 89 2 -101 -200 -274 -304 -273 -172 -2  226 ...
     494 773 1033 1245 1383 1431 1383 1245 1033 773 494 226 -2 -172 -273 ...
     -304 -274 -200 -101 2 89 148 172 160 120 62 -1 -57 -97 -114 -108 -82 ...
     -43 0 40 68 81 77 60 32 0 -28 -49 -59 -57 -45 -24 -1 20 36 44 43 34 ...
     19 2 -15 -27 -34 -34 -27 -16 -3 11 22 29 30 26 17 5 -8 -21 -32 -39 ...
     -43 -43 -41 -35 -29 -22 -15 -10 -6 -3 -1 0 1  1 ...
     ]);

% 调用去加重滤波器对象
pDeEmphFilter = dsp.BiquadFilter(...
        'Structure',    'Direct form II', ...
        'SOSMatrix',    sosMatrixDeemph, ...
        'ScaleValues',  ScaleValuesDeemph);

% 调用多级FIR码率转换器对象
pRateConv228 = dsp.SampleRateConverter( ...
                              'InputSampleRate', SampleRate, ...
                              'OutputSampleRate', 228e3, ...
                              'Bandwidth', 125e3, ...
                              'OutputRateTolerance', 1e-6);
pAudioRateConv = dsp.SampleRateConverter('Bandwidth',30e3,...
    'InputSampleRate',  228e3, ...
    'OutputSampleRate', AudioSampleRate, ...
    'OutputRateTolerance', 1e-6);

% 调用低通滤波器对象
pLowPass = dsp.FIRFilter('Numerator', LowPassNumerator);

% 调用接收机RTL-SDR的硬件接口对象
rxsdr = comm.SDRRTLReceiver('0','CenterFrequency',CenterFrequency,'SampleRate',SampleRate, ...
    'SamplesPerFrame',SamplesPerFrame,'EnableTunerAGC',true,'OutputDataType','double');

%调用音频播放器对象
player = audioDeviceWriter('SampleRate',AudioSampleRate);

% 循环之前将循环体内的变量初始化
NowTime = 0; % 当前时间
LastSample = 0+0*1i; % LastSample为上一组的最后一个信号采样，为复数

% 主循环
while NowTime < PlayTime % 当前时间 < 播放总时间 
    
    % 从RTL-SDR硬件中接收一帧信号
    x = rxsdr();
    
    % 信号初始解码,从载波中分离出原始信号(调频的本质是调相)
    phase =  angle(x .* conj([LastSample;x(1:end-1,:)]));
    LastSample = x(end,:);
    yDemod = (SampleRate / (2*pi*FrequencyDeviation)) .* phase;

    % 转换为中间采样率228kHz
    yDemodIntermediate228 = pRateConv228(yDemod);

    % 低通滤波器，用于分离出准音频信号（单声道）：
    yAudioIntermediate = pLowPass(yDemodIntermediate228);
    
    % 去加重,也就是广播电台发射信号时所进行的"预加重"的逆操作
    yFilt =  pDeEmphFilter(yAudioIntermediate);

    % 将中间采样率228kHz的信号转换为音频采样率
    yAudio = pAudioRateConv(yFilt);
    
    % 播放声音信号,
    % 本来打算采用更简单的"sound(yAudio,AudioSampleRate);",但是此法无法连续播放
    player(yAudio);
    
    % 更新播放时间
    NowTime = NowTime + FrameTime;
end

% 绘制4张频谱图
xInFreq = fftshift(fft(x));
freq = (-4095/2:4095/2-1)*(SampleRate/4095); % 频域
power = abs(xInFreq).^2/4095; % 功率
subplot(2,2,1)
plot(freq,power)
xlabel('Raw Signal Frequency')
ylabel('Power')

yAudioIntermediateInFreq = fftshift(fft(yAudioIntermediate));
freq = (-4095/2:4095/2-1)*(SampleRate/4095);
power = abs(yAudioIntermediateInFreq).^2/4095;
subplot(2,2,2)
plot(freq,power)
xlabel('Filtered Signal Frequency')
ylabel('Power')

yFiltInFreq = fftshift(fft(yFilt));
freq = (-4095/2:4095/2-1)*(SampleRate/4095);
power = abs(yFiltInFreq).^2/4095;
subplot(2,2,3)
plot(freq,power)
xlabel('De-Emphasized Signal Frequency')
ylabel('Power')

yAudioInFreq = fftshift(fft(yAudio));
freq = (-819/2:819/2-1)*(AudioSampleRate/819);
power = abs(yAudioInFreq).^2/819;
subplot(2,2,4)
plot(freq,power)
xlabel('Audio Frequency')
ylabel('Power')

% 释放此前调用的所有对象
release(pDeEmphFilter)
release(pRateConv228)
release(pAudioRateConv)
release(pLowPass)
release(rxsdr)
release(player)