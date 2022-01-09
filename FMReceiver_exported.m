classdef FMReceiver_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        MortgageCalculatorUIFigure  matlab.ui.Figure
        GridLayout                  matlab.ui.container.GridLayout
        LeftPanel                   matlab.ui.container.Panel
        Button_2                    matlab.ui.control.Button
        RxLamp                      matlab.ui.control.Lamp
        RxLampLabel                 matlab.ui.control.Label
        MHzGauge                    matlab.ui.control.LinearGauge
        MHzGaugeLabel               matlab.ui.control.Label
        Switch                      matlab.ui.control.Switch
        Label                       matlab.ui.control.Label
        Button                      matlab.ui.control.Button
        sEditField                  matlab.ui.control.NumericEditField
        sEditFieldLabel             matlab.ui.control.Label
        MHzEditField                matlab.ui.control.NumericEditField
        MHzLabel                    matlab.ui.control.Label
        RightPanel                  matlab.ui.container.Panel
        Image                       matlab.ui.control.Image
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, event)
            currentFigureWidth = app.MortgageCalculatorUIFigure.Position(3);
            if(currentFigureWidth <= app.onePanelWidth)
                % Change to a 2x1 grid
                app.GridLayout.RowHeight = {296, 296};
                app.GridLayout.ColumnWidth = {'1x'};
                app.RightPanel.Layout.Row = 2;
                app.RightPanel.Layout.Column = 1;
            else
                % Change to a 1x2 grid
                app.GridLayout.RowHeight = {'1x'};
                app.GridLayout.ColumnWidth = {246, '1x'};
                app.RightPanel.Layout.Row = 1;
                app.RightPanel.Layout.Column = 2;
            end
        end

        % Value changed function: MHzEditField
        function FreqChanged(app, event)
            global CenterFrequency;
            CenterFrequency = app.MHzEditField.Value;
            app.MHzGauge.Value = CenterFrequency;
            
            %熄灭Ready灯，示意未正确初始化
            app.RxLamp.Color = [1,1,1];
        end

        % Value changed function: sEditField
        function TimeChanged(app, event)
            global PlayTime;
            PlayTime = app.sEditField.Value;
            
            %熄灭Ready灯，示意未正确初始化
            app.RxLamp.Color = [1,1,1];
        end

        % Button pushed function: Button
        function StartButtonPushed(app, event)
            
            % 全局变量和全局对象
            global PlayTime
            global SampleRate
            global AudioSampleRate
            global FrequencyDeviation
            global FrameTime

            global pDeEmphFilter
            global pRateConv228
            global pAudioRateConv
            global pLowPass
            global rxsdr
            global player

            % 循环之前将循环体内的变量初始化
            NowTime = 0; % 当前时间
            LastSample = 0+0*1i; % LastSample为上一组的最后一个信号采样，为复数

            %循环体
            while NowTime < PlayTime
    
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
            if app.Switch.Value
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
            end
        end

        % Button pushed function: Button_2
        function InitButtonPushed(app, event)
            %点亮Ready灯，示意初始化已完成
            app.RxLamp.Color = [1,0,0];
            % 清空
           % clear all
            %close all
           % clc
            
            % 全局变量和全局对象
            global CenterFrequency
            global PlayTime
            global SampleRate
            global AudioSampleRate
            global FrequencyDeviation
            global AudioSampleRate
            global SamplesPerFrame
            global FrameTime

            global pDeEmphFilter
            global pRateConv228
            global pAudioRateConv
            global pLowPass
            global rxsdr
            global player
            
            % 定义一些常量
            SampleRate = 228e3; % 信号的采样率
            FrequencyDeviation = 75e3; % 最大频偏
            AudioSampleRate = 45.6e3; % 最终播放的音频的采样率
            SamplesPerFrame = 4095; % 接收机发来的每一帧里包含多少组数据
            FrameTime = SamplesPerFrame/SampleRate; % 由前面的数据计算出每一帧所需的时间,用于计时
            PlayTime = app.sEditField.Value; % 总播放时间,单位:s
            CenterFrequency = round(app.MHzEditField.Value*1e6); % 接收机的频率,单位:Hz.缺省值为92.1MHz天津相声广播:-)

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

            
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create MortgageCalculatorUIFigure and hide until all components are created
            app.MortgageCalculatorUIFigure = uifigure('Visible', 'off');
            app.MortgageCalculatorUIFigure.AutoResizeChildren = 'off';
            app.MortgageCalculatorUIFigure.Position = [100 100 533 296];
            app.MortgageCalculatorUIFigure.Name = 'Mortgage Calculator';
            app.MortgageCalculatorUIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);

            % Create GridLayout
            app.GridLayout = uigridlayout(app.MortgageCalculatorUIFigure);
            app.GridLayout.ColumnWidth = {246, '1x'};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.ColumnSpacing = 0;
            app.GridLayout.RowSpacing = 0;
            app.GridLayout.Padding = [0 0 0 0];
            app.GridLayout.Scrollable = 'on';

            % Create LeftPanel
            app.LeftPanel = uipanel(app.GridLayout);
            app.LeftPanel.BackgroundColor = [1 1 1];
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;
            app.LeftPanel.Scrollable = 'on';

            % Create MHzLabel
            app.MHzLabel = uilabel(app.LeftPanel);
            app.MHzLabel.HorizontalAlignment = 'right';
            app.MHzLabel.Position = [58 220 64 22];
            app.MHzLabel.Text = '频率 / MHz';

            % Create MHzEditField
            app.MHzEditField = uieditfield(app.LeftPanel, 'numeric');
            app.MHzEditField.Limits = [70 108];
            app.MHzEditField.ValueDisplayFormat = '%.3f';
            app.MHzEditField.ValueChangedFcn = createCallbackFcn(app, @FreqChanged, true);
            app.MHzEditField.Position = [141 220 100 22];
            app.MHzEditField.Value = 92.1;

            % Create sEditFieldLabel
            app.sEditFieldLabel = uilabel(app.LeftPanel);
            app.sEditFieldLabel.HorizontalAlignment = 'right';
            app.sEditFieldLabel.Position = [68 167 46 22];
            app.sEditFieldLabel.Text = '时间 / s';

            % Create sEditField
            app.sEditField = uieditfield(app.LeftPanel, 'numeric');
            app.sEditField.Limits = [10 1000];
            app.sEditField.ValueDisplayFormat = '%.3f';
            app.sEditField.ValueChangedFcn = createCallbackFcn(app, @TimeChanged, true);
            app.sEditField.Position = [141 167 100 22];
            app.sEditField.Value = 10;

            % Create Button
            app.Button = uibutton(app.LeftPanel, 'push');
            app.Button.ButtonPushedFcn = createCallbackFcn(app, @StartButtonPushed, true);
            app.Button.FontSize = 20;
            app.Button.Position = [73 28 152 52];
            app.Button.Text = '接    收';

            % Create Label
            app.Label = uilabel(app.LeftPanel);
            app.Label.HorizontalAlignment = 'center';
            app.Label.Position = [76 110 36 22];
            app.Label.Text = '绘  图';

            % Create Switch
            app.Switch = uiswitch(app.LeftPanel, 'slider');
            app.Switch.Items = {'关', '开'};
            app.Switch.Position = [172 111 45 20];
            app.Switch.Value = '关';

            % Create MHzGaugeLabel
            app.MHzGaugeLabel = uilabel(app.LeftPanel);
            app.MHzGaugeLabel.HorizontalAlignment = 'center';
            app.MHzGaugeLabel.Position = [13 7 30 22];
            app.MHzGaugeLabel.Text = 'MHz';

            % Create MHzGauge
            app.MHzGauge = uigauge(app.LeftPanel, 'linear');
            app.MHzGauge.Limits = [70 108];
            app.MHzGauge.Orientation = 'vertical';
            app.MHzGauge.Position = [7 44 39 245];
            app.MHzGauge.Value = 92.1;

            % Create RxLampLabel
            app.RxLampLabel = uilabel(app.LeftPanel);
            app.RxLampLabel.HorizontalAlignment = 'right';
            app.RxLampLabel.Position = [163 260 25 22];
            app.RxLampLabel.Text = 'Rx';

            % Create RxLamp
            app.RxLamp = uilamp(app.LeftPanel);
            app.RxLamp.Position = [203 260 20 20];
            app.RxLamp.Color = [1 1 1];

            % Create Button_2
            app.Button_2 = uibutton(app.LeftPanel, 'push');
            app.Button_2.ButtonPushedFcn = createCallbackFcn(app, @InitButtonPushed, true);
            app.Button_2.Position = [62 252 63 36];
            app.Button_2.Text = '初始化';

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.Enable = 'off';
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 2;

            % Create Image
            app.Image = uiimage(app.RightPanel);
            app.Image.ScaleMethod = 'fill';
            app.Image.Position = [1 6 280 284];
            app.Image.ImageSource = 'Loudspeaker.jpg';

            % Show the figure after all components are created
            app.MortgageCalculatorUIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = FMReceiver_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.MortgageCalculatorUIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.MortgageCalculatorUIFigure)
        end
    end
end
