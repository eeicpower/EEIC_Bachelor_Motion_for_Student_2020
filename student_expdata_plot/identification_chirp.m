%% identification_chirp.m
%% 学生に共有
% date: 2021.9.29
% author: shirato, miyoshi
% 位置で取ると不安定なので，トルク指令値から速度までのシステム同定
% 時間，角速度，トルク指令値を測定。指令値から速度までをシステム同定。
% その後ctrldesign_PID.mで設計
% はじめの１周期のデータは捨て，続く3周期のデータを同定に使うため，4周期以上のチャープデータが必要。
% 0.1-50Hz(30s), 51Hz-100 Hz(10s), 101 Hz-500 Hz(1s)に分けてシステム同定すると精度が良く同定できた。
% 対数チャープを使うと1回のシステム同定で周波数幅広く精度良く同定できた。 2021/9/29

close all;
clear;

%% １回ですべての周波数を同定する場合
load('./matfiles/.mat');
fs = 1000;
tperiod = 20; 

[~,time_chirpstart] = min(abs(time-tperiod)); %はじめの１周期のデータは捨てる
time_chirpfin = time_chirpstart + (tperiod*1000)*3 -1; %３周期のデータを使う
time_chirp = time(time_chirpstart:time_chirpfin,1);
ctrlcmd_chirp = ctrlcmd(time_chirpstart:time_chirpfin,1);
velocity_chirp = velocity(time_chirpstart:time_chirpfin,1);
t = time_chirp;
x = ctrlcmd_chirp;
y = velocity_chirp;

y_detrend = detrend(y); %detrend
[txy,f] = tfestimate(x,y_detrend,fs*tperiod,[],[],fs); %detrend
G1 = frd(txy,f,'FrequencyUnit','Hz');
figure; bode(G1)

G1prime = G1;
% fittingがうまく行かない場合，次の行のコメントを外し，frdデータを見て形がきれいな周波数部分を取り出す
G1prime = fselect(G1,0.01,100);

% トルク指令値から角速度までを1次系として同定
Gest = tfest(G1prime,1);
Gfit = tf(Gest.Numerator,Gest.Denominator);
s = tf('s');
Gfit.name = 'fitting';
%Gfit2.sys = Gfit.sys * exp(-0.002*s);%actually, there must be a time delay
figure; bode(G1); hold on; bode(Gfit); legend;
%expfig(['plot/chirp/chirp_0p1Hz_500Hz_GcatGfitBode'],'-png','-pdf','-emf');

[cxy,f] = mscohere(x,y_detrend,fs*tperiod,[],[],fs);
G1.UserData = cxy;
figure;
semilogx(G1.Frequency,G1.UserData);
xlabel('frequency[Hz]')
ylabel('coherence')

%% 複数のデータを統合する場合
% numdata = 3;
% % 1つめのデータファイル
% data{1} = load('./matfiles/.mat');
% % 2つめのデータファイル
% data{2} = load('./matfiles/.mat');
% % 3つめのデータファイル
% data{3} = load('./matfiles/.mat');
% % 各データファイルに対応するチャープ周期
% tperiod = [30, 10, 1];
% 
% fs = 1000;
% 
% % data -> G
% for i = 1 : numdata
%     %はじめの１周期のデータは捨てる
%     [~,time_chirpstart] = min(abs(data{i}.time-tperiod(i)));
%     %３周期のデータを使う
%     time_chirpfin = time_chirpstart + (tperiod(i)*1000)*3 -1;
%     time_chirp = data{i}.time(time_chirpstart:time_chirpfin,1);
%     ctrlcmd_chirp = data{i}.ctrlcmd(time_chirpstart:time_chirpfin,1);
%     velocity_chirp = data{i}.velocity(time_chirpstart:time_chirpfin,1);
%     t = time_chirp;
%     x = ctrlcmd_chirp;
%     y = velocity_chirp;
% 
%     y_detrend = detrend(y); %detrend
%     [txy,f] = tfestimate(x,y_detrend,fs*tperiod(i),[],[],fs); %detrend
%     G{i} = frd(txy,f,'FrequencyUnit','Hz');
%     % Bode線図の表示
%     figure; bode(G{i});
%     % コヒーレンス（システム同定の質を表す，1に近いほど良い）の表示
%     figure; mscohere(x,y_detrend,fs*tperiod(i),[],[],fs);
% end
% 
% OP.fmin = 1; OP.fmax = 200;
% % frdデータの周波数範囲の切り出し
% G1_ = fselect(G{1},0.1,50);
% G2_ = fselect(G{2},51,100);
% G3_ = fselect(G{3},101,500);
% % frdデータを結合
% Gcat.sys = fcat(G1_,G2_,G3_);
% Gcat.name = 'data';
% figure; bode(Gcat.sys)
% 
% % トルク指令値から角速度までを1次系として同定
% Gfit.sys = tfest(Gcat.sys,1,0);
% s = tf('s');
% Gfit.name = 'fitting';
% Gfit2.sys = Gfit.sys * exp(-0.002*s);% actually, there must be a time delay
% Gfit2.name = 'fitting (w delay)';
% OP.fmin = 1; OP.fmax = 500;
% bode(Gcat.sys,Gfit.sys);
% 
