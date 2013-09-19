ll=dir('*.txt');

[a,b] = sort([ll(:).datenum],'descend');

latest_fn = ll(b(1)).name

fp = fopen(latest_fn,'r');

f_data=textscan(fp,'%f;%f;%f;%f;%f;%f;%f;%f');

time  = f_data{1};
time = (time-time(1))/1000;

vx = f_data{2}/1000;
vy = f_data{3}/1000;

depth = f_data{4};
s_error = f_data{5};

front_cmd = f_data{6};
side_cmd = f_data{7};

alt_m = f_data{8};


figure('Color',[1 1 1],'Position',[100,100,1300,700]);

subplot(2,3,1);
plot(time,vx);
legend('vx');
grid;

subplot(2,3,2);
plot(time,depth);
legend('depth');
grid;

subplot(2,3,3);
plot(time,front_cmd);
legend('front_cmd');
grid;

subplot(2,3,4);
plot(time,vy);
legend('vy');
grid;

subplot(2,3,5);
plot(time,s_error);
legend('s_error');
grid;

subplot(2,3,6);
plot(time,side_cmd);
legend('side_cmd');
grid;

figure('Color',[1 1 1],'Position',[1600,100,400,400]);

subplot(2,3,1);
plot(time,alt_m);
legend('alt');
grid;
