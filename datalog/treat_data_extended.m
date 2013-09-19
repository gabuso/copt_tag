ll=dir('*.txt');

[a,b] = sort([ll(:).datenum],'descend');

latest_fn = ll(b(1)).name

fp = fopen(latest_fn,'r');

f_data=textscan(fp,'%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;');

time  = f_data{1};
time = (time-time(1))/1000;

vx = f_data{2}/1000;
vy = f_data{3}/1000;

depth = f_data{4};
s_error = f_data{5};

front_cmd = f_data{6};
side_cmd = f_data{7};

alt_m = f_data{8};

inte_x = f_data{9};
inte_y = f_data{10};

pitch = f_data{11};
roll = f_data{12};

pitch_ref = f_data{13};
roll_ref = f_data{14};

vx_sig = f_data{15};
vy_sig = f_data{16};
pitch_sig = f_data{17};
roll_sig = f_data{18};

p0 = f_data{19};
p1 = f_data{20};
p2 = f_data{21};

yhat_n = f_data{22};


figure('Color',[1 1 1],'Position',[100,100,1300,700]);

subplot(2,3,1);
plot(time,[vx 0.1*(0.65-depth)]);
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
plot(time,[vy 0.0001*s_error]);
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


figure('Color',[1 1 1],'Position',[100,200,1300,700]);

subplot(2,3,1);
plot(time,inte_x);
legend('inte_x');
grid;

subplot(2,3,2);
plot(time,inte_y);
legend('inte_y');
grid;

subplot(2,3,3);
plot(time,[pitch ]);
legend('pitch');
grid;

subplot(2,3,4);
plot(time,[roll ]);
legend('roll');
grid;

subplot(2,3,5);
plot(time,pitch_ref);
legend('pitch_ref');
grid;

subplot(2,3,6);
plot(time,roll_ref);
legend('roll_ref');
grid;

figure('Color',[1 1 1],'Position',[100,300,1300,700]);

subplot(2,3,1);
plot(time,[vx vx_sig]);
legend('vx');
grid;

subplot(2,3,2);
plot(time,[vy vy_sig]);
legend('vy');
grid;

subplot(2,3,3);
plot(time,[pitch pitch_sig]);
legend('pitch');
grid;

subplot(2,3,4);
plot(time,[roll roll_sig]);
legend('roll');
grid;

subplot(2,3,5);
plot(time,[p0 p1 p2]);
legend('p');
grid;

subplot(2,3,6);
plot(time,yhat_n);
legend('yhat_n');
grid;

figure('Color',[1 1 1],'Position',[1600,100,400,400]);

subplot(2,3,1);
plot(time,alt_m);
legend('alt');
grid;
