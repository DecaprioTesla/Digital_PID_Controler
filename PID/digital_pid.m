%%%PID控制算法
clear;
clc;
ts=0.5;  
sys=tf(4,[1 2 0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');
e_1=0;      
e_2=0;
u_1=0.0;   
u_2=0.0;
y_1=0;
y_2=0;
Esum=0;
%PID参数
kp=1.7;    
ki=0.5;
kd=0.2;
u=zeros(1,100);
time=zeros(1,100);
ts=0.5;
for k=1:1:100
    time(k)=k*ts;   %时间参数由 1 时刻开始
    r(k)=1;      %期望值
    y(k)=-1*den(3)*y_2-den(2)*y_1+num(3)*u_2+num(2)*u_1+num(1)*u(k);
    e(k)=r(k)-y(k);  
    u(k)=kp*(e(k)-e_1)+ki*e(k)+kd*(e(k)-2*e_1+e_2); %系统PID控制器输出序列
    u_2=u_1;
    y_2=y_1;
    e_2=e_1;
    u_1=u(k);    	%前一个的控制器输出值
    y_1=y(k);    	%前一个的系统响应输出值
    e_1=e(k);       
end

%%%%全量式算法

ksp=1.5;
ksi=0.2;
ksd=0;
for k=1:1:100
    time(k)=k*ts;   %时间参数由 1 时刻开始
    r(k)=1;      %期望值
    ys(k)=-1*den(3)*y_2-den(2)*y_1+num(3)*u_2+num(2)*u_1+num(1)*u(k);% 初始化系统响应输出序列，与前一时刻的输出有关（这就是差分方程）
    e(k)=r(k)-ys(k); 
    u(k)=ksp*(e(k)-e_1)+ksi*Esum+ksd*(e_1-e_2);
    Esum=Esum+e(k);
    u_2=u_1;
    y_2=y_1;
    e_2=e_1;
    u_1=u(k);    	%前一个的控制器输出值
    y_1=ys(k);    	%前一个的系统响应输出值
    e_1=e(k);       %前一个误差信号的值
end
%（仅绘制过渡过程的曲线，x坐标限制为[0,1]）
figure(1);
p1=plot(time,r,'-.');xlim([0,50]);hold on;%指令信号的曲线（即期望输入）
p2=plot(time,y,'b--');xlim([0,50]);hold on;%不含积分分离的PID曲线
figure(2);
p1=plot(time,r,'-.');xlim([0,50]);hold on;%指令信号的曲线（即期望输入）
p2=plot(time,ys,'b--');xlim([0,50]);hold on;%不含积分分离的PID曲线


