%%%PID�����㷨
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
%PID����
kp=1.7;    
ki=0.5;
kd=0.2;
u=zeros(1,100);
time=zeros(1,100);
ts=0.5;
for k=1:1:100
    time(k)=k*ts;   %ʱ������� 1 ʱ�̿�ʼ
    r(k)=1;      %����ֵ
    y(k)=-1*den(3)*y_2-den(2)*y_1+num(3)*u_2+num(2)*u_1+num(1)*u(k);
    e(k)=r(k)-y(k);  
    u(k)=kp*(e(k)-e_1)+ki*e(k)+kd*(e(k)-2*e_1+e_2); %ϵͳPID�������������
    u_2=u_1;
    y_2=y_1;
    e_2=e_1;
    u_1=u(k);    	%ǰһ���Ŀ��������ֵ
    y_1=y(k);    	%ǰһ����ϵͳ��Ӧ���ֵ
    e_1=e(k);       
end

%%%%ȫ��ʽ�㷨

ksp=1.5;
ksi=0.2;
ksd=0;
for k=1:1:100
    time(k)=k*ts;   %ʱ������� 1 ʱ�̿�ʼ
    r(k)=1;      %����ֵ
    ys(k)=-1*den(3)*y_2-den(2)*y_1+num(3)*u_2+num(2)*u_1+num(1)*u(k);% ��ʼ��ϵͳ��Ӧ������У���ǰһʱ�̵�����йأ�����ǲ�ַ��̣�
    e(k)=r(k)-ys(k); 
    u(k)=ksp*(e(k)-e_1)+ksi*Esum+ksd*(e_1-e_2);
    Esum=Esum+e(k);
    u_2=u_1;
    y_2=y_1;
    e_2=e_1;
    u_1=u(k);    	%ǰһ���Ŀ��������ֵ
    y_1=ys(k);    	%ǰһ����ϵͳ��Ӧ���ֵ
    e_1=e(k);       %ǰһ������źŵ�ֵ
end
%�������ƹ��ɹ��̵����ߣ�x��������Ϊ[0,1]��
figure(1);
p1=plot(time,r,'-.');xlim([0,50]);hold on;%ָ���źŵ����ߣ����������룩
p2=plot(time,y,'b--');xlim([0,50]);hold on;%�������ַ����PID����
figure(2);
p1=plot(time,r,'-.');xlim([0,50]);hold on;%ָ���źŵ����ߣ����������룩
p2=plot(time,ys,'b--');xlim([0,50]);hold on;%�������ַ����PID����


