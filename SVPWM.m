%The code runs using a time vector made using the linspace command

%Syms and constant definition
syms A_a A_b A_c f_a f_b f_c V_DC T_s t f A
pi = 3.141592;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Parameters definition

n = 10000;
V_DC = 540;
T_s = 0.0001;
t = linspace(0,1/50,n);
t_pwm = mod(t,T_s);
f = 50;
A = 300;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Input signals

s_a = A*sin(2*pi*f*t);
s_b = A*sin(2*pi*f*t-2*pi/3);
s_c = A*sin(2*pi*f*t-4*pi/3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Clark transformation from abc to alpha beta

s_alpha = 2/3*(s_a - 1/2*(s_b + s_c));
s_beta = sqrt(3)/3*(s_b-s_c);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Space vector angle and magnitude

SV_theta = atan2(s_beta, s_alpha);
for i = 1:length(t)
    if SV_theta(i) < 0
        SV_theta(i) = SV_theta(i) + 2*pi;
    end
end

SV_m = sqrt(s_alpha.^2+s_beta.^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Sector calculator

%Empty vector for storing sector data
v_sector = zeros(1, n);

%For loop that fills v_sector
for i = 1:length(t)
    if 0 <= SV_theta(i) && SV_theta(i) < pi/3
        sector = 1;
    elseif pi/3 <= SV_theta(i) && SV_theta(i) < 2*pi/3
        sector = 2;
    elseif 2*pi/3 <= SV_theta(i) && SV_theta(i) < 3*pi/3
        sector = 3;
    elseif 3*pi/3 <= SV_theta(i) && SV_theta(i) < 4*pi/3
        sector = 4;
    elseif 4*pi/3 <= SV_theta(i) && SV_theta(i) < 5*pi/3
        sector = 5;
    elseif 5*pi/3 <= SV_theta(i) && SV_theta(i) < 6*pi/3
        sector = 6;
    end
    v_sector(i) = sector;
end

%The Space Vector angle measured from beginning of sector
sector_theta = SV_theta - (v_sector-1)*pi/3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Dwell Time Calculator

a = SV_m/((2/3)*V_DC);

T_1 = T_s*a.*sin(pi/3 - sector_theta);
T_2 = T_s*a.*sin(sector_theta);
T_0 = T_s - (T_1 + T_2);

%Switch Times calculated based on sector and dwell times
for i = 1:length(t)
    if v_sector(i) == 1 || v_sector(i) == 3 || v_sector(i) == 5
        t_s0 = T_0(i)/2;
        t_s1 = t_s0+T_1(i);
        t_s2 = t_s1+T_2(i);
        t_s3 = t_s2+T_0(i);
        t_s4 = t_s3+T_2(i);
        t_s5 = t_s4+T_1(i);
        t_s6 = t_s5+T_0(i)/2;
    else
        t_s0 = T_0(i)/2;
        t_s1 = t_s0+T_2(i);
        t_s2 = t_s1+T_1(i);
        t_s3 = t_s2+T_0(i);
        t_s4 = t_s3+T_1(i);
        t_s5 = t_s4+T_2(i);
        t_s6 = t_s5+T_0(i)/2;
    end
    %Switch times are assigned to a corresponding vector
    t_sv0(i) = t_s0;
    t_sv1(i) = t_s1;
    t_sv2(i) = t_s2;
    t_sv3(i) = t_s3;
    t_sv4(i) = t_s4;
    t_sv5(i) = t_s5;
    t_sv6(i) = t_s6;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Switching Sequence Generator

%Switchstate lookup table
v_switchstate = [0,0,0;
                 1,0,0;
                 1,1,0;
                 0,1,0;
                 0,1,1;
                 0,0,1;
                 1,0,1;
                 1,1,1];

%Empty matrix for storing switchstates
switchstate = zeros(3,length(t));

%For loop that fills in switchstate matrix based on switch times and sector
for i = 1:length(t)
    if v_sector(i) == 1
        if t_pwm(i) < t_sv0(i)
            switchstate(:,i) = v_switchstate(1,:);
        elseif t_pwm(i) < t_sv1(i)
            switchstate(:,i) = v_switchstate(2,:);
        elseif t_pwm(i) < t_sv2(i)
            switchstate(:,i) = v_switchstate(3,:);
        elseif t_pwm(i) < t_sv3(i)
            switchstate(:,i) = v_switchstate(8,:);
        elseif t_pwm(i) < t_sv4(i)
            switchstate(:,i) = v_switchstate(3,:);
        elseif t_pwm(i) < t_sv5(i)
            switchstate(:,i) = v_switchstate(2,:);
        elseif t_pwm(i) < t_sv6(i)
            switchstate(:,i) = v_switchstate(1,:);
        end
    elseif v_sector(i) == 2
        if t_pwm(i) < t_sv0(i)
            switchstate(:,i) = v_switchstate(1,:);
        elseif t_pwm(i) < t_sv1(i)
            switchstate(:,i) = v_switchstate(4,:);
        elseif t_pwm(i) < t_sv2(i)
            switchstate(:,i) = v_switchstate(3,:);
        elseif t_pwm(i) < t_sv3(i)
            switchstate(:,i) = v_switchstate(8,:);
        elseif t_pwm(i) < t_sv4(i)
            switchstate(:,i) = v_switchstate(3,:);
        elseif t_pwm(i) < t_sv5(i)
            switchstate(:,i) = v_switchstate(4,:);
        elseif t_pwm(i) < t_sv6(i)
            switchstate(:,i) = v_switchstate(1,:);
        end
    elseif v_sector(i) == 3
        if t_pwm(i) < t_sv0(i)
            switchstate(:,i) = v_switchstate(1,:);
        elseif t_pwm(i) < t_sv1(i)
            switchstate(:,i) = v_switchstate(4,:);
        elseif t_pwm(i) < t_sv2(i)
            switchstate(:,i) = v_switchstate(5,:);
        elseif t_pwm(i) < t_sv3(i)
            switchstate(:,i) = v_switchstate(8,:);
        elseif t_pwm(i) < t_sv4(i)
            switchstate(:,i) = v_switchstate(5,:);
        elseif t_pwm(i) < t_sv5(i)
            switchstate(:,i) = v_switchstate(4,:);
        elseif t_pwm(i) < t_sv6(i)
            switchstate(:,i) = v_switchstate(1,:);
        end
    elseif v_sector(i) == 4
        if t_pwm(i) < t_sv0(i)
            switchstate(:,i) = v_switchstate(1,:);
        elseif t_pwm(i) < t_sv1(i)
            switchstate(:,i) = v_switchstate(6,:);
        elseif t_pwm(i) < t_sv2(i)
            switchstate(:,i) = v_switchstate(5,:);
        elseif t_pwm(i) < t_sv3(i)
            switchstate(:,i) = v_switchstate(8,:);
        elseif t_pwm(i) < t_sv4(i)
            switchstate(:,i) = v_switchstate(5,:);
        elseif t_pwm(i) < t_sv5(i)
            switchstate(:,i) = v_switchstate(6,:);
        elseif t_pwm(i) < t_sv6(i)
            switchstate(:,i) = v_switchstate(1,:);
        end
    elseif v_sector(i) == 5
        if t_pwm(i) < t_sv0(i)
            switchstate(:,i) = v_switchstate(1,:);
        elseif t_pwm(i) < t_sv1(i)
            switchstate(:,i) = v_switchstate(6,:);
        elseif t_pwm(i) < t_sv2(i)
            switchstate(:,i) = v_switchstate(7,:);
        elseif t_pwm(i) < t_sv3(i)
            switchstate(:,i) = v_switchstate(8,:);
        elseif t_pwm(i) < t_sv4(i)
            switchstate(:,i) = v_switchstate(7,:);
        elseif t_pwm(i) < t_sv5(i)
            switchstate(:,i) = v_switchstate(6,:);
        elseif t_pwm(i) < t_sv6(i)
            switchstate(:,i) = v_switchstate(1,:);
        end
    elseif v_sector(i) == 6
        if t_pwm(i) < t_sv0(i)
            switchstate(:,i) = v_switchstate(1,:);
        elseif t_pwm(i) < t_sv1(i)
            switchstate(:,i) = v_switchstate(2,:);
        elseif t_pwm(i) < t_sv2(i)
            switchstate(:,i) = v_switchstate(7,:);
        elseif t_pwm(i) < t_sv3(i)
            switchstate(:,i) = v_switchstate(8,:);
        elseif t_pwm(i) < t_sv4(i)
            switchstate(:,i) = v_switchstate(7,:);
        elseif t_pwm(i) < t_sv5(i)
            switchstate(:,i) = v_switchstate(2,:);
        elseif t_pwm(i) < t_sv6(i)
            switchstate(:,i) = v_switchstate(1,:);
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Figures

%Reference signals
figure
plot(t,s_a)
hold on
plot(t,s_b)
hold on
plot(t,s_c)
title('Reference signals');
xlabel('Time');
ylabel('Voltage');
legend('s_a','s_b','s_c')

%Clarke transformed reference signals
figure
plot(t,s_alpha)
hold on 
plot(t,s_beta)
title('Transformed reference signals');
xlabel('Time');
ylabel('Voltage');
legend('s_alpha','s_beta')

%Space Vector angle
figure
plot(t,SV_theta)
title('Space Vector angle');
xlabel('Time');
ylabel('Angle');

%Sector
figure
plot(t,v_sector)
title('Sector');
xlabel('Time');
ylabel('Sector #');

%Angle inside sector
figure
plot(t,sector_theta)
title('Angle inside sector');
xlabel('Time');
ylabel('Angle');

%Dwell times
figure
plot(t,T_0)
hold on
plot(t,T_1)
hold on
plot(t,T_2)
title('Dwell times');
legend('T_0','T_1','T_2')

%Switchstates

%Scaled reference signals
s_an = s_a/(2*A)+0.5;
s_bn = s_b/(2*A)+0.5;
s_cn = s_c/(2*A)+0.5;

%Calculate zero-sequence voltage for reference signals
Vmin = min([s_a; s_b; s_c]);
Vmax = max([s_a; s_b; s_c]);
Vzero = -0.5 * (Vmax + Vmin);

% Clamped reference signals (inject zero sequence)
s_ac = s_a + Vzero;
s_bc = s_b + Vzero;
s_cc = s_c + Vzero;

%Scaled and clamped reference signals
s_acn = s_ac/(2*A)+0.5;
s_bcn = s_bc/(2*A)+0.5;
s_ccn = s_cc/(2*A)+0.5;

figure
subplot(3,1,1);
plot(t,switchstate(1,:))
hold on
plot(t,s_an)
hold on
plot(t,s_acn)
title('Switch state for 1st leg');
xlabel('Time');
ylabel('Switchstate');

subplot(3,1,2);
plot(t, switchstate(2,:))
hold on
plot(t,s_bn)
hold on
plot(t,s_bcn)
title('Switch state for 2nd leg');
xlabel('Time');
ylabel('Switchstate');

subplot(3,1,3);
plot(t,switchstate(3,:))
hold on
plot(t,s_cn)
hold on
plot(t,s_ccn)
title('Switch state for 3rd leg');
xlabel('Time');
ylabel('Switchstate');

%sgtitle('Switchstates for all three inverter legs');