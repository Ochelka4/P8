syms A_a A_b A_c f_a f_b f_c V_DC T_s t f A
pi = 3.141592;

%Example signals and sample time definition
%
%
n = 10000;
V_DC = 50;
T_s = 1e-4;
t = linspace(0,1/50,n);
t_pwm = mod(t,T_s);
f = 50;
A = 10;
%
v_ar = A*sin(2*pi*f*t);
v_br = A*sin(2*pi*f*t-2*pi/3);
v_cr = A*sin(2*pi*f*t-4*pi/3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Input signals

%v_ar = A_a*sin(2*pi*f_a*t);
%v_br = A_b*sin(2*pi*f_b*t-2*pi/3);
%v_cr = A_c*sin(2*pi*f_c*t-4*pi/3);

%v_rabc = [v_ar;
%          v_br;
%          v_cr];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Transformation from abc to alpha beta

%T_Clarke = 2/3*[1, -1/2     , -1/2      ;
 %               0, sqrt(3)/2, -sqrt(3)/2];

%v_r = T_Clarke*v_rabc;

v_ralpha = 2/3*(v_ar - 1/2*(v_br + v_cr));
v_rbeta = sqrt(3)/3*(v_br-v_cr);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Space vector angle and magnitude

theta = atan2(v_rbeta, v_ralpha);
for i = 1:length(t)
    if theta(i) < 0
        theta(i) = theta(i) + 2*pi;
    end
end

v_rm = sqrt(v_ralpha.^2+v_rbeta.^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Modulation index

V_s = V_DC/sqrt(3);
M = v_rm/V_s;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Sector calculator

v_sector = zeros(1, n);

for i = 1:length(t)
    if 0 <= theta(i) && theta(i) < pi/3
        sector = 1;
    elseif pi/3 <= theta(i) && theta(i) < 2*pi/3
        sector = 2;
    elseif 2*pi/3 <= theta(i) && theta(i) < 3*pi/3
        sector = 3;
    elseif 3*pi/3 <= theta(i) && theta(i) < 4*pi/3
        sector = 4;
    elseif 4*pi/3 <= theta(i) && theta(i) < 5*pi/3
        sector = 5;
    elseif 5*pi/3 <= theta(i) && theta(i) < 6*pi/3
        sector = 6;
    end
    v_sector(i) = sector;
end

theta_k = theta - (v_sector-1)*pi/3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Dwell Time Calculator

T_1 = T_s*M.*sin(pi/3 - theta_k);
T_2 = T_s*M.*sin(theta_k);
T_0 = T_s - (T_1 + T_2);

%Switch Times

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
%Calculate Space Voltage Vectors

v_0 = 0;
v_1 = [2/3*V_DC*cos(0)     ; 2/3*V_DC*sin(0)     ];
v_2 = [2/3*V_DC*cos(pi/3)  ; 2/3*V_DC*sin(pi/3)  ];
v_3 = [2/3*V_DC*cos(2*pi/3); 2/3*V_DC*sin(2*pi/3)];
v_4 = [2/3*V_DC*cos(3*pi/3); 2/3*V_DC*sin(3*pi/3)];
v_5 = [2/3*V_DC*cos(4*pi/3); 2/3*V_DC*sin(4*pi/3)];
v_6 = [2/3*V_DC*cos(5*pi/3); 2/3*V_DC*sin(5*pi/3)];
v_7 = 0;

v_vv = [v_1;
        v_2;
        v_3;
        v_4;
        v_5;
        v_6];

v_switchstate = [0,0,0;
                 1,0,0;
                 1,1,0;
                 0,1,0;
                 0,1,1;
                 0,0,1;
                 1,0,1;
                 1,1,1];

switchstate = zeros(3,length(t));

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

% DEAD TIME + COMPLEMENTARY SWITCHES
% -------------------------------
dead_time = 2e-6;
dead_time_steps = ceil(dead_time / T_s);

Q1_A = zeros(1, n); Q2_A = zeros(1, n);
Q1_B = zeros(1, n); Q2_B = zeros(1, n);
Q1_C = zeros(1, n); Q2_C = zeros(1, n);

for i = 2:n
    % A phase
    if switchstate(1,i) == 1
        if switchstate(1,i-1) == 0 && i > dead_time_steps
            Q1_A(i) = 0;
        elseif switchstate(1,i-1) == 1
            Q1_A(i) = 1;
        end
    end
    if switchstate(1,i) == 0
        if switchstate(1,i-1) == 1
            Q2_A(i) = 0;
        elseif i > dead_time_steps
            Q2_A(i) = 1;
        end
    end

    % B phase
    if switchstate(2,i) == 1
        if switchstate(2,i-1) == 0 && i > dead_time_steps
            Q1_B(i) = 0;
        elseif switchstate(2,i-1) == 1
            Q1_B(i) = 1;
        end
    end
    if switchstate(2,i) == 0
        if switchstate(2,i-1) == 1
            Q2_B(i) = 0;
        elseif i > dead_time_steps
            Q2_B(i) = 1;
        end
    end

    % C phase
    if switchstate(3,i) == 1
        if switchstate(3,i-1) == 0 && i > dead_time_steps
            Q1_C(i) = 0;
        elseif switchstate(3,i-1) == 1
            Q1_C(i) = 1;
        end
    end
    if switchstate(3,i) == 0
        if switchstate(3,i-1) == 1
            Q2_C(i) = 0;
        elseif i > dead_time_steps
            Q2_C(i) = 1;
        end
    end
end

% -------------------------------
% PLOTS (optional for validation)
% -------------------------------
figure;
subplot(3,1,1);
plot(t, Q1_A, 'r'); hold on; plot(t, Q2_A, 'b');
title('Gate Signals Phase A'); legend('Upper (Q1\_A)', 'Lower (Q2\_A)');

subplot(3,1,2);
plot(t, Q1_B, 'r'); hold on; plot(t, Q2_B, 'b');
title('Gate Signals Phase B'); legend('Upper (Q1\_B)', 'Lower (Q2\_B)');

subplot(3,1,3);
plot(t, Q1_C, 'r'); hold on; plot(t, Q2_C, 'b');
title('Gate Signals Phase C'); legend('Upper (Q1\_C)', 'Lower (Q2\_C)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Figures
figure
plot(t,v_ar)
hold on
plot(t,v_br)
hold on
plot(t,v_cr)

figure
plot(t,v_ralpha)
hold on 
plot(t,v_rbeta)

figure
plot(t,theta)

figure
plot(t,v_sector)

figure
plot(t,theta_k)

figure
plot(t,T_0)
hold on
plot(t,T_1)
hold on
plot(t,T_2)
legend
grid on

figure
plot(t,switchstate(1,:))
%hold on
%plot(t, switchstate(2,:))
%hold on
%plot(t,switchstate(3,:))

%Calculate Effective Vectors
%
%v_e1 = T_1/T_s*v_vv(sector);
%v_e2 = T_2/T_s*v_vv(sector+1);
%v_e0 = T_0/T_s*v_0;

