pi = 3.1415;
T_s = XX;

//When Nucleo recieves new speed signal it trips an interrupt
	//The analog speed signal is converted to a digital variable N
f = 4*N/120;
U_RL = f*8;
U_PP = U_RL*sqrt(2)/sqrt(3);

//An interrupt samples a variable t and V_DC and generates PWM switching signals
	//t = ReadTimer(2)/10^6; - This might change depending on how a timer can be scaled
s_a = U_PP*sin(2*pi*f*t);
s_b = U_PP*sin(2*pi*f*t-2*pi/3);
s_c = U_PP*sin(2*pi*f*t+2*pi/3);

//Clarke transformation
s_alpha = 2/3*(s_a - 1/2*(s_b + s_c));
s_beta = sqrt(3)/3*(s_b - s_c);

//Space Vector Coordinates Calculator
SV_theta = atan2(s_beta, s_alpha);
if (SV_theta < 0)
{
	SV_theta = SV_theta + 2*pi;
}

SV_m = sqrt(s_alpha^2+s_beta^2);

//Modulation Index Calculator
M = SV_m/(V_DC/sqrt(3));

//Sector Calculator
if (0 <= SV_theta && SV_theta < pi/3)
{
	sector = 1;
}
else if (pi/3 <= SV_theta && SV_theta < 2*pi/3)
{
	sector = 2;
}
else if (2*pi/3 <= SV_theta && SV_theta < 3*pi/3)
{
	sector = 3;
}
else if (3*pi/3 <= SV_theta && SV_theta < 4*pi/3)
{
	sector = 4;
}
else if (4*pi/3 <= SV_theta && SV_theta < 5*pi/3)
{
	sector = 5;
}
else if (5*pi/3 <= SV_theta && SV_theta < 6*pi/3)
{
	sector = 6;
}

sector_theta = SV_theta - (sector-1)*pi/3;

//Dwell Time Calculator
T_1 = T_s*M*sin(pi/3 - sector_theta);
T_2 = T_s*M*sin(sector_theta);
T_0 = T_s - (T_1 + T_2);

//Switching Sequence Generator
	//Switch Times
/* Might not be needed
if (sector == 1 || sector == 3 || sector == 5)
{
	t_s0 = T_0/2;
	t_s1 = t_s0+T_1;
    t_s2 = t_s1+T_2;
    t_s3 = t_s2+T_0;
    t_s4 = t_s3+T_2;
    t_s5 = t_s4+T_1;
    t_s6 = t_s5+T_0/2;
}
else
{
	t_s0 = T_0/2;
    t_s1 = t_s0+T_2;
    t_s2 = t_s1+T_1;
    t_s3 = t_s2+T_0;
    t_s4 = t_s3+T_1;
    t_s5 = t_s4+T_2;
    t_s6 = t_s5+T_0/2;
}
*/
/*General Switchstate Vector

v_switchstate = [0,0,0;
                 1,0,0;
                 1,1,0;
                 0,1,0;
                 0,1,1;
                 0,0,1;
                 1,0,1;
                 1,1,1];
*/

if (sector == 1)
{
	DC_a = (2*(T_1 + T_2) + 2*T_0)/(2*T_s) * 100;
	DC_b = (2*T_2 + T_0)/(2*T_s) * 100;
	DC_c = T_0/(2*T_s) * 100;
}
else if (sector == 2)
{
	DC_a = (2*T_1 + T_0)/(2*T_s) * 100;
	DC_b = (2*(T_1 + T_2) + 2*T_0)/(2*T_s) * 100;
	DC_c = T_0/(2*T_s) * 100;
}
else if (sector == 3)
{
	DC_a = T_0/(2*T_s) * 100;
	DC_b = (2*(T_1 + T_2) + 2*T_0)/(2*T_s) * 100;
	DC_c = (2*T_2 + T_0)/(2*T_s) * 100;
}
else if (sector == 4)
{
	DC_a = T_0/(2*T_s) * 100;
	DC_b = (2*T_1 + T_0)/(2*T_s) * 100;
	DC_c = (2*(T_1 + T_2) + 2*T_0)/(2*T_s) * 100;
}
else if (sector == 5)
{
	DC_a = (2*T_1 + T_0)/(2*T_s) * 100;
	DC_b = T_0/(2*T_s) * 100;
	DC_c = (2*(T_1 + T_2) + 2*T_0)/(2*T_s) * 100;
}
else if (sector == 6)
{
	DC_a = (2*(T_1 + T_2) + 2*T_0)/(2*T_s) * 100;
	DC_b = T_0/(2*T_s) * 100;
	DC_c = (2*T_1 + T_0)/(2*T_s) * 100;
}
