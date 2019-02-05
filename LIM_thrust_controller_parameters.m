%% Control parameters
Ts = 5e-6;         % s, fundamental sample time
fsw = 2e3;         % Hz, switching frequency 
Tsc = 1/(fsw*50);  % s, control sample time
Tso = 1/(fsw*100); % s, observer sample time

%% Motor Parameters
Pn  = 164e3;     % W, nominal power
Vn  = 550;       % V,  rms phase-to-phase, rated voltage
fn  = 100;        % Hz, rated frequency

Vbase = Vn/sqrt(3)*sqrt(2);  % V, base voltage, peak, line-to-neutral
Ibase = Pn/(1.5*Vbase);  % A, base current, peak
Zbase = Vbase/Ibase;     % ohm, base resistance
wbase = 2*pi*fn;         % rad/s, base elec. radial frequency

m            = 300;             %kg
RR           = 2.7;               % stator resistance
Rs           = 0.01;               % transformed rotor resistance


Ls           = 40.1e-3;
Lr           = 33.1e-3;
Lm           = 32.6e-3;
Lo           = Ls;                % linkage inductance
LM           = Lm;                   % matual inductance
Ty           = Lr/RR;
tau          = 0.2;
p            = 4;
D            = 1;

%b            = 0;                 % pu
w1           = 1; 
%T            = 10*w_base;
psi_ref      = 0.9;
%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Current controller    %
%%%%%%%%%%%%%%%%%%%%%%%%%%
L1           = Lo;
ac           = log(9)/(0.8e-3);
kp           = ac*Lo;
ki           = ac^2*Lo;
Ra           = ac*Lo - RR;
Ed           = 0;
Eq           = 0;
%id_ref       = psi_ref/LM;


% %%%%%%%%%%%%%%%%%%%%%%%%%%
% %     Speed controller   %
% %%%%%%%%%%%%%%%%%%%%%%%%%%
% Smax         = 0.8*0;
% as           = ac/10;
% Kps          = as*J/1;
% Kis          = as^2*J/1;
% ba           = (as*J-b)/1;
% t_step       = T/2;

%%%%%%%%%%%%%%%%%%%%%%%%%%
%     Field Weakening    %
%%%%%%%%%%%%%%%%%%%%%%%%%%
% V_base       = 1;
% I_base       = 1;
Inom         = psi_ref/LM;
Imin         = 0.1*Inom;
Imax         = 4*Ibase;
xi           = (Lo+LM)/Lo;
% t_step2      = T/3;
% Smax2        = 1.8 - Smax;
% t_3          = T*3;


% sim('A4_2_4_2016.mdl');
% t            = wr.time/w_base;
% subplot(3,1,1);
% xlabel(Rr_ifo);
% plot(t,psi_dq0.data); grid on; legend('\psi_d','\psi_q','\psi_0');xlim auto;
% subplot(3,1,2);
% plot(t,id.data,t,iq.data); grid on; legend('id','iq');xlim auto;
% subplot(3,1,3);
% plot(t,wr.data,t,psi.data,t,Is.data); grid on; legend('wr','\psi','Is');xlim auto;
% 
% len = length(psi_dq0.data);
% psi_d = psi_dq0.data(len-1,1)
% psi_q = psi_dq0.data(len-1,2)
% sqrt(psi_d^2 + psi_q^2)
