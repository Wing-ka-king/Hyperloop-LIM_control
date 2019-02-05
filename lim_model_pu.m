% Single-Sided Linear Indution Motor Model using MATLAB/S-Function

% State Variables x(1) = psi_sa, x(2) = psi_sb, x(3) = psi_ra, x(4) = psi_rb, x(5) = v;
% Inputs Variables u(1) = u_sa, u(2) = u_sb, u(3) = F_load;

function[sys,x0] = lim_model_pu(~,x,u,flag)
switch flag
  case 0
      sys = [5;0;7;3;1;1];
      x0 = [0;0;0;0;0];
  case 1
      %Parameters
      Vn = 250; %V, phase to phase
      Pn = 1e5; %W, rated power %%%modified%%%
      fn = 70; %Hz, rated frequency %%%modified%%%
      tau=0.2;
      p = 4;
      D = 1;    %meter
      Vbase = Vn/sqrt(3)*sqrt(2); %V, base voltage (peak) line to neutral %%%modified%%%
      Ibase = Pn/(1.5*Vbase); %A, base current(peak) %%%modified%%%
      Zbase = Vbase/Ibase; %ohm, base resistance
      wbase = 2*pi*fn; %rad/s, base frequency
      v_base= tau*wbase/pi; %m/s, base speed
      Fbase = Pn/(v_base/p); %N, base force %%%modified%%%
      
      L_base=Zbase/wbase;
      psi_base=Ibase*L_base;
      Q_base=Zbase/(L_base*v_base);

      m_base=Fbase/(v_base*wbase);
      A_base=L_base^2;
      %normalized values
      m = 68/m_base; %pu    %%%modified%%%
      Rs = 0.01/Zbase; %pu
      Rr = 2.7/Zbase; %pu
      Ls = 40.1e-3/L_base; %pu
      Lr = 33.1e-3/L_base;  %pu
      Lm = 32.6e-3/L_base;  %pu
      %Current calculation
      Q = abs(D*Rr/(Lr*x(5)));
      f =(1-exp(-Q*Q_base))/(Q*Q_base);
      A = (Ls-Lm*f)*(Lr-Lm*f)-(Lm)^2*(1-f)^2;
      i_sa = ((Lr-Lm*f)*x(1)-Lm*(1-f)*x(3))/A;
      i_sb = ((Lr-Lm*f)*x(2)-Lm*(1-f)*x(4))/A;
      i_ra = -(Lm*(1-f)*x(1)-(Ls-Lm*f)*x(3))/A;
      i_rb = -(Lm*(1-f)*x(2)-(Ls-Lm*f)*x(4))/A;
      F = 3*pi*p/(2*tau*2)*(x(1)*i_sb-x(2)*i_sa);
      %Detervatives calculation
      dx(1) = (u(1)-Rs*i_sa-Rr*f*(i_sa+i_ra));
      dx(2) = (u(2)-Rs*i_sb)-Rr*f*(i_sb+i_rb);
      dx(3) = (-Rr*i_ra-Rr*f*(i_sa+i_ra)-x(5)*x(4)); %Is x(5) linear velocity or angular velocity?
      dx(4) = (-Rr*i_rb+x(5)*x(3))-Rr*f*(i_sb+i_rb);
      dx(5) = ((F-u(3))/(m));
      sys = dx;
      
  case 3
      %Parameters
      Vn = 250; %V, phase to phase
      Pn = 1e5; %W, rated power %%%modified%%%
      fn = 70; %Hz, rated frequency %%%modified%%%
      tau=0.2;
      p = 4;
      D = 1;    %meter
      Vbase = Vn/sqrt(3)*sqrt(2); %V, base voltage (peak) line to neutral %%%modified%%%
      Ibase = Pn/(1.5*Vbase); %A, base current(peak) %%%modified%%%
      Zbase = Vbase/Ibase; %ohm, base resistance
      wbase = 2*pi*fn; %rad/s, base frequency
      v_base= tau*wbase/pi; %m/s, base speed
      Fbase = Pn/(v_base/p); %N, base force %%%modified%%%
      
      L_base=Zbase/wbase;
      psi_base=Ibase*L_base;
      Q_base=Zbase/(L_base*v_base);

      m_base=Fbase/(v_base*wbase);
      A_base=L_base^2;
      %normalized values
      m = 68/m_base; %pu    %%%modified%%%
      Rs = 0.01/Zbase; %pu
      Rr = 2.7/Zbase; %pu
      Ls = 40.1e-3/L_base; %pu
      Lr = 33.1e-3/L_base;  %pu
      Lm = 32.6e-3/L_base;  %pu
      %Current calculation
      Q = abs(D*Rr/(Lr*x(5)));
      f =(1-exp(-Q*Q_base))/(Q*Q_base);
      A = (Ls-Lm*f)*(Lr-Lm*f)-(Lm)^2*(1-f)^2;
      i_sa = ((Lr-Lm*f)*x(1)-Lm*(1-f)*x(3))/A;
      i_sb = ((Lr-Lm*f)*x(2)-Lm*(1-f)*x(4))/A;
      F = 3*pi*p/(2*tau*2)*(x(1)*i_sb-x(2)*i_sa);
      sys = [x(1); x(2); i_sa; i_sb; F; x(5); f];
  case {2, 4, 9}
    sys = [];
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
