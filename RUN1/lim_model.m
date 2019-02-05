% Single-Sided Linear Indution Motor Model using MATLAB/S-Function

% State Variables x(1) = psi_sa, x(2) = psi_sb, x(3) = psi_ra, x(4) = psi_rb, x(5) = v;
% Inputs Variables u(1) = u_sa, u(2) = u_sb, u(3) = F_load;

function[sys,x0] = lim_model(~,x,u,flag)
switch flag
  case 0
      sys = [5;0;7;3;1;1];
      x0 = [0;0;0;0;0];
  case 1
      %Parameters
      tau=0.2;
      p = 4;
      D = 4;
      V_base=250; %V
      I_base=200; %A
      Z_base=V_base/I_base;
      w_base=2*pi*50; %rad/sec
      L_base=Z_base/w_base;
      v_base=tau*w_base/pi;
      psi_base=I_base*L_base;
      Q_base=Z_base/(L_base*v_base);
      F_base=V_base*I_base/w_base;
      m_base=F_base/(v_base*w_base);
      A_base=L_base^2;
      %normalized values
      m = 5/m_base;
      Rs = 0.01/Z_base;
      Rr = 2.7/Z_base;
      Ls = 40.1e-3/L_base;
      Lr = 33.1e-3/L_base;
      Lm = 32.6e-3/L_base;
      %Current calculation
      Q = abs(D*Rr/(Lr*x(5)));
      f =(1-exp(-Q*Q_base))/(Q*Q_base);
      %f=0;
      A = (Ls-Lm*f)*(Lr-Lm*f)-(Lm)^2*(1-f)^2;
      i_sa = ((Lr-Lm*f)*x(1)-Lm*(1-f)*x(3))/A;
      i_sb = ((Lr-Lm*f)*x(2)-Lm*(1-f)*x(4))/A;
      i_ra = -(Lm*(1-f)*x(1)-(Ls-Lm*f)*x(3))/A;
      i_rb = -(Lm*(1-f)*x(2)-(Ls-Lm*f)*x(4))/A;
      F = 3*pi*p/(2*tau*2)*(x(1)*i_sb-x(2)*i_sa);
      %Detervatives calculation
      dx(1) = (u(1)-Rs*i_sa-Rr*f*(i_sa+i_ra));
      dx(2) = (u(2)-Rs*i_sb);
      dx(3) = (-Rr*i_ra-Rr*f*(i_sa+i_ra)-x(5)*x(4)); %Is x(5) linear velocity or angular velocity?
      dx(4) = (-Rr*i_rb+x(5)*x(3));
      dx(5) = ((F-u(3))/(m));
      sys = dx;
  case 3
      %Parameters
      tau=0.2;
      p = 4;
      D =1;
      V_base=250; %V
      I_base=200; %A
      Z_base=V_base/I_base;
      w_base=2*pi*50; %rad/sec
      L_base=Z_base/w_base;
      v_base=tau*w_base/pi;
      psi_base=V_base/w_base;
      Q_base=Z_base/(L_base*v_base);
      F_base=V_base*I_base/w_base;
      m_base=F_base/(v_base*w_base);
      A_base=L_base^2;
      %normalized values
      m = 5/m_base;
      Rs = 0.01/Z_base;
      Rr = 2.7/Z_base;
      Ls = 40.1e-3/L_base;
      Lr = 33.1e-3/L_base;
      Lm = 32.6e-3/L_base;
      %Current calculation
      Q = abs(D*Rr/(Lr*x(5)));
      %f =(1-exp(-Q*Q_base))/(Q*Q_base);
      f=0;
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
