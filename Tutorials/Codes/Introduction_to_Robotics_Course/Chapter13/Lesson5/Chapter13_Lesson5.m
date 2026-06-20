% True parameters
Jt=0.02; bt=0.1; Ktt=0.05; Ket=0.05; Rt=2.0; Lt=0.5;
% Twin guesses
Jh=0.03; bh=0.2; Kth=0.04; Keh=Ket; Rh=Rt; Lh=Lt;

dt=0.01; T=5; N=T/dt;
t=(0:N-1)*dt;
V=6*(t>0.5);

xTrue=zeros(2,N);
xHat=zeros(2,N);
yMeas=zeros(1,N);
Lobs=[30;5];

% Dynamics function
fMotor=@(x,V,J,b,Kt,Ke,R,L)[(-b/J)*x(1)+(Kt/J)*x(2);
                           (-Ke/L)*x(1)-(R/L)*x(2)+(1/L)*V];

for k=1:N-1
    xTrue(:,k+1)=xTrue(:,k)+dt*fMotor(xTrue(:,k),V(k),Jt,bt,Ktt,Ket,Rt,Lt);
    yMeas(k)=xTrue(1,k)+0.02*randn;

    xHat(:,k+1)=xHat(:,k)+dt*fMotor(xHat(:,k),V(k),Jh,bh,Kth,Keh,Rh,Lh);
    r=yMeas(k)-xHat(1,k);
    xHat(:,k+1)=xHat(:,k+1)+dt*Lobs*r;
end

omega=yMeas;
iHat=xHat(2,:);
domega=diff(omega)/dt;
Phi=[-omega(1:end-1)'  iHat(1:end-1)'];
thetaLS=(Phi'*Phi)\(Phi'*domega');

theta1=thetaLS(1); theta2=thetaLS(2);
disp(['theta1=b/J estimate: ', num2str(theta1)])
disp(['theta2=Kt/J estimate: ', num2str(theta2)])

bh=theta1*Jh; Kth=theta2*Jh; %#ok<NASGU>
      
