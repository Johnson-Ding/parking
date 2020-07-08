figure(1);
set(gcf,'color','w');
subplot(2,2,1);
plot(0:optimalT:N*optimalT,optimalState(4,:));
set(gca,'XTick',0:2:10,'color','w');
set(gca,'YTick',-2:0.5:2,'color','w');
grid on
xlabel('$t/s$','Interpreter','latex');
ylabel('$v/m\cdot s^{-1}$','Interpreter','latex');    
set(gca,'FontSize',10,'FontName','Times NewRoman','GridLineStyle','--','LineWidth',0.3);
subplot(2,2,2);
plot(0:optimalT:N*optimalT,optimalInput(1,:));
set(gca,'XTick',0:2:10,'color','w');
set(gca,'YTick',-5:1:5,'color','w');
grid on
xlabel('$t/s$','Interpreter','latex');
ylabel('$a/m\cdot s^{-2}$','Interpreter','latex');    
set(gca,'FontSize',10,'FontName','Times NewRoman','GridLineStyle','--','LineWidth',0.3);
subplot(2,2,3);
plot(0:optimalT:N*optimalT,optimalState(5,:));
set(gca,'XTick',0:2:10,'color','w');
set(gca,'YTick',-0.2:0.05:0.2,'color','w');
grid on
xlabel('$t/s$','Interpreter','latex');
ylabel('$\kappa/m^{-1}$','Interpreter','latex');    
set(gca,'FontSize',10,'FontName','Times NewRoman','GridLineStyle','--','LineWidth',0.3);
subplot(2,2,4);
plot(0:optimalT:N*optimalT,optimalInput(2,:));
set(gca,'XTick',0:2:10,'color','w');
set(gca,'YTick',-1.5:0.5:1.5,'color','w');
grid on
xlabel('$t/s$','Interpreter','latex');
ylabel('$\sigma/m^{-1}\cdot s^{-1}$','Interpreter','latex');    
set(gca,'FontSize',10,'FontName','Times NewRoman','GridLineStyle','--','LineWidth',0.3);
