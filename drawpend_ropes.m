function drawpend_ropes(Matrix_ropes, k)
plot3([0 0],[0 0], [0 0],'k','LineWidth',2); hold on;
for i = 1:Matrix_ropes.number_rope
   for j = 1:Matrix_ropes.number_mass-1
       mr_2 =1*sqrt(Matrix_ropes.n_masses(i, j).m);  % mass radius
       mr_3 =1*sqrt(Matrix_ropes.n_masses(i, j+1).m);  % mass radius
       
       pendx_1 =   Matrix_ropes.data(1, j, i, k);
       pendy_1 =   Matrix_ropes.data(2, j, i, k);
       pendz_1 =   Matrix_ropes.data(3, j, i, k);
       
       pendx_2 =   Matrix_ropes.data(1, j+1, i, k);
       pendy_2 =   Matrix_ropes.data(2, j+1, i, k);
       pendz_2 =   Matrix_ropes.data(3, j+1, i, k);
       
      
      
       %rectangle('Position',[pendx_1-mr_2/2,pendz_1-mr_2/2,mr_2,mr_2],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
       
       plot3([pendx_1 pendx_2],[pendy_1 pendy_2],[pendz_1 pendz_2],'.-','Color',[56,171,217]/255,'linewidth',5*mr_2); % Draw pendulum
       
   end
end

for i = 1:Matrix_ropes.number_rope
   for j = Matrix_ropes.number_mass
       mr_2 =1*sqrt(Matrix_ropes.n_masses(i, j).m);  % mass radius
      
       
       pendx_1 =   Matrix_ropes.data(1, j, i, k);
       pendy_1 =   Matrix_ropes.data(2, j, i, k);
       pendz_1 =   Matrix_ropes.data(3, j, i, k);
       
       plot3(pendx_1,pendy_1,pendz_1,'o','Color',[100,100,100]/255,'linewidth',5*mr_2); % Draw pendulum
       
       
   end
end
title('$\textrm{Movement Executed by the Aerial Robots}$','Interpreter','latex','FontSize',11);
xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
axis([-3.5 3.5 -3.5 3.5 -3.5 3.5]);
grid minor;
% hold off;