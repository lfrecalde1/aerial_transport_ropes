function drawpend_ropes(Matrix_ropes, k)
plot([0 0],[0 0],'k','LineWidth',2), hold on
for i = 1:Matrix_ropes.number_rope
   for j = 1:Matrix_ropes.number_mass-1
       mr_2 =0.3*sqrt(Matrix_ropes.n_masses(i, j).m);  % mass radius
       mr_3 =0.3*sqrt(Matrix_ropes.n_masses(i, j+1).m);  % mass radius
       
       pendx_1 =   Matrix_ropes.data(1, j, i, k);
       pendy_1 =   Matrix_ropes.data(2, j, i, k);
       
       pendx_2 =   Matrix_ropes.data(1, j+1, i, k);
       pendy_2 =   Matrix_ropes.data(2, j+1, i, k);
       
      
      
       rectangle('Position',[pendx_1-mr_2/2,pendy_1-mr_2/2,mr_2,mr_2],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
       
       plot([pendx_1 pendx_2],[pendy_1 pendy_2],'k','LineWidth',2); % Draw pendulum
       
   end
end

for i = 1:Matrix_ropes.number_rope
   for j = Matrix_ropes.number_mass
       mr_2 =0.3*sqrt(Matrix_ropes.n_masses(i, j).m);  % mass radius
      
       
       pendx_1 =   Matrix_ropes.data(1, j, i, k);
       pendy_1 =   Matrix_ropes.data(2, j, i, k);
       
       rectangle('Position',[pendx_1-mr_2/2,pendy_1-mr_2/2,mr_2,mr_2],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
       
       
   end
end
% dimensions


% positions

axis([-1 1 -2 0.5]);
axis equal
grid on;
%set(gcf,'Position',[100 100 1000 800])
drawnow, hold off