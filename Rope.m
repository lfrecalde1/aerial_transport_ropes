classdef Rope < matlab.mixin.SetGet
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        n_masses particle;
        number_rope;
        number_mass;
        g;
        spring_l;
        spring_k;
        damper_k;
        wind_drag;
        gravitation;
        ts;
        m;
        fm;
        alpha;
        data;
        N;
        total_force;
        norm_force;

    end
    
    methods
        function obj = Rope(nr, nm, m, fm, L)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.m = m;
            obj.fm = fm;
            obj.number_mass = nm;
            obj.number_rope = nr;
            obj.g = L(1);
            obj.spring_l = L(2);
            obj.spring_k = L(3);
            obj.damper_k = L(4);
            obj.wind_drag = L(5);
            obj.ts = L(6);
            obj.alpha = L(7);
            obj.N = L(8);
            obj.gravitation = [0; 0; obj.g];
            obj.n_masses(1:obj.number_rope,1:obj.number_mass) = particle(m, obj.ts);
            obj.init_ropes();
            obj.set_ropes();
            obj.total_force =   zeros(3, nr);
            obj.norm_force = zeros(1, nr);
            %% Set Data
            obj.data = zeros(3,nm,nr, obj.N);
            for i = 1:obj.number_rope
                for j = 1:obj.number_mass
                    obj.data(:, j, i, 1) = obj.n_masses(i, j).get_pos();
                end
            end
        end
        %% Init Ropes Variables
        function init_ropes(obj)
            for i = 1:obj.number_rope
                for j = 1:obj.number_mass-1
                    obj.n_masses(i, j) = particle(obj.m, obj.ts);
                end
            end
            for i = 1:obj.number_rope
                for j = obj.number_mass
                    obj.n_masses(i, j) = particle(obj.fm, obj.ts);
                end
            end
        end
        %% Function to set the position of each rope element
        function set_ropes(obj)
            for i = 1:obj.number_rope
               for j = 2:obj.number_mass
                   position = obj.n_masses(i, j-1).get_pos();
                   aux = obj.spring_l*[cos(obj.alpha); 0; -sin(obj.alpha)];
                   position = position + aux;
                   obj.n_masses(i, j).set_pos(position)
               end
            end
        end
        
        function system_forces(obj)
            for i = 1:obj.number_rope
                for j = 2:obj.number_mass-1
                    %% Left Size force
                    L_1 = obj.n_masses(i, j).pos - obj.n_masses(i, j-1).pos;
                    r_1 = L_1/norm(L_1,2);
                    if norm(L_1,2) ~= 0
                        s_1 = norm(L_1,2)-obj.spring_l;
                        F_spring_1 = - obj.spring_k*s_1*r_1;
                    else
                        F_spring_1 = [0; 0; 0];
                    end
                    
                    
                    %% External Forces
                    F_wind_1 = - obj.n_masses(i, j).vel*obj.wind_drag;
                    V_1 = obj.n_masses(i, j).vel - obj.n_masses(i, j-1).vel;
                    rp_1 = V_1;
                    F_damper_1 = -obj.damper_k*rp_1;
                    F_grav_1 = -obj.n_masses(i, j).m* obj.gravitation;
                    
                    %% Right Size forces
                    L_2 = obj.n_masses(i, j+1).pos - obj.n_masses(i, j).pos;
                    r_2 = L_2/norm(L_2,2);
                    if norm(L_2,2) ~= 0
                        s_2 = norm(L_2,2)-obj.spring_l;
                        F_spring_2 = - obj.spring_k*s_2*r_2;
                    else
                        F_spring_2 = [0; 0; 0];
                    end
                    
                    V_2 = obj.n_masses(i, j+1).vel - obj.n_masses(i, j).vel;
                    rp_2 = V_2;
                    F_damper_2 = -obj.damper_k*rp_2;
                    
                    force_1 = F_spring_1 + F_grav_1 - F_spring_2 - F_damper_2  + F_wind_1 + F_damper_1;
                    
                    
                    obj.n_masses(i, j).apply_force(force_1);
                    
                    
                end
            end
            for i = 1:obj.number_rope
                for j = obj.number_mass
                    
                    %% Left Size Forces
                    L_2 = obj.n_masses(i, j).pos - obj.n_masses(i, j-1).pos;
                    r_2 = L_2/norm(L_2,2);
                    if norm(L_2,2) ~= 0
                        s_2 = norm(L_2,2)-obj.spring_l;
                        F_spring_2 = -(1*obj.fm/obj.m)*obj.spring_k*s_2*r_2;
                    else
                        F_spring_2 = [0; 0; 0];
                    end
      
                    %% Internal Forces
                    F_wind_2 = -obj.n_masses(i, j).vel*obj.wind_drag;
                    V_2 = obj.n_masses(i, j).vel - obj.n_masses(i, j-1).vel;
                    rp_2 = V_2;
                    F_damper_2 = -(1*obj.fm/obj.m)*obj.damper_k*rp_2;
                    F_grav_2 = -obj.n_masses(i, j).m*obj.gravitation;
                    
                    %% Total Force
                    force_2 = F_grav_2 + F_spring_2 + F_wind_2 + F_damper_2;
                    obj.n_masses(i, j).apply_force(force_2);
                end
            end
             
        end
        
        function f_system_ropes(obj, k)
            %obj.system_forces();
            for i = 1:obj.number_rope
                for j = 1:obj.number_mass
                    obj.n_masses(i, j).f_system();
                    obj.data(:, j, i, k) = obj.n_masses(i, j).get_pos();
                end
            end
            
        end
        
        function [x, x_norm] = get_total_force(obj)
            for i = 1:obj.number_rope
                aux_rope = [0;0;0];
                for j = 1:obj.number_mass
                    aux_rope = aux_rope + obj.n_masses(i, j).get_internal_force();
                end
                aux_rope = aux_rope + (-obj.n_masses(i, j).m*obj.gravitation);
                aux_rope(1) = -aux_rope(1);
                aux_rope(2) = -aux_rope(2);
                obj.total_force(:, i) = aux_rope;
                obj.norm_force(1, i) = norm(aux_rope, 2);
            end
            x = obj.total_force;
            x_norm = obj.norm_force;
        end
        
        function apply_Velocity(obj,Velocity_systems)
            for i = 1:obj.number_rope
                obj.n_masses(i, 1).set_vel(Velocity_systems(:,i))
            end
        end
        
        
    end
end


