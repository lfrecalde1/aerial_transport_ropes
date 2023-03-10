classdef Ropes_general < matlab.mixin.SetGet
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        n_masses particle;
        final_mass  particle;
        number_rope;
        number_mass;
        number_mass_total;
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
        data_final;
        N;
        total_force;
        norm_force;
        fm_pose;

    end
    
    methods
        function obj = Ropes_general(nr, nm, m, fm, L, fm_pose)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.m = m;
            obj.fm = fm;
            obj.number_mass = nm;
            obj.number_mass_total = obj.number_mass + 1;
            obj.number_rope = nr;
            obj.g = L(1);
            obj.spring_l = L(2);
            obj.spring_k = L(3);
            obj.damper_k = L(4);
            obj.wind_drag = L(5);
            obj.ts = L(6);
            obj.alpha = L(7);
            obj.N = L(8);
            obj.fm_pose = fm_pose;
            obj.gravitation = [0; 0; obj.g];
            
            %% Initialization of the Matrix of ropes and the final mass
            obj.n_masses(1:obj.number_rope,1:obj.number_mass) = particle(m, obj.ts);
            obj.final_mass = particle(fm, obj.ts);
            
            %% Initialization Ropes
            obj.init_ropes();
            obj.set_ropes();
             
            %% Vector of the forces
            obj.total_force =   zeros(3, nr);
            obj.norm_force = zeros(nr, 1);
            
            %% Set Data
            obj.data = zeros(3, obj.number_mass, obj.number_rope, obj.N);
            obj.data_final = zeros(3, obj.N);
            
            %% Set Data to all masses
            for i = 1:obj.number_rope
                for j = 1:obj.number_mass
                    obj.data(:, j, i, 1) = obj.n_masses(i, j).get_pos();
                end
            end
            %% Set Final Mass Data
            obj.data_final(:, 1) = obj.final_mass.get_pos();
        end
        %% Init Ropes Variables
        function init_ropes(obj)
            for i = 1:obj.number_rope
                for j = 1:obj.number_mass
                    obj.n_masses(i, j) = particle(obj.m, obj.ts);
                end
            end
            obj.final_mass = particle(obj.fm, obj.ts);
        end
        %% Function to set the position of each rope element
        function set_ropes(obj)
            %% Set Position of the final mass in the rope
            position = obj.fm_pose;
            obj.final_mass.set_pos(position)
            
            %% Set position to the internal masses in the rope
            for i = 1:obj.number_rope
                for j = obj.number_mass:-1:1
                    if j == obj.number_mass
                        position = obj.final_mass.get_pos();
                        aux = obj.ropes_initial_angles(i);
                        position = position + aux;
                        obj.n_masses(i, j).set_pos(position)
                    else
                        position = obj.n_masses(i, j+1).get_pos();
                        aux = obj.ropes_initial_angles(i);
                        position = position + aux;
                        obj.n_masses(i, j).set_pos(position)
                    end
                    
                end
            end
            
        end
        
        function system_forces(obj)
            %% Force of the load
            aux_final_load = zeros(3, obj.number_rope);
            for i = 1:obj.number_rope
                %% Internal forces spring
                L = obj.n_masses(i, obj.number_mass).pos - obj.final_mass.pos;
                r = L/norm(L,2);
                if norm(L,2) ~= 0
                    s = norm(L,2)-obj.spring_l;
                    F_spring = obj.spring_k*s*r;
                else
                    F_spring = [0; 0; 0];
                end
                
                %% Internal damping
                V = obj.n_masses(i, obj.number_mass).vel -obj.final_mass.vel;
                rp = V;
                F_damper = obj.damper_k*rp;
                
                %% Total Force
                force = F_spring + F_damper;
                aux_final_load(:, i) = force;

            end
            
            %% External Forces
            F_wind = -obj.final_mass.vel*obj.wind_drag;
            F_grav = -obj.final_mass.m*obj.gravitation;
            
            %% Total Force
            force_load = F_grav + F_wind +  sum(aux_final_load, 2);
            obj.final_mass.apply_force(force_load);
            
            %% Auxiliar variables internal forces
            aux_internal_forces = zeros(3, obj.number_mass, obj.number_rope);
            
            for i = 1:obj.number_rope
                %% Init aux variable get total internal foorce in each rope
                aux_rope_internal = [0;0;0];
                for j = obj.number_mass:-1:2
                    %% It is just for the first element
                    if j == obj.number_mass
                        F_down = aux_final_load(:,i);
                    else
                        %% Down Side Memories
                          F_down = aux_internal_forces(:,j+1,i);
                        
                    end
                    %% Upper side
                    L_up = obj.n_masses(i, j-1).pos - obj.n_masses(i, j).pos;
                    r_up = L_up/norm(L_up,2);
                    if norm(L_up,2) ~= 0
                        s_up = norm(L_up,2)-obj.spring_l;
                        F_spring_up = obj.spring_k*s_up*r_up;
                    else
                        F_spring_up = [0; 0; 0];
                    end
                    V_up = obj.n_masses(i, j-1).vel - obj.n_masses(i, j).vel;
                    rp_up = V_up;
                    F_damper_up = obj.damper_k*rp_up;
                    aux_internal_forces(:, j, i) = F_damper_up + F_spring_up;
                    
                    %% External Forces
                    F_wind = - obj.n_masses(i, j).vel*obj.wind_drag;
                    F_grav = -obj.n_masses(i, j).m* obj.gravitation;
                    
                    force = F_spring_up + F_damper_up - F_down + F_wind + F_grav;
                    aux_rope_internal = aux_rope_internal + force;
                    obj.n_masses(i, j).apply_force(force);
                end
                %% Information first Link each Rope
                first_link = aux_internal_forces(:,2,i);
                obj.total_force(:, i) = aux_rope_internal - first_link;    
            end
            
            %% Total force of the system; Add the last element?
            obj.total_force(:,:) = obj.total_force(:,:) + force_load;
            for i = 1:obj.number_rope
                obj.norm_force(i, 1) = norm(obj.total_force(:, i), 2);
            end
        end
        
        %% Evolution system
        function f_system_ropes(obj, k)
            for i = 1:obj.number_rope
                for j = 1:obj.number_mass
                    obj.n_masses(i, j).f_system();
                    obj.data(:, j, i, k) = obj.n_masses(i, j).get_pos();
                end
            end
            obj.final_mass.f_system();
            obj.data_final(:, k) = obj.final_mass.get_pos();
            
        end
        
        
        %% Apply velocity function
        function apply_Velocity(obj,Velocity_systems)
            for i = 1:obj.number_rope
                obj.n_masses(i, 1).set_vel(Velocity_systems(:,i))
            end
        end
        
        %% Get initial position and velocity of the first element
        function [x, xp]= get_position_initial(obj)
            x = [];
            xp = [];
           for i = 1:obj.number_rope
                x = [x;obj.n_masses(i, 1).get_pos()'];
                xp = [xp;obj.n_masses(i, 1).get_pos()'];
           end
        end
        
        %% Function for positioning of the rope
        function x = ropes_initial_angles(obj, k)
            if k ==1
                
                x = obj.spring_l*[-cos(obj.alpha); 0; sin(obj.alpha)];
                
            else
                x = obj.spring_l*[+cos(obj.alpha); 0; sin(obj.alpha)];
                
            end
            
        end
        function [x, x_norm] = get_total_force(obj)
           x = obj.total_force;
           x_norm = obj.norm_force;
        end
        
    end
end


