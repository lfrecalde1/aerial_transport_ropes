classdef Multiple_Ropes < matlab.mixin.SetGet
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
        function obj = Multiple_Ropes(nr, nm, m, fm, L, fm_pose)
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
             
            %% Vector of the forces in
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
            for i = 1:obj.number_rope
                for j = 2:obj.number_mass
                    %% Left Size forces
                    L_1 = obj.n_masses(i, j-1).pos - obj.n_masses(i, j).pos;
                    r_1 = L_1/norm(L_1,2);
                    if norm(L_1,2) ~= 0
                        s_1 = norm(L_1,2)-obj.spring_l;
                        F_spring_1 =  obj.spring_k*s_1*r_1;
                    else
                        F_spring_1 = [0; 0; 0];
                    end

                    %% External Forces
                    F_wind_1 = - obj.n_masses(i, j).vel*obj.wind_drag;
                    V_1 = obj.n_masses(i, j-1).vel - obj.n_masses(i, j).vel;
                    rp_1 = V_1;
                    F_damper_1 = obj.damper_k*rp_1;
                    F_grav_1 = -obj.n_masses(i, j).m* obj.gravitation;
                    
                    if j == obj.number_mass
                        %% Right Size forces
                        L_2 = obj.n_masses(i, j).pos - obj.final_mass.pos;
                        r_2 = L_2/norm(L_2,2);
                        if norm(L_2,2) ~= 0
                            s_2 = norm(L_2,2)-obj.spring_l;
                            F_spring_2 =  obj.spring_k*s_2*r_2;
                        else
                            F_spring_2 = [0; 0; 0];
                        end
                        V_2 =  obj.n_masses(i, j).vel - obj.final_mass.vel;
                        rp_2 = V_2;
                        F_damper_2 = obj.damper_k*rp_2;
                        
                    else
                        %% Right Size forces
                        L_2 = obj.n_masses(i, j).pos - obj.n_masses(i, j+1).pos;
                        r_2 = L_2/norm(L_2,2);
                        if norm(L_2,2) ~= 0
                            s_2 = norm(L_2,2)-obj.spring_l;
                            F_spring_2 =  obj.spring_k*s_2*r_2;
                        else
                            F_spring_2 = [0; 0; 0];
                        end
                        
                        V_2 = obj.n_masses(i, j).vel - obj.n_masses(i, j+1).vel;
                        rp_2 = V_2;
                        F_damper_2 = obj.damper_k*rp_2;
                        
                    end
                    
                    
                    force_1 = F_spring_1 + F_grav_1 - F_spring_2 - F_damper_2  + F_wind_1 + F_damper_1;
                    
                    
                    obj.n_masses(i, j).apply_force(force_1);
                    
                    
                end
            end
            
            
            for i = 1:obj.number_rope
                L_2 = obj.n_masses(i, obj.number_mass).pos - obj.final_mass.pos;
                r_2 = L_2/norm(L_2,2);
                if norm(L_2,2) ~= 0
                    s_2 = norm(L_2,2)-obj.spring_l;
                    F_spring_2 = obj.spring_k*s_2*r_2;
                else
                    F_spring_2 = [0; 0; 0];
                end
                
                %% Internal Forces
                F_wind_2 = -obj.final_mass.vel*obj.wind_drag/obj.number_rope;
                V_2 = obj.n_masses(i, obj.number_mass).vel -obj.final_mass.vel;
                rp_2 = V_2;
                F_damper_2 = obj.damper_k*rp_2;
                F_grav_2 = (-obj.final_mass.m*obj.gravitation)/obj.number_rope;
                
                %% Total Force
                force_2 = F_grav_2 + F_spring_2 + F_wind_2 + F_damper_2;
                obj.final_mass.apply_force(force_2);
                
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
            obj.final_mass.f_system();
            obj.data_final(:, k) = obj.final_mass.get_pos();
            
        end
        
        function [x, x_norm] = get_total_force(obj)
            
            for i = 1:obj.number_rope
                aux_rope_internal = [0;0;0];
                for j = 2:obj.number_mass
                    %% Left Size forces
                    L_1 = obj.n_masses(i, j-1).pos - obj.n_masses(i, j).pos;
                    r_1 = L_1/norm(L_1,2);
                    if norm(L_1,2) ~= 0
                        s_1 = norm(L_1,2)-obj.spring_l;
                        F_spring_1 =  obj.spring_k*s_1*r_1;
                    else
                        F_spring_1 = [0; 0; 0];
                    end
                    
                    %% External Forces
                    F_wind_1 = - obj.n_masses(i, j).vel*obj.wind_drag;
                    V_1 = obj.n_masses(i, j-1).vel - obj.n_masses(i, j).vel;
                    rp_1 = V_1;
                    F_damper_1 = obj.damper_k*rp_1;
                    F_grav_1 = -obj.n_masses(i, j).m* obj.gravitation;
                    
                    if j == obj.number_mass
                        %% Right Size forces
                        L_2 = obj.n_masses(i, j).pos - obj.final_mass.pos;
                        r_2 = L_2/norm(L_2,2);
                        if norm(L_2,2) ~= 0
                            s_2 = norm(L_2,2)-obj.spring_l;
                            F_spring_2 =  obj.spring_k*s_2*r_2;
                        else
                            F_spring_2 = [0; 0; 0];
                        end
                        
                        V_2 =  obj.n_masses(i, j).vel - obj.final_mass.vel;
                        rp_2 = V_2;
                        F_damper_2 = obj.damper_k*rp_2;
                        
                    else
                        %% Right Size forces
                        L_2 = obj.n_masses(i, j).pos - obj.n_masses(i, j+1).pos;
                        r_2 = L_2/norm(L_2,2);
                        if norm(L_2,2) ~= 0
                            s_2 = norm(L_2,2)-obj.spring_l;
                            F_spring_2 =  obj.spring_k*s_2*r_2;
                        else
                            F_spring_2 = [0; 0; 0];
                        end
                        
                        V_2 = obj.n_masses(i, j).vel - obj.n_masses(i, j+1).vel;
                        rp_2 = V_2;
                        F_damper_2 = obj.damper_k*rp_2;
                        
                    end
                    
                    force_1 = F_spring_1 + F_grav_1 - F_spring_2 - F_damper_2  + F_wind_1 + F_damper_1;
                    aux_rope_internal = aux_rope_internal + force_1;
                    
                end
                %% Auxiliar link 1
                L_1 = obj.n_masses(i, 1).pos - obj.n_masses(i, 2).pos;
                r_1 = L_1/norm(L_1,2);
                if norm(L_1,2) ~= 0
                    s_1 = norm(L_1,2)-obj.spring_l;
                    F_spring_1 =  obj.spring_k*s_1*r_1;
                else
                    F_spring_1 = [0; 0; 0];
                end
                
                obj.total_force(:, i) = aux_rope_internal - F_spring_1;
            end

            
            for i = 1:obj.number_rope
                
                L_2 = obj.n_masses(1, obj.number_mass).pos - obj.final_mass.pos;
                r_2 = L_2/norm(L_2,2);
                if norm(L_2,2) ~= 0
                    s_2 = norm(L_2,2)-obj.spring_l;
                    F_spring_2 = obj.spring_k*s_2*r_2;
                else
                    F_spring_2 = [0; 0; 0];
                end
                
                L_3 = obj.n_masses(2, obj.number_mass).pos - obj.final_mass.pos;
                r_3 = L_3/norm(L_3,2);
                
                if norm(L_3,2) ~= 0
                    s_3 = norm(L_3,2)-obj.spring_l;
                    F_spring_3 = (obj.spring_k*s_3*r_3);
                else
                    F_spring_3 = [0; 0; 0];
                end
                
                %% Internal Forces
                F_wind_2 = -obj.final_mass.vel*obj.wind_drag;
                
                V_2 = obj.n_masses(1, obj.number_mass).vel -obj.final_mass.vel;
                rp_2 = V_2;
                F_damper_2 = obj.damper_k*rp_2;
                
                V_3 = obj.n_masses(2, obj.number_mass).vel -obj.final_mass.vel;
                rp_3 = V_3;
                F_damper_3 = obj.damper_k*rp_3;
                
                F_grav_2 = (-obj.final_mass.m*obj.gravitation);
                
                %% Total Force
                force_2 = F_grav_2 + F_spring_2 + F_spring_3 + F_wind_2 + F_damper_2 + F_damper_3;
                
                aux_rope_final = force_2;
                obj.total_force(:, i) = obj.total_force(:, i) + aux_rope_final;
                obj.norm_force(i, 1) = norm(obj.total_force(:, i), 2);
                
            end
            x = obj.total_force;
            x_norm = obj.norm_force;
        end
        
        function apply_Velocity(obj,Velocity_systems)
            for i = 1:obj.number_rope
                obj.n_masses(i, 1).set_vel(Velocity_systems(:,i))
            end
        end
        
        function x = ropes_initial_angles(obj, k)
            if k ==1
                
                x = obj.spring_l*[-cos(obj.alpha); 0; sin(obj.alpha)];
                
            else
                x = obj.spring_l*[+cos(obj.alpha); 0; sin(obj.alpha)];
                
            end
            
        end
        
        
    end
end


