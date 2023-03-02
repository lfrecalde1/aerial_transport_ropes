classdef particle < matlab.mixin.SetGet
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        m
        pos
        vel
        q
        ts
        internal_force
        internal_aceleration;
    end
    
    methods
        function obj = particle(m, dt)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.m = m;
            obj.pos = zeros(3, 1);
            obj.vel = zeros(3, 1);
            obj.q = [obj.pos;obj.vel];
            obj.ts = dt;
            obj.internal_force = zeros(3, 1);
            obj.internal_aceleration = zeros(3,1);
        end
        
        function set_pos(obj, position)
            obj.pos = position;
            obj.q = [obj.pos;obj.vel];
        end
        
        function set_vel(obj, velocity)
            obj.vel = velocity;
            obj.q = [obj.pos;obj.vel];
        end
        
        function x = get_pos(obj)
            x = obj.pos;
        end
        
        function x = get_vel(obj)
            x = obj.vel;
        end
        
        function x = get_internal_force(obj)
            x = obj.internal_force;
            
        end
        
        function x = get_internal_aceleration(obj)
            x = obj.internal_aceleration;
            
        end
        
        function apply_force(obj, forces)
           obj.internal_force = obj.internal_force + forces;
        end
        
        function xp = f_model(obj, x, force)
            obj.internal_aceleration = force/obj.m;
            xp1 = x(4:6);
            xp2 = obj.internal_aceleration;
            xp = [xp1;xp2];
        
        end
        
        function f_system(obj)
            force = obj.internal_force;
            T_s = obj.ts;
            % General States System
            x = obj.q;
           
            k1 = obj.f_model(x, force);
            k2 = obj.f_model(x + T_s/2*k1, force);
            k3 = obj.f_model(x + T_s/2*k2, force);
            k4 = obj.f_model(x + T_s*k3, force);
            x = x +T_s/6*(k1 +2*k2 +2*k3 +k4);
            
            % Update values system and set force to zero
            obj.q = x;
            obj.pos = x(1:3);
            obj.vel = x(4:6);
            obj.internal_force = zeros(3,1);
            obj.internal_aceleration = zeros(3, 1);
        end
    end
end

