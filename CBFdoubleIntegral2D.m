% Author: Jiyoung Hwang (hjy8918@yonsei.ac.kr)

classdef CBFdoubleIntegral2D < CBFcontroller
    methods
        function obj = CBFdoubleIntegral2D(params, u_des)
            obj@CBFcontroller(params, u_des)
        end

        function B = safetySet(obj, x)
            if x(2) > 0
                B(1,:) = obj.params.posMax - x(1) + x(2)^2/(2*obj.params.accMin);
                B(2,:) = obj.params.posMax - x(1);
            else
                B(1,:) = x(1) - obj.params.posMin;
                B(2,:) = x(1) - obj.params.posMin - x(2)^2/(2*obj.params.accMax);
            end

            B(3,:) = obj.params.velMax - x(2);
            B(4,:) = x(2) - obj.params.velMin;
        end

        function Bjacob = safetySetJacob(obj, x)
            % [ dB1(x)/x1 dB1(x)/x2 ]
            % [ dB2(x)/x1 dB2(x)/x2 ]
            % [         ...         ]
            if x(2) > 0
                Bjacob(1,:) = [-1 x(2)/(obj.params.accMin)];
                Bjacob(2,:) = [1 0];
            else
                Bjacob(1,:) = [-1 0];
                Bjacob(2,:) = [1 -x(2)/(obj.params.accMax)];
            end

            Bjacob(3,:) = [0 -1];
            Bjacob(4,:) = [0 1];
        end

        function dxdt = dynamics(obj, x, t)
            % define the state equations
            f = [x(2); 0];
            g = [0; 1];
            u = CBFqpSolver(obj, x, f, g);
            dxdt = f + g*u;
        end
    end
end

