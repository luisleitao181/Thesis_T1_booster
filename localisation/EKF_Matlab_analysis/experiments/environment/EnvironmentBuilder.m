%% EnvironmentBuilder.m
classdef EnvironmentBuilder
    methods(Static)
        function env = create_simple(n_features)     
            env.xp = [];
            env.yp = [];
            env.zp = [];
            env.feature_groups = [];
            env.feature_types = [];
            
            if n_features == 0
                % Sem features
                
            elseif n_features == 1
                env.xp = [2];
                env.yp = [-1];
                env.zp = [2];
                
            elseif n_features == 2
                env.xp = [2, -2];
                env.yp = [-1, -1];
                env.zp = [2, 2];
                
            elseif n_features == 3
                env.xp = [2, -2, 2];
                env.yp = [-1, -1, 4];
                env.zp = [2, 2, 2];
                
            elseif n_features == 4
                env.xp = [2, -2, 2, -2];
                env.yp = [-1, -1, 4, 4];
                env.zp = [2, 2, 2, 2];
                
            elseif n_features == 5
                env.xp = [2, -2, 2, -2, 0];
                env.yp = [-1, -1, 4, 4, 1.5];
                env.zp = [2, 2, 2, 2, 2.5];
                
            elseif n_features == 6
                env.xp = [2, -2, 2, -2, 1, -1];
                env.yp = [-1, -1, 4, 4, 1.5, 1.5];
                env.zp = [2, 2, 2, 2, 2.5, 2.5];
                
            elseif n_features == 7
                env.xp = [2, -2, 2, -2, 1, -1, 0];
                env.yp = [-1, -1, 4, 4, 1.5, 1.5, -0.5];
                env.zp = [2, 2, 2, 2, 2.5, 2.5, 1.8];
                
            elseif n_features == 8
                env.xp = [2, -2, 2, -2, 1, -1, 0, 0];
                env.yp = [-1, -1, 4, 4, 1.5, 1.5, -0.5, 4.5];
                env.zp = [2, 2, 2, 2, 2.5, 2.5, 1.8, 2.2];
                
            elseif n_features == 9
                env.xp = [2, -2, 2, -2, 1, -1, 0, 0, 3];
                env.yp = [-1, -1, 4, 4, 1.5, 1.5, -0.5, 4.5, 2];
                env.zp = [2, 2, 2, 2, 2.5, 2.5, 1.8, 2.2, 2.3];
                
            elseif n_features == 10
                env.xp = [2, -2, 2, -2, 1, -1, 0, 0, 3, -3];
                env.yp = [-1, -1, 4, 4, 1.5, 1.5, -0.5, 4.5, 2, 2];
                env.zp = [2, 2, 2, 2, 2.5, 2.5, 1.8, 2.2, 2.3, 2.3];
                
            else
                error('n_features deve ser entre 0 e 10');
            end
            
            n = length(env.xp);
            env.feature_groups = ones(1, n);
            env.feature_types = ones(1, n);
            env.num_features = n;
            env.num_groups = min(1, n);
        end
    end
end