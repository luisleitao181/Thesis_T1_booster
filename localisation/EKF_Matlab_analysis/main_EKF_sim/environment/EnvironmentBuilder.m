%% EnvironmentBuilder_simple.m
classdef EnvironmentBuilder
    methods(Static)
        function env = create_simple(n_features)     
            env.xp = [];
            env.yp = [];
            env.zp = [];
            env.feature_groups = [];
            env.feature_types = [];
            
            if n_features == 0
                
            elseif n_features == 1
                %env.xp = -1.5;
                %env.yp = 1.5;
                %env.zp = 2;
                env.xp = [2];
                env.yp = [-1];
                env.zp = [2];                
            elseif n_features == 2
                %env.xp = [-1, -1.5];
                %env.yp = [1.5, 4];
                %env.zp = [2, 2];
                env.xp = [2, -2];
                env.yp = [-1, -1];
                env.zp = [2, 2];
                
            elseif n_features == 3
                %env.xp = [-1, -1.5, 1];
                %env.yp = [1.5, 4, 2.5];
                %env.zp = [2, 2, 2];
                env.xp = [2, -2, 2];
                env.yp = [-1, -1, 4];
                env.zp = [2, 2, 2];
            elseif n_features == 4
                env.xp = [2, -2, 2, -2];
                env.yp = [-1, -1, 4, 4];
                env.zp = [2, 2, 2, 2];
                
            else
                error('n_features must be 0, 1, 2, 3, or 4');
            end
            
            n = length(env.xp);
            env.feature_groups = ones(1, n);
            env.feature_types = ones(1, n);
            env.num_features = n;
            env.num_groups = min(1, n);
        end
    end
end