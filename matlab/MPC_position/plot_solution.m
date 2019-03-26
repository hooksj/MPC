function [] = plot_solution(x, ref_traj, P, C, dt, np)

    fig1 = figure;
    hold on
    body = plot(x(1,1), x(6,1), 'o');
    traj = plot(ref_traj(1,1), ref_traj(2,1));
    foot1 = plot(P(1,1), P(2,1), 'x');
    foot2 = plot(P(3,1), P(4,1), 'x');
    foot3 = plot(P(5,1), P(6,1), 'x');
    foot4 = plot(P(7,1), P(8,1), 'x');
    
    t = 0.0;
    for i = 0.0:0.001:(dt*np)
        pause(0.001)
        i = round(i,4);
        index = ceil(i/dt);
        T = [1, t, t^2, t^3, t^4];
        
        if index < 1
            index = 1;
        end
        
        ax = x(1+15*(index-1):5+15*(index-1),1);
        ay = x(6+15*(index-1):10+15*(index-1),1);
        
        set(body, 'XData', T*ax);
        set(body, 'YData', T*ay);
        
        set(traj, 'XData', ref_traj(1,1:index));
        set(traj, 'YData', ref_traj(3,1:index));
        
        if C(1,index) == 1.0
            set(foot1, 'XData', P(1,index));
            set(foot1, 'YData', P(2,index));
        else
            set(foot1, 'XData', NaN);
            set(foot1, 'YData', NaN);
        end
        
        if C(2,index) == 1.0
            set(foot2, 'XData', P(3,index));
            set(foot2, 'YData', P(4,index));
        else
            set(foot2, 'XData', NaN);
            set(foot2, 'YData', NaN);
        end
        
        if C(3,index) == 1.0
            set(foot3, 'XData', P(5,index));
            set(foot3, 'YData', P(6,index));
        else
            set(foot3, 'XData', NaN);
            set(foot3, 'YData', NaN);
        end
        
        if C(4,index) == 1.0
            set(foot4, 'XData', P(7,index));
            set(foot4, 'YData', P(8,index));
        else
            set(foot4, 'XData', NaN);
            set(foot4, 'YData', NaN);
        end
        axis([T*ax-0.4 T*ax+0.4 T*ay-0.4 T*ay+0.4])
        drawnow;  
        
%         t = round(t+0.001,4);
%         if t > dt
%             t = 0.00;
%         end
          t = rem(i,dt);
    end

end