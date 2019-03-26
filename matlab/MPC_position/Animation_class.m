classdef Animation_class < handle
    properties
        
        body % the object use to plot the body
        foot1 
        foot2
        foot3
        foot4
        
        fig % the figure object
        
    end
    
    methods
        function obj = Animation_class(cm, p1, p2, p3, p4) 
            obj.fig = figure('KeyPressFcn', @MyKeyDown, 'KeyReleaseFcn', @MyKeyUp);
            obj.fig.UserData = false(1,7);
            set(obj.fig, 'Position', get(0,'Screensize'));
            obj.body = plot(cm(1,1),cm(2,1),'o','MarkerSize',30);
            hold on
            
            obj.foot1 = plot(p1(1,1), p1(2,1), 'x','MarkerSize',30);
            obj.foot2 = plot(p2(1,1), p2(2,1), 'x','MarkerSize',30);
            obj.foot3 = plot(p3(1,1), p3(2,1), 'x','MarkerSize',30);
            obj.foot4 = plot(p4(1,1), p4(2,1), 'x','MarkerSize',30);
            
            axis([cm(1,1)-0.4 cm(1,1)+0.4 cm(2,1)-0.4 cm(2,1)+0.4])     
        end
        
        function update(obj,x,p1,p2,p3,p4,C)
            set(obj.body, 'XData', x(1,1));
            set(obj.body, 'YData', x(2,1));

            if C(1) == 1.0
                set(obj.foot1, 'XData', p1(1,1));
                set(obj.foot1, 'YData', p1(2,1));
            else
                set(obj.foot1, 'XData', NaN);
                set(obj.foot1, 'YData', NaN);
            end
            
            if C(2) == 1.0
                set(obj.foot2, 'XData', p2(1,1));
                set(obj.foot2, 'YData', p2(2,1));
            else
                set(obj.foot2, 'XData', NaN);
                set(obj.foot2, 'YData', NaN);
            end
            
            if C(3) == 1.0
                set(obj.foot3, 'XData', p3(1,1));
                set(obj.foot3, 'YData', p3(2,1));
            else
                set(obj.foot3, 'XData', NaN);
                set(obj.foot3, 'YData', NaN);
            end
            
            if C(4) == 1.0
                set(obj.foot4, 'XData', p4(1,1));
                set(obj.foot4, 'YData', p4(2,1));
            else
                set(obj.foot4, 'XData', NaN);
                set(obj.foot4, 'YData', NaN);
            end
            
            axis([-0.4+x(1,1) 0.4+x(1,1) -0.4+x(2,1) 0.4+x(2,1)])
            drawnow;
        end
        
        function inputs = GetUserInputs(obj)
            inputs = obj.fig.UserData;
        end
        
    end
    
end

%% Callback function for detecting key presses
function MyKeyDown(hObject, event, handles)
    KeyNames = {'w', 's','a', 'd', 'j', 'k', 'q'};
    key = get(hObject,'CurrentKey');
    hObject.UserData = (strcmp(key, KeyNames) | hObject.UserData);
end

function MyKeyUp(hObject, event, handles)
    KeyNames = {'w', 's','a', 'd', 'j', 'k', 'q'};
    key = get(hObject,'CurrentKey');
    hObject.UserData = (~strcmp(key, KeyNames) & hObject.UserData);
end
        
        
            