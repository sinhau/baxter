classdef RobotRaconteur
         
    methods
        function a=RobotRaconteur(c)
           error('Valid commands are Connect, Disconnect, EnableEvents, DisableEvents, RequestObjectLock, ReleaseObjectLock, MonitorEnter,MonitorExit, ProcessRequests, FindService, Shutdown'); 
        end
    end
    
    methods(Static=true)
        function ret=Connect(url,username,credentials)
			if nargin ==1
				ret=RobotRaconteurMex('Connect',url); 
			else
				ret=RobotRaconteurMex('Connect',url,username,credentials); 
		    end
        end
		
		
        
        function Disconnect(objref)
           disconnect(objref);
        end
        
        function EnableEvents(objref)
           enableevents(objref);
        end
        
        function DisableEvents(objref)
           disableevents(objref);
        end
        
        function s=FindService(name)
           s=RobotRaconteurMex('FindService',name); 
        end
        
        function ProcessRequests()
           
           RobotRaconteurMex('ProcessRequests');
        end
        
        function RequestObjectLock(objref,type)
            if (nargin==1)
                type='User';
            end
            if (strcmp(type,'User'))
                lockop(objref,'RequestUserLock');
            elseif (strcmp(type,'Client'))
                lockop(objref,'RequestClientLock');
            else
                error('Unknown command')
            end
        end
        
        function ReleaseObjectLock(objref)
            lockop(objref,'ReleaseUserLock');
        end
        
        function MonitorEnter(objref)
           lockop(objref,'MonitorEnter'); 
        end
        
        function MonitorExit(objref)
            lockop(objref,'MonitorExit'); 
        end
        
        function Shutdown()
           RobotRaconteurMex('Shutdown');
           clear RobotRaconteurMex
        end
        
    end
    
end

