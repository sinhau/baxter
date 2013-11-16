function button11_Callback(hObject,eventdata)
% Callback function for button 11

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','angVel',[0;0;0.2]);
else
    assignin('base','angVel',[0;0;0]);
end


end