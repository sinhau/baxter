function button8_Callback(hObject,eventdata)
% Callback function for button 8

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','angVel',[0;-0.2;0]);
else
    assignin('base','angVel',[0;0;0]);
end


end