function button5_Callback(hObject,eventdata)
% Callback function for button 5

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','linVel',[0;0;0.06]);
else
    assignin('base','linVel',[0;0;0]);
end


end