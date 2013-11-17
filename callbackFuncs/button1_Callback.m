function button1_Callback(hObject,eventdata)
% Callback function for button 1

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','linVel_L',[0.04;0;0]);
else
    assignin('base','linVel_L',[0;0;0]);
end

end