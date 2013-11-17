function button9_Callback(hObject,eventdata)
% Callback function for button 9

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','wristVel_L',[0;0.5;0]);
else
    assignin('base','wristVel_L',[]);
end


end