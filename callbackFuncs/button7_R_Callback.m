function button7_R_Callback(hObject,eventdata)
% Callback function for button 7_R

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','wristVel_R',[0.5;0;0]);
else
    assignin('base','wristVel_R',[]);
end


end