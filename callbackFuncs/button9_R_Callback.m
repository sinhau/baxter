function button9_R_Callback(hObject,eventdata)
% Callback function for button 9_R

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','wristVel_R',[0;0.5;0]);
else
    assignin('base','wristVel_R',[]);
end


end