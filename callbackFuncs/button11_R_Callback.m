function button11_R_Callback(hObject,eventdata)
% Callback function for button 11_R

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','wristVel_R',[0;0;1]);
else
    assignin('base','wristVel_R',[]);
end


end