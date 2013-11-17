function button1_R_Callback(hObject,eventdata)
% Callback function for button 1_R

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','linVel_R',[0.04;0;0]);
else
    assignin('base','linVel_R',[0;0;0]);
end

end