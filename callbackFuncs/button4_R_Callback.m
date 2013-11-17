function button4_R_Callback(hObject,eventdata)
% Callback function for button 4_R

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','linVel_R',[0;-0.04;0]);
else
    assignin('base','linVel_R',[0;0;0]);
end


end