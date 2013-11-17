function button6_R_Callback(hObject,eventdata)
% Callback function for button 6_R

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','linVel_R',[0;0;-0.06]);
else
    assignin('base','linVel_R',[0;0;0]);
end


end