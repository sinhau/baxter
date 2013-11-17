function button6_Callback(hObject,eventdata)
% Callback function for button 6

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','linVel_L',[0;0;-0.06]);
else
    assignin('base','linVel_L',[0;0;0]);
end


end