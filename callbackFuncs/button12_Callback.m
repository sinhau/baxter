function button12_Callback(hObject,eventdata)
% Callback function for button 12

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','wristVel_L',[0;0;-1]);
else
    assignin('base','wristVel_L',[]);
end


end