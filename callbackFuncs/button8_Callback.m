function button8_Callback(hObject,eventdata)
% Callback function for button 8

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','wristVel_L',[-0.5;0;0]);
else
    assignin('base','wristVel_L',[]);
end


end