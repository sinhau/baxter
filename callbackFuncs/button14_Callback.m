function button14_Callback(hObject,eventdata)
% Callback function for button 14

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','grip_L',single(0));
else
    assignin('base','grip_L',[]);
end


end