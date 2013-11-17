function button13_Callback(hObject,eventdata)
% Callback function for button 13

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','grip_L',single(100));
else
    assignin('base','grip_L',[]);
end


end