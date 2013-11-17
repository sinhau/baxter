function button14_R_Callback(hObject,eventdata)
% Callback function for button 14_R

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','grip_R',single(0));
else
    assignin('base','grip_R',[]);
end


end