function button13_R_Callback(hObject,eventdata)
% Callback function for button 13_R

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','grip_R',single(100));
else
    assignin('base','grip_R',[]);
end


end