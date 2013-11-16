function button2_Callback(hObject,eventdata)

buttonState = get(hObject,'Value');

if buttonState==1
    assignin('base','linVel',[-0.04;0;0]);
else
    assignin('base','linVel',[0;0;0]);
end


end