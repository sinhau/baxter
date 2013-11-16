function makeGUI()

panel = figure('Visible','off','position',[50 500 1000 500]);

button1 = uicontrol('Style','togglebutton','position',[150 450 75 25],...
    'callback',@button1_Callback,'String','Forward');
button2 = uicontrol('Style','togglebutton','position',[150 400 75 25],...
    'callback',@button2_Callback,'String','Backward');
button3 = uicontrol('Style','togglebutton','position',[50 400 75 25],...
    'callback',@button3_Callback,'String','Left');
button4 = uicontrol('Style','togglebutton','position',[250 400 75 25],...
    'callback',@button4_Callback,'String','Right');
button5 = uicontrol('Style','togglebutton','position',[350 450 75 25],...
    'callback',@button5_Callback,'String','Up');
button6 = uicontrol('Style','togglebutton','position',[350 400 75 25],...
    'callback',@button6_Callback,'String','Down');

set(panel,'Visible','on');

end