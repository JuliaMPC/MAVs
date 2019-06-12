
try
    close(yingshi)
end
yingshi = UDPSocket();
bind(yingshi,ip"73.108.52.5",13000); # change this ip to the server (or julia code)

gains=[30.0,0.15,3.0];
function tune(gains)
  MsgOutString = ' ';
  for j in 1:length(gains)
      MsgOutString = string(MsgOutString,' ',gains[j]);
  end
  MsgOutString = string(MsgOutString," \n");
  send(yingshi,ip"141.212.141.245",36896,MsgOutString);
  # change this to the ip where you are running Simulink!
  print(MsgOutString)
end

#tune([90.0,20.,0.008]) # good gains
tune([90.0,0.0,0.0]) # good gains
