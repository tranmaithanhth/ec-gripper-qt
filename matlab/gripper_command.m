function response = gripper_command(action, position)
% MATLAB UDP client for the Qt application; NOT a Simulink model converter.
% Requires Instrument Control Toolbox and MATLAB supporting udpport.
% Qt: Connect -> ARM -> enable VR5 UDP. Stop robot_main (port 5051).
% gripper_command('open') or gripper_command('close',700)
if nargin < 2, position = 700; end
if strcmpi(action,'open')
    command = 'GRIPPER_OPEN';
elseif strcmpi(action,'close')
    validateattributes(position, {'numeric'}, {'scalar','integer','>=',0,'<=',1000});
    command = sprintf('GRIPPER_CLOSE:%d',position);
else
    error('Use open or close');
end
u = udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',5051,'Timeout',25);
write(u,uint8(command),'uint8','127.0.0.1',5050);
t = tic;
while u.NumDatagramsAvailable == 0
    if toc(t) > 25
        error('No response; command may still be active. Check Qt. No automatic retry.');
    end
    pause(0.02);
end
d = read(u,1,'uint8');
response = char(d(1).Data);
disp(response);
end
