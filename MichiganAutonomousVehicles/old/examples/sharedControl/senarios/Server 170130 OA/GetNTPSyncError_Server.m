[status,cmdout] = dos('ntpq -nc peers','-echo');
% [status,cmdout] = dos('ntpdate -d 66.219.116.140','-echo');
st = strread(cmdout,'%s');

st_table = [st(1:10) st(12:21)]';
name = char(st_table(2,1));
ServerOffset = str2double(cell2mat(st_table(2,9))); % average in ms,offset<0, local computer is lagging behind


% st_table = [st(1:10) st(12:21) st(22:31) st(32:41) st(42:51) st(52:61)]';
% 
% for i = 2:6
%     name = char(st_table(i,1));
%     if (name(1)=='*')
%         ServerOffset = str2double(cell2mat(st_table(i,9)));       
%                    
%         server = name(2:end);
%         break
%     end
% end
