import sys
import io
import os

def find_all(a_str, sub):
    start = 0
    while True:
        start = a_str.find(sub, start)
        if start == -1: return
        yield start
        start += len(sub) # use start += 1 to find overlapping matches


print "This is the name of the script: ", sys.argv[0]

F = open(str(sys.argv[1]), 'r')
F_w = open(str(sys.argv[1])[0:(len(str(sys.argv[1]))-4)] + "_state.csv", 'w')
F_w1 = open(str(sys.argv[1])[0:(len(str(sys.argv[1]))-4)] + "_temp.csv", 'w')

old = F.read()
F.seek(0,0)
old1 = F.read()
F.seek(0,0)
old2 = F.read()
# copy the whole input F file and remove all the [] and replaced that with ; for state
old = old.replace("[", "")
old = old.replace("]", "")
old = old.replace(",", ";")
# copy the whole input F file and remove all the [] and replaced that with ; for planned
old1 = old1.replace("[", "")
old1 = old1.replace("]", "")
old1 = old1.replace(",", ";")
# copy the whole input F file and remove all the [] and replaced that with ; for control
old2 = old2.replace("[", "") # for control information
old2 = old2.replace("]", "")
old2 = old2.replace(",", ";")

# Find the start position for three topics: /state, /opt, /control
result = old.find("state")
result1 = old1.find("/opt")
result2 = old1.find("/nlopcontrol_planner/control")
old = old[result+7:len(old)]
old = old.replace("tire_f", "vtffr;vtffl;vtfrr;vtfrl")
old1 = old1[result1+6:result2]
old1 = old1.replace("X0p;X0a;X0e", "x;y;v;r;psi;sa;ux;ax")
old2 = old2[result2+30:result]

# The first step cleaned values for three topics in total are written in F_w1 for latter use
# State information is already good to go and store in F_w
F_w.truncate(0)
F_w.write(old)
F_w1.truncate(0)
F_w1.write(old1)
F_w1.seek(0,0)
F_w1.close()

# Continue to process planned csv
F_w1 = open(str(sys.argv[1])[0:(len(str(sys.argv[1]))-4)] + "_temp.csv", 'r')
F_w2 = open(str(sys.argv[1])[0:(len(str(sys.argv[1]))-4)] + "_planned.csv", 'w')
old = ""
count = 0
line = F_w1.readline()
cnt = 1
old = line
while line:
	line = F_w1.readline()
	temp = line.split(';',13)
	old = old + '\n'
	try:
		for i in xrange(0,11):
			old = old + temp[i] + ';'
		old = old + temp[11]
	except:
		pass
	cnt += 1
F_w2.truncate(0)
F_w2.write(old)
F_w1.close()

# Continue to process control csv
F_w1 = open(str(sys.argv[1])[0:(len(str(sys.argv[1]))-4)] + "_temp.csv", 'w')
F_w1.truncate(0)
F_w1.write(old2)
F_w1.close()
F_w1 = open(str(sys.argv[1])[0:(len(str(sys.argv[1]))-4)] + "_temp.csv", 'r')
F_w3 = open(str(sys.argv[1])[0:(len(str(sys.argv[1]))-4)] + "_control.csv", 'w')
old = ""
count = 0
line = F_w1.readline()
cnt = 1
vce = 0.0 # Velocity control effort
sce = 0.0 # Steering control effort
vce_accum = 0.0 # accumulated value for velocity control effort
sce_accum = 0.0 # accumulated value for steering control effort
old = line[0:-2] + ';sce' + ';vce'
while line:
	line = F_w1.readline()
	temp = line.split(';',11*16-1)
	# print temp
	old = old + '\n'
	try:
		for i in xrange(0,11):
			old = old + temp[i*16] + '; '
		sce_accum = sce_accum + abs(float(temp[6 * 16]))
		vce_accum = vce_accum + abs(float(temp[7 * 16]))
		sce = vce_accum/(float(temp[0]) + 0.5) # Normalize the velocity control effort by time
		vce = sce_accum/(float(temp[0]) + 0.5) # Normalize the steering control effort by time
		old = old + str(sce) + ' ; ' # Concatenate the normalized velcoity control effort
		old = old + str(vce) 		 # Concatenate the normalized steering control effort
	except:
		pass
	cnt += 1
F_w3.truncate(0)
F_w3.write(old)
F_w1.close()
F_w3.close()
# Remove last several lines for control, because there are several redudant symbols
F_w3 = open(str(sys.argv[1])[0:(len(str(sys.argv[1]))-4)] + "_control.csv", 'r')
lines = F_w3.readlines()
F_w3.close()
F_w3 = open(str(sys.argv[1])[0:(len(str(sys.argv[1]))-4)] + "_control.csv", 'w')
F_w3.writelines([item for item in lines[:-3]])
F_w3.close()
# Remove last several lines for planned, because there are several redudant symbols
F_w2 = open(str(sys.argv[1])[0:(len(str(sys.argv[1]))-4)] + "_planned.csv", 'r')
lines = F_w2.readlines()
F_w2.close()
F_w2 = open(str(sys.argv[1])[0:(len(str(sys.argv[1]))-4)] + "_planned.csv", 'w')
F_w2.writelines([item for item in lines[:-2]])
F_w2.close()
os.remove(str(sys.argv[1])[0:(len(str(sys.argv[1]))-4)] + "_temp.csv")
