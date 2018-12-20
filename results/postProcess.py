#!/usr/bin/python
import sys
import csv
import rosbag
import io
import os

def bag2csv(bag_name):

    csv_name = bag_name.rstrip('.bag')

    # open bag file for read
    print "Reading " + bag_name
    bag = rosbag.Bag(bag_name)
    bag_topics = bag.get_type_and_topic_info()[1].keys()

    # open csv file for write
    csvfile = open(csv_name+'.csv', 'w')
    #bag2csv = csv.writer(csvfile, delimiter=';', quotechar='\'', quoting=csv.QUOTE_MINIMAL)
    bag2csv = csv.writer(csvfile, delimiter=',', quotechar='\'', quoting=csv.QUOTE_MINIMAL)

    for bag_topic in bag_topics:
        # write topic name
        print "Write " + bag_topic + " into csv file..."
        bag2csv.writerow([bag_topic])
        # set title_flag to 0
        title_flag = 0

        for topic, msg, t in bag.read_messages(topics=bag_topic):
            # split msg
            msg_list = str(msg).split('\n')
            value_list = []
            title_list = []

            for msg_pair in msg_list:
                split_pair = msg_pair.split(':')
                msg_title = split_pair[0]
                msg_value = split_pair[1]

                if title_flag == 0:
                    title_list.append(msg_title)
                value_list.append(msg_value)

            if title_flag == 0:
                bag2csv.writerow(title_list)
                title_flag = 1

            bag2csv.writerow(value_list)

        # insert a row between to topics
        bag2csv.writerow([])


    # close files
    csvfile.close()
    bag.close()
    return csv_name+'.csv'


def processcsv(pathname, csvfilename):
	csvfilename_full = csvfilename
	statefilename_full = os.path.join(pathname, "state.csv")
	tempfilename_full = os.path.join(pathname, "temp.csv")
	F = open(csvfilename_full, 'r')
	F_w = open(statefilename_full, 'w')
	F_w1 = open(tempfilename_full, 'w')

	old = F.read()
	F.seek(0,0)
	old1 = F.read()
	F.seek(0,0)
	old2 = F.read()
	# copy the whole input F file and remove all the [] and replaced that with ; for state
	old = old.replace("[", "")
	old = old.replace("]", "")
	#old = old.replace(",", ";")
	# copy the whole input F file and remove all the [] and replaced that with ; for planned
	old1 = old1.replace("[", "")
	old1 = old1.replace("]", "")
	#old1 = old1.replace(",", ";")
	# copy the whole input F file and remove all the [] and replaced that with ; for control
	old2 = old2.replace("[", "") # for control information
	old2 = old2.replace("]", "")
	#old2 = old2.replace(",", ";")

	# Find the start position for three topics: /state, /opt, /control
	result = old.find("state")
	result1 = old1.find("/opt")
	result2 = old1.find("/nlopcontrol_planner/control")
	old = old[result+7:len(old)]
	old = old.replace("tire_f", "vtffr,vtffl,vtfrr,vtfrl")
	old1 = old1[result1+6:result2]
	old1 = old1.replace("X0p,X0a,X0e", "x,y,v,r,psi,sa,ux,ax")
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
	F_w1 = open(tempfilename_full, 'r')
	plannedfilename_full = os.path.join(pathname, "planned.csv")
	F_w2 = open(plannedfilename_full, 'w')
	old = ""
	count = 0
	line = F_w1.readline()
	cnt = 1
	se = list()
	ve = list()
	old = line
	while line:
		line = F_w1.readline()
		temp = line.split(',',28)
		old = old + '\n'
		try:
			for i in xrange(0,11):
				old = old + temp[i] + ','
			old = old + temp[11]
			se.append(temp[25])
			ve.append(temp[26])
		except:
			pass
		cnt += 1
	F_w2.truncate(0)
	F_w2.write(old)
	F_w1.close()

	# Continue to process control csv
	controlfilename_full = os.path.join(pathname, "miscResults.csv")
	F_w1 = open(tempfilename_full, 'w')
	F_w1.truncate(0)
	F_w1.write(old2)
	F_w1.close()
	F_w1 = open(tempfilename_full, 'r')
	F_w3 = open(controlfilename_full, 'w')
	old = ""
	count = 0
	line = F_w1.readline()
	cnt = 0
	vce = 0.0 # Acc control effort
	sce = 0.0 # Steering control effort
	vce_accum = 0.0 # accumulated value for acc control effort
	sce_accum = 0.0 # accumulated value for steering control effort
	old = line[0:-2] + ',ceStr' + ',ceAcc' + ',teStr' + ',teU' # se is the steering error and ve is the acc error
	while line:
		line = F_w1.readline()
		temp = line.split(',',11*16-1)
		# print temp
		old = old + '\n'
		try:
			for i in xrange(0,11):
				old = old + temp[i*16] + ', '
			sce_accum = sce_accum + abs(float(temp[6 * 16]))
			vce_accum = vce_accum + abs(float(temp[8 * 16]))
			sce = vce_accum/(float(temp[0]) + 0.5) # Normalize the velocity control effort by time
			vce = sce_accum/(float(temp[0]) + 0.5) # Normalize the steering control effort by time
			old = old + str(sce) + ' , ' # Concatenate the normalized acc control effort
			old = old + str(vce) + ' , ' # Concatenate the normalized steering control effort
			old = old + se[cnt] + ' , '
			old = old + ve[cnt]

		except:
			pass
		cnt += 1
	F_w3.truncate(0)
	F_w3.write(old)
	F_w1.close()
	F_w3.close()
	# Remove last several lines for control, because there are several redudant symbols
	F_w3 = open(controlfilename_full, 'r')
	lines = F_w3.readlines()
	F_w3.close()
	F_w3 = open(controlfilename_full, 'w')
	F_w3.writelines([item for item in lines[:-3]])
	F_w3.close()
	# Remove last several lines for planned, because there are several redudant symbols
	F_w2 = open(plannedfilename_full, 'r')
	lines = F_w2.readlines()
	F_w2.close()
	F_w2 = open(plannedfilename_full, 'w')
	F_w2.writelines([item for item in lines[:-2]])
	F_w2.close()
	os.remove(tempfilename_full)
	os.remove(csvfilename_full)


# save sX.ymal
def readcase(demoname, casename, plannername, pathname):
	configpath = "../ros/src/system/config/case/"
	sxfile= configpath+casename+".yaml"
	print("Reading "+casename+".yaml ...")
	casefilename_full = os.path.join(pathname, "case.csv")

	F_sx = open(sxfile, 'r')
	F_w = open(casefilename_full, 'w')

	line = F_sx.readline()

	name = ""
	value = ""
	while line:
		line = str(line)
		if len(line) > 1:
			if line[1]!=' ':
				line = line[1:-1]
				line = line.split(":")
				name1 = line[0]
			elif line[2]!=' ':
				line = line[2:]
				line = line.split(":")
				name2 = line[0]
				if(name2[0] <= 'z' and name2[0] >= 'a'):
					name2 = name2[0].upper()+name2[1:]
				for char2 in name2:
					if(char2 == '0'):
						char2 = 'O'
				if(len(line) > 1):
					value2 = line[1]
					value2 = value2[0:-1]


					if(len(value2) > 0):
						name = name + name1 + name2 + ','
						value = value + value2 + ','

			elif line[3]!=' ':
				line = line[3:]
				line = line.split(": ")
				name3 = line[0]
				value3 = line[1]

				if(name3[0] <= 'z' and name3[0] >= 'a'):
					name3 = name3[0].upper()+name3[1:]
				for char3 in name3:
					if(char3 == '0'):
						char3 = 'O'

				name = name + name1 + name2 + name3 + ','
				value3 = value3.replace(",", " ")
				value = value + value3[0:-1] + ','


		line = F_sx.readline()
	value = name+'\n'+value
	F_w.truncate(0)
	#F_w.writelines(name)
	F_w.write(value)
	F_sx.close()
	F_w.close()
	print "Finished!"


if __name__ == '__main__':
	# handle result information
	print "Start to convert rosbag to csv: "
	bag_name = sys.argv[1]
	print("filename: "+bag_name)
	tempname = str(sys.argv[1]).split("_")
	demoname = tempname[0]
	casename = tempname[1]
	plannername = tempname[2]
	print(demoname+" "+casename+" "+plannername)

	if len(sys.argv) == 2:
		pathname = "test"
	else:
		pathname = str(sys.argv[2])

	pathname = "./"+pathname+"/"+casename+plannername
	print("save files to: " + pathname)

	# make dir
	#dirname = os.path.dirname(filename)
	if not os.path.exists(pathname):
    		os.makedirs(pathname)

	filename = bag2csv(bag_name)
	processcsv(pathname, filename)
	readcase(demoname, casename, plannername, pathname)
