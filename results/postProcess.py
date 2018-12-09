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
    bag2csv = csv.writer(csvfile, delimiter=';', quotechar='\'', quoting=csv.QUOTE_MINIMAL)

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

    print "Finished!"
    # close files
    csvfile.close()
    bag.close()
    return csv_name+'.csv'


def processcsv(csvfilename):
	F = open(csvfilename, 'r')
	F_w = open(csvfilename[0:(len(csvfilename)-4)] + "_state.csv", 'w')
	F_w1 = open(csvfilename[0:(len(csvfilename)-4)] + "_temp.csv", 'w')

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
	F_w1 = open(csvfilename[0:(len(csvfilename)-4)] + "_temp.csv", 'r')
	F_w2 = open(csvfilename[0:(len(csvfilename)-4)] + "_planned.csv", 'w')
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
	F_w1 = open(csvfilename[0:(len(csvfilename)-4)] + "_temp.csv", 'w')
	F_w1.truncate(0)
	F_w1.write(old2)
	F_w1.close()
	F_w1 = open(csvfilename[0:(len(csvfilename)-4)] + "_temp.csv", 'r')
	F_w3 = open(csvfilename[0:(len(csvfilename)-4)] + "_control.csv", 'w')
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
	F_w3 = open(csvfilename[0:(len(csvfilename)-4)] + "_control.csv", 'r')
	lines = F_w3.readlines()
	F_w3.close()
	F_w3 = open(csvfilename[0:(len(csvfilename)-4)] + "_control.csv", 'w')
	F_w3.writelines([item for item in lines[:-3]])
	F_w3.close()
	# Remove last several lines for planned, because there are several redudant symbols
	F_w2 = open(csvfilename[0:(len(csvfilename)-4)] + "_planned.csv", 'r')
	lines = F_w2.readlines()
	F_w2.close()
	F_w2 = open(csvfilename[0:(len(csvfilename)-4)] + "_planned.csv", 'w')
	F_w2.writelines([item for item in lines[:-2]])
	F_w2.close()
	os.remove(csvfilename[0:(len(csvfilename)-4)] + "_temp.csv")


if __name__ == '__main__':
	print "Start to convert rosbag to csv: "
	i = 0;
	if len(sys.argv) > 1:
		for bag_name in sys.argv:
			#print(bag_name[(len(bag_name)-4):(len(bag_name))])
			#if bag_name[(len(bag_name)-5):(len(bag_name)-1)]=='.bag':
			if(i > 0):
				print("processing "+bag_name)
				filename = bag2csv(bag_name)
				processcsv(filename)
			i = i + 1
	else:
		for file in os.listdir("."):
			    	if file.endswith(".bag"):
					bag_name = os.path.join("", file)
					print("processing "+bag_name)
					#bag_name = sys.argv[1]
					filename = bag2csv(bag_name)
					processcsv(filename)
