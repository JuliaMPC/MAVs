#!/usr/bin/python
import sys
import csv
import rosbag
import io
import os

def bag2csv(bag_name, pathname):

    csv_name = bag_name.rstrip('.bag')

    # open bag file for read
    print "Reading " + bag_name
    bag = rosbag.Bag(bag_name)
    bag_topics = bag.get_type_and_topic_info()[1].keys()



    for bag_topic in bag_topics:
        # write topic name
        print "Write " + bag_topic + " into csv file..."
        csv_filename_temp = bag_topic.replace('/', '_')

		# open csv file for write
        csvfile = open(pathname + '/' +csv_filename_temp[1:]+'.csv', 'w')
        bag2csv = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)

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
                msg_value = msg_value.replace('[', '')
                msg_value = msg_value.replace(']', '')

                if title_flag == 0:
                    title_list.append(msg_title)
                value_list.append(msg_value)


            if title_flag == 0:
                bag2csv.writerow(title_list)
                title_flag = 1

            bag2csv.writerow(value_list)

		# close files
        csvfile.close()

    bag.close()
    return csv_name+'.csv'


def processcsv(pathname):
	planned_filename = open(pathname+'/'+'nloptcontrol_planner_control.csv', 'r')
	planned_csv = csv.reader(planned_filename, delimiter=',')
	state_filename = open(pathname+'/'+'state.csv', 'r')
	state_csv = csv.reader(state_filename, delimiter=',')
	miscResults_filename = open(pathname+'/'+'miscResults.csv', 'w')
	miscResults_csv = csv.writer(miscResults_filename, delimiter=',', quoting=csv.QUOTE_MINIMAL)
	miscResults_csv.writerow(['ceStr', 'ceAcc', 'teStr', 'teU'])

	planned_data = []
	for row in planned_csv:
		try:
			t_list = row[0].split(',')
			sa_list = row[6].split(',')
			ux_list = row[7].split(',')
			planned_data.append([t_list[0], sa_list[0], ux_list[0]])
		except:
			pass
	planned_data = planned_data[1:]
	# print planned_data


	cnt = 0
	vce = 0.0 # Acc control effort
	sce = 0.0 # Steering control effort
	vce_accum = 0.0 # accumulated value for acc control effort
	sce_accum = 0.0 # accumulated value for steering control effort

	for row in state_csv:
		try:
			se = float(planned_data[cnt][1]) - float(row[8])
			ve = float(planned_data[cnt][2]) - float(row[3])
			sce_accum = sce_accum + abs(float(row[8]))
			vce_accum = vce_accum + abs(float(row[4]))
			sce = vce_accum/(float(row[0]) + 0.5) # Normalize the acc control effort by time
			vce = sce_accum/(float(row[0]) + 0.5) # Normalize the steering control effort by time

			miscResults_csv.writerow([sce, vce, se, ve])

			if float(row[0]) > float(planned_data[cnt][0]):
				cnt += 1
		except:
			pass
	state_filename.close()
	miscResults_filename.close()


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

	# add result
	planned_filename1 = open('report.csv', 'r')
	report_csv = csv.reader(planned_filename1, delimiter=',')
	name = name +'collision' + ',' +'goalReached'+ ',' +'timeLimit'+ ',' +'rollover'
	iter2 = 0

	for row2 in report_csv:
		iter2 += 1


	iter2 -= 1
	planned_filename1.close()

	planned_filename2 = open('report.csv', 'r')
	report_csv1 = csv.reader(planned_filename2, delimiter=',')
	iter1 = 0
	for row1 in report_csv1:
		if(iter1 == iter2 ):
			value = value +row1[1]+ ','+row1[2]+ ','+row1[3]+ ','+row1[4]

		iter1 += 1


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

	filename = bag2csv(bag_name, pathname)
	processcsv(pathname)
	readcase(demoname, casename, plannername, pathname)
