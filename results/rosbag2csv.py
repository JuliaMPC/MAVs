#!/usr/bin/python
import sys
import csv
import rosbag

if __name__ == '__main__':
    print "Start to convert rosbag to csv"
    bag_name = sys.argv[1]
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