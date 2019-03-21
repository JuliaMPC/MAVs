# Results Directory
This directory share files between local host and container and it is used to store results data.

There are two main scripts that are provided to help the user process the rosbag (i.e., `.bag`) data: `bag_to_csv.py` and `plottingData.jl`.

## bag_to_csv.py
This is the main processing script and it goes through and saves all of the topics in the bag files to individual `.csv` files.

Consider the example
```
python bag_to_csv.py /home/mavs/MAVs/results/tmp.bag "/home/mavs/MAVs/results/tmp"
```
where the two arguments that are passed to this script are the rosbag file and the directory that the `.csv` files will be saved.

## plottingData.jl
This script is provided to sample the position of obstacles and vehicle at a set number of points, which is `4` by default. This data is then saved into `.csv` files.

The main function is `main(folder,case)`, where ``folder`` is the folder name that the results are stored in and ``case`` it the ``.yaml`` file case name, which is set in the launch file.

## Complete example using these scripts

Running in the MAVs container
```
cd /home/mavs/MAVs/results
rmdir tmp
mkdir tmp
python bag_to_csv.py /home/mavs/MAVs/results/tmp.bag "/home/mavs/MAVs/results/tmp"
julia plottingData.jl "/home/mavs/MAVs/results/tmp/" "s8"
rm /home/mavs/MAVs/results/tmp/state.csv
```


## Other examples

Saving obstacle detector data

```
cd /home/mavs/MAVs/results
rm -rf obsD
mkdir obsD
python bag_to_csv.py /home/mavs/MAVs/results/obs.bag "/home/mavs/MAVs/results/obsD"
```
