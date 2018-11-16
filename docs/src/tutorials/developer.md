# Developer

## Modifying files
### Modifying files that do not need to be compiled
There are several file types that can be modified without having to recompile:
 YAML files: configuration files
Launch files: these files orchestrate the launching the nodes and scripts needed to run the demos
Julia and Python files: these files contain the scripts needed to accomplish certain tasks

### Modifying files that need to be compiled (i.e. C++ files)
You can rebuild the workspace with two methods, where the first method takes shorter time and is recommended.

#### Method 1. Rebuild the ros workspace **inside the container**:  
```
$cd ~/MAVs/ros
$catkin_make
```
After that, you need to commit the change of image. To do this, open a terminal **outside of the container** and type:
```
$docker commit <container_name> <image_name>
```
Where `container_name` can be found with:
```
$ docker container ls
```
You will see something like this:
```
CONTAINER ID        IMAGE               COMMAND                  CREATED             STATUS              PORTS               NAMES
7607f52ca7a2        mavs                "/ros_entrypoint.sh …"   8 minutes ago       Up 8 minutes                            reverent_shtern
ce4096e14bef        mavs                "/ros_entrypoint.sh …"   12 hours ago        Up 12 hours                             vigilant_johnson
```
The most recently created one, `reverent_shtern`, is the one you want to commit. Don't forget about tab completion.

Then you can look at your images with:
```
$ docker image ls
```
Which gives something like:
```
REPOSITORY          TAG                 IMAGE ID            CREATED             SIZE
<none>              <none>              46eac405dcb4        8 minutes ago       9.56GB
mavs                latest              7c00d6949be1        12 hours ago        11.8GB
```
where the first `image` is the failed build obtained with **Method 2**...

Finally, in this example we have:
```
$docker commit reverent_shtern mavs
```

#### Method 2. Rebuild the image if container is terminated:
```
$sh build.sh
```

##### Potential issue
```
File "/home/mavs/MAVs/ros/src/CMakeLists.txt" already existsCMake Error: The current CMakeCache.txt directory /home/mavs/MAVs/ros/build/CMakeCache.txt is different than the directory /home/febbo/Documents/workspace/MAVs/ros/build where CMakeCache.txt was created. This may result in binaries being created in the wrong place. If you are not sure, reedit the CMakeCache.txt
CMake Error: The source "/home/mavs/MAVs/ros/src/CMakeLists.txt" does not match the source "/home/febbo/Documents/workspace/MAVs/ros/src/CMakeLists.txt" used to generate cache.  Re-run cmake with a different source directory.
```
##### Fix
Remove the cache file OR **just use the first method**

## Creating a demo
TODO

## Testing
TODO

## Documentation
TODO

## Making a pull request
TODO
