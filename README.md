# cs225a Grocery Robot

This repository contain the simulation of a robot that pick and place objects from a shelf and then does checkout on a "Grocery Store"

You can find the video of the simulation in:
https://youtu.be/f_2yzn_A708

## Dependencies
The project depends on the sai2 libraries.
For more information please check http://khatib.stanford.edu/projects/controlandsimulation.html


## Build and run
Inside the SAI2/app folder copy this repo.
In the repo folder make a build folder and compile from there
```
mkdir build
cd build
cmake .. && make -j4
```
## run the code
in a terminal do
```
cd bin/grocery_robot
./simviz_grocery_robot 
```

and in other terminal:
```
cd bin/grocery_robot
./controller_grocery_robot 
```


