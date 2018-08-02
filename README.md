# README #

## I Description ##

* What is Alexandria : Alexandria is a collection of functions for the Neural Networks Simulator Kheops.
* Alexandria contains the code of functions for kheops
* Alexandria contains XML description files for papyrus

## II Installation ##

### Dependancy ###

* Alexandria requiers kheops and all its dependancies.
* You need to install kheops first.

### Install Alexandria ###
* clone the repository in your catkin workspace :

**_cd /home/johndoe/catkin_workspace/src_**

**_git clone git@git.instar-robotics.com:software/NeuralNetwork/alexandria.git_**

* go to your root catkin workspace :

**_cd /home/johndoe/catkin_workspace_**

* And run **_catkin_make_ install**

* Alexandria builds a collection of libraries (.so) 
* By default, libraries are copied in $CMAKE_INSTALL_PREFIX/lib/alexandria
* CMAKE_INSTALL_PREFIX default value is the install directory in your catkin_workspace 
* You can set CMAKE_INSTALL_PREFIX to every install dir before running **_catkin_make_ install**
* The path $CMAKE_INSTALL_PREFIX/lib/alexandria should be communicate to kheops

* Alexandria also copies XML description files in $CMAKE_INSTALL_PREFIX/share/alexandria/description
* The path $CMAKE_INSTALL_PREFIX/share/alexandria/description should be communicate to papyrus

* For developpers : 

* TODO : describe XML structure

* Object : link, input an Function

* Function : Strongly typed, output can be Scalar (double) or Matrix (double unit)
* Functions have a number of defines inputs
* Each input have a define name and a type, and could have weights 
  1. String : 
  2. Scalar_Scalar : Input from Scalar output function. Weight is a scalar
  3. Scalar_Matrix : Input from Matrix output function. Weight is a scalar and is apply globaly on every neurons of the Matrix
  4. Matrix_Matrix : Input from Matrix output function. Weights are a Matrix.

* Matrix_Matrix details : We have 3 types of connections
  1. One to All connections (ONE_TO_ALL) : Dense connections between input and output 
  2. One to One connections (ONE_TO_ONE) : Sparse conenction between input and output 
  3. One to Neighborhood connections (ONE_TO_NEI) : Sparse conenction between input and output 
  4. 
  

* Using Input and iLink 

* Sparse Matrix : Connections is define is a Sparse Matrix Filter and we can generate every topology


TODO: 

* Howto develop is own function : 
* demofct example 
  1. CMakeList.txt 
  2. Class Function : FScalar or FMatrix
  3. Files header : function.h and kernel.h
  3. Inputs defintions : add each input in private part
  4. Input register : setparameters functions and input bind
  5. Function register : Macro Register
