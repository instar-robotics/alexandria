# README #

## I Description ##

* What is Alexandria : Alexandria is a collection of functions for the Neural Networks Simulator Kheops.
  1. Alexandria contains the code of functions for kheops
  2. Alexandria contains XML description files for papyrus

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

* For Users : 
  1. Alexandria builds a collection of libraries (.so) 
  2. By default, libraries are copied in $CMAKE_INSTALL_PREFIX/lib/alexandria
  3. CMAKE_INSTALL_PREFIX default value is the install directory in your catkin_workspace 
  4. You can set CMAKE_INSTALL_PREFIX to every install dir before running **_catkin_make_ install**
  5. The path $CMAKE_INSTALL_PREFIX/lib/alexandria should be communicate to kheops

  1. Alexandria also copies XML description files in $CMAKE_INSTALL_PREFIX/share/alexandria/description
  2. The path $CMAKE_INSTALL_PREFIX/share/alexandria/description should be communicate to papyrus

* For developpers : 
  1. If you code ant test some news functions you probably doesn't want to install Alexandrai at each build time.
  2. So, you could only run : 
  
**_catkin_make_**

  1. Libraries are copied in $CATKIN_DEVEL_PREFIX/lib/alexandria
  2. XML desription files are copied in $CATKIN_DEVEL_PREFIX/share/alexandria/description
  3. You have to communicate both path to kheops and papyrus


## Develop it first Function ##

### General description ###

* Functions are defined by some objects : 
  1. .h file : contains the C++ header of the Function
  2. .cpp file : contains the C++ code of the Function
  3. .xml file : contains the description of the Function (For papyrus)
  4. icons directory : contains the SVG file for Function icon in payrus

* First you have to create a directory and the 3 empties files in the lib directory of Alexandria
* For a Function, "HelloFunct" you have to create the hellofunct directory then inside hellofunct.cpp , hellofunct.h and hellofunct.xml

### First class HelloFunct ###

* In kheops, Functions are strongly typed. Output can be SCALAR (double) or MATRIX (double unit)
* When you built your first Function you have first to decide which type of output you want.
* For the firt example, we chose a MATRIX output for the function : 

```javascript
class HelloFunct : public FMatrix
```

* For SCALAR Function you just have to inherit from FScalar class.
* FScalar and FMatrix provide an interface for kheops kernel and you have at least 2 functions to defines : 

```javascript
virtual void compute();

virtual void setparameters();
```

* compute function is called by kheops at each kernel iteration : this is the payload of your Function.
* setparameters is called by kheops when it load the function before creating the graph. We will describe in details this function later.

* There are third function which are optional : 

```javascript
virtual void prerun();
```

* This function is called by kheops after creating the graph.  We will describe in details this function later.
* The last functions you have to define is the class constructor and destructor : 

```javascript
HelloFunct();

virtual ~HelloFunct();
```

* So at this time your hellofunct.h should be like that : 

```javascript
#ifndef __HELLOFUNCT_HPP__
#define __HELLOFUNCT_HPP__

#include "kheops/kernel/function.h"   // Mandatory header file 
#include "kheops/kernel/kernel.h"     // Mandatory header file 

class HelloFunct : public FMatrix
{
        public :

                HelloFunct();
                virtual ~HelloFunct();

                virtual void compute();
                virtual void setparameters();
};

#endif // __HELLOFUNCT_HPP__
```

### Implements first Hello world ###

* Now we have to write the hellofunc.cpp file.
* So, we have 4 functions to implements. 
* As the HelloFunct class have no input, constructor and destructor should be empty, and the setparameters too.

* So a first implementation should be like this : 

```javascript

#include "hellofunct.h"   // Mandatory to find Class definition
#include <iostream>       // For cout function

REGISTER_FUNCTION(HelloFunct);  // Mandatory to register the class in the kernel Factory

HelloFunct::HelloFunct() {}  // Empty construtor
HelloFunct::~HelloFunct() {}  // Empty destrutor

void HelloFunct::setparameters() {}

void HelloFunct::compute() 
{
    std::cout << "Hello World" << std::endl;
}
```

* The code is pretyy simple. We have a HelloFunct class which print "Hello world" on the standard output at each kernel iteration.
* Macro **_REGISTER_FUNCTION_** is mandatory to register the HelloFunct into the kernel.

* At this time, you should be able to compile the Function and find your **_libhellofunct.so_** 
* Just run, in your catkin workspace : 
  
**_catkin_make_** 

* And look in devel/lib/alexandria

### Write XML Description file ###

* Now we have to write the hellofunc.xml file.

```xml
<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>
<libname>lms</libname>
<functions>
        <function>
                <name>LMS</name>
                <output type="MATRIX">
                </output>
                <inputs>
                        <input multiple="false" type="SCALAR_SCALAR">
                                <name>learning_rate</name>
                        </input>
                        <input multiple="false" type="SCALAR_MATRIX">
                                <name>unconditionnal</name>
                        </input>
                        </input>
                        <input multiple="true" type="MATRIX_MATRIX">
                                <name>conditionnals</name>
                        </input>
                </inputs>
        </function>
</functions>
```

### Add Inputs to the Function ###

* Object : link, input an Function


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


### Update XML Description file ###



## Create its own repository ##

* demofct example 
  1. CMakeList.txt 

