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

* For **Neural Developpers** : 
  1. Alexandria builds a collection of libraries (.so) 
  2. By default, libraries are copied in $CMAKE_INSTALL_PREFIX/lib/alexandria
  3. CMAKE_INSTALL_PREFIX default value is the install directory in your catkin_workspace 
  4. You can set CMAKE_INSTALL_PREFIX to every install dir before running **_catkin_make_ install**
  5. The path $CMAKE_INSTALL_PREFIX/lib/alexandria should be communicate to kheops

  1. Alexandria also copies XML description files in $CMAKE_INSTALL_PREFIX/share/alexandria/description
  2. The path $CMAKE_INSTALL_PREFIX/share/alexandria/description should be communicate to papyrus

* For **Functions Developpers** : 
  1. If you code ant test some news functions you probably doesn't want to install Alexandrai at each build time.
  2. So, you could only run : 
  
**_catkin_make_**

  1. Libraries are copied in $CATKIN_DEVEL_PREFIX/lib/alexandria
  2. XML desription files are copied in $CATKIN_DEVEL_PREFIX/share/alexandria/description
  3. You have to communicate both path to kheops and papyrus


## Functions developper's guide ##

### Develop it first Function : General description ###

* Functions are defined by some objects : 
  1. .h file : contains the C++ header of the Function
  2. .cpp file : contains the C++ code of the Function
  3. .xml file : contains the description of the Function (For papyrus)
  4. icons directory : contains the SVG file for Function icon in payrus

* First you have to create a directory and the 3 empties files in the lib directory of Alexandria
* For a Function **HelloFunct** you have to create the hellofunct directory then inside the files hellofunct.cpp , hellofunct.h and hellofunct.xml

### First class HelloFunct ###

* In kheops, Functions are strongly typed. Output can be **SCALAR** (double) or **MATRIX** (double unit)
* When you built your Function you have first to decide which type of output you want.
* For the firt example, we chose a **MATRIX** output's Function : 

```javascript
class HelloFunct : public FMatrix
```

* For SCALAR Function you just have to inherit from **FScalar** class instead.
* **FScalar** and **FMatrix** both provide an interface for kheops kernel and you have at least 2 functions to defines : 

```javascript
virtual void compute();

virtual void setparameters();
```

* **compute** function is called by kheops at each kernel iteration : this is the payload of your Function.
* **setparameters** is called by kheops when it load the function before creating the graph. We will describe in details this function later.

* There are a third function which are optional : 

```javascript
virtual void prerun();
```

* **prerun** function is called by kheops after creating the graph.  We will describe in details this function later.
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

* Now we have to write the hellofunc.xml file : 

```xml
<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>
<libname>hellofunc</libname>
<functions>
        <function>
                <name>HelloFunct</name>
                <output type="MATRIX">
                </output>
                <icon>filename</icon>
        </function>
</functions>
```

* File explanations : 
 1. \<libname\>    : define the name of the library. the value should be egal to the .cpp file name. **hellofunct.cpp** gives **libhellofunct.so** and kheops kernel uses **hellofunct** to find the library.
 2. \<functions\>  : define the Function container. A lib could contain more than one Function. So you could list all the Function inside the <functions> balise.
 3. \<function\>   : start the Function's definition.
 4. \<name\>       : the C++ class name.
 5. \<output\>     : define the output type. The attribute **type** is mandatory and could be egal to **MATRIX** or **SCALAR**
 6. \<icon\>       : icon's filename (without extension). The icon have to be a SVG file and should be put in **icons** directory.

* At this time, you should be able to compile your lib and to add the function in a script using **papyrus**.

### Add Inputs to the Function ###

* Kheops defines some **Inputs** which could be add to **Functions**.
* In papyrus, Neural developpers can link the output of a Function with an input of another Function by adding an **iLink** between them.
* Each iLink have a *type* (**SCALAR** or **MATRIX**) and an associated **weight** also with a type (**SCALAR** or **MATRIX**).
* Becareful, **iLink type** (call **itype**) and **weight type** (call **wtype**) could be different !
* List of the iLink's type : **[WTYPE_ITYPE]** 
  1. **SCALAR_SCALAR** : iLink from **SCALAR type** with a **SCALAR weight**.
  2. **SCALAR_MATRIX** : iLink from **SCALAR type** with a **SCALAR weight** and the weight is global for every neurons of the Matrix.
  3. **MATRIX_MATRIX** : iLink from **SCALAR type** with a **SCALAR weight** (each neuron have it own weight)

* Kheops is **strongly typed**, so you can only link an output on an input with the same type.
* So, Input could be : 
  1. **IString** : Input receive a string.
  2. **ISInput** : Input receive **SCALAR_SCALAR** iLink.
  2. **ISMInput** : Input receive **SCALAR_MATRIX** iLink.   
  3. **IMMInput** : Input receive **MATRIX_MATRIX** iLink.  

* So, to add an Input to the HelloFunct class, we just have to declare the Input in the class definition.
* Let add a **ISMInput** called **inMat** and a **IString** calles **myString** : 

```javascript
class HelloFunct : public FMatrix
{
        private :     // Always declare the input as private members !
        
                ISMInput inMat;
                IString myString;
        
        public :

                HelloFunct();
                virtual ~HelloFunct();

                virtual void compute();
                virtual void setparameters();
};
```

* After that, we have to bind this Inputs in Kernel structure to let him manage iLink association.
* This operation is done in the **setparameters** function :

```javascript
void HelloFunct::setparameters() 
{
        Kernel::iBind(inMat,"inMat", getUuid());
        Kernel::iBind(myString,"myString", getUuid());
}
```

* For each input, you have to call the kernel **iBind** function :
 1. First parameter : reference to the input instance
 2. Second parameter : the name used by the kernel to find the input. This string has to be egal to the Input <name> in XML File (see below)


### Update XML Description file ###

* So, now we have to add, for Papyrus, the Input description in the XML file .

```xml
        <function>
                <name>HelloFunct</name>
                <output type="MATRIX">
                </output>
                <icon>filename</icon>
                <inputs>
                    <input type="SCALAR_MATRIX">
                        <name>inMat</name>
                    </input> 
                    <input type="STRING">
                        <name>myString</name>
                    </input> 
                </inputs>
        </function>
```

* For each input we had a <input> section inside the <inputs> section
* An Input has to define the attribute **type* (**STRING**, **SCALAR_SCALAR**, **SCALAR_MATRIX**, **MATRIX_MATRIX**)
* And it **name**. The **name value** is the **same** that the **second attribute** given to the **iBind** function !  

### Input with multiple iLink ###

* By default, an Input could receive only one iLink
* If you want to add more than one iLink on an Input you have to specify **multiple true**

* In **setparameters** function :

```javascript
void HelloFunct::setparameters() 
{
        inMat.setMultiple(true);   // inMat can now receive many iLink

        Kernel::iBind(inMat,"inMat", getUuid());
        Kernel::iBind(myString,"myString", getUuid());
}
```

* And in the XML File : 


```javascript
     <input type="SCALAR_MATRIX"  multiple="true">
        <name>inMat</name>
     </input> 
     <input type="STRING"  multiple="false">
       <name>myString</name>
     </input> 
```

* Note : you have to specify the attribute **multiple** in every case ! Even if input is false


###  MATRIX_MATRIX iLink ###

* Matrix_Matrix details : We have 3 types of connections
  1. One to All connections (ONE_TO_ALL) : Dense connections between input and output 
  2. One to One connections (ONE_TO_ONE) : Sparse conenction between input and output 
  3. One to Neighborhood connections (ONE_TO_NEI) : Sparse conenction between input and output 

### Local variable and load kernel ###


### Input and iLink operator ###

## Create its own repository ##

* demofct example 
  1. CMakeList.txt 

