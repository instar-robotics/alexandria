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

* Note : kheops uses Eigen library to manage Linear Algebra. 
* We hardly recommand to read Eigen's doc and try the library beafor writing kheops's Function ! 

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

### Accces to the Function's output ###

* Every Function has an **output** variable with the same type than the Function.
  1. For FScalar : output is a **double**
  2. For FMatrix : output is an EigenDense **MatrixXd**

* You can acces to this variable in everry function. 
* **output** is initialize automatically in the **setparameters** function (see below).

### Add Inputs to the Function ###

* Kheops defines some **Inputs** which could be add to **Functions**.
* In papyrus, Neural developpers can link the output of a Function with an input of another Function by adding an **iLink** between them.
* Each iLink have a *type* (**SCALAR** or **MATRIX**) and an associated **weight** also with a type (**SCALAR** or **MATRIX**).
* Becareful, **iLink type** (call **itype**) and **weight type** (call **wtype**) could be different !
* List of the iLink's type : **[WTYPE_ITYPE]** 
  1. **IScalar : **[SCALAR_SCALAR]**  iLink from **SCALAR type** with a **SCALAR weight**.
  2. **ISMatrix : **[SCALAR_MATRIX]**  iLink from **SCALAR type** with a **SCALAR weight** and the weight is global for every neurons of the Matrix.
  3. **IMMatrix : **[MATRIX_MATRIX]** iLink from **SCALAR type** with a **SCALAR weight** (each neuron have it own weight)

* Kheops is **strongly typed**, so you can only link an output on an input with the same type.
* Input could be : 
  1. **IString** : Input receive a string.
  2. **ISInput** : Input receive **SCALAR_SCALAR** iLink.
  2. **ISMInput** : Input receive **SCALAR_MATRIX** iLink.   
  3. **IMMInput** : Input receive **MATRIX_MATRIX** iLink.  

* Input receive iLink to give acces to Function output.
* iLink contains a const pointer to the output of the Function.
* The graph is contains in the XML script file generated by Papyrus and kheops build the graph when reading this file.

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


```xml
     <input type="SCALAR_MATRIX"  multiple="true">
        <name>inMat</name>
     </input> 
     <input type="STRING"  multiple="false">
       <name>myString</name>
     </input> 
```

* Note : you have to specify the attribute **multiple** in every case ! Even if input is false


### kernel loading parameters ###

* We describe **setparameters** and **prerun**
* Each function is called by the kernel at a specific moment.

* **setparameters** :
  1. is called after the kernel load every Function. 
  2. At this moment, the graph is not built (iLink is not created), only Function
  3. So. at this moment, Input variable can't acces to the Output data from the predecessor Function (from iLink) 
  4. In case of **FMatrix**, the output dimension is set an the MatrixXd is initialize 
  5. Output is set to zero (SCALAR and MATRIX)

* **prerun** :
  1. is called after the kernel load the graph
  2. At this moment, the graph is complete and each input can acces to the Output data from the predessor Function (from iLink)

### class member and initialisation ###

* You can add as many class member (attribute and/or function). This is standard C++

```javascript
class HelloFunct : public FMatrix
{
        private :     // Always declare the input as private members !
        
                ISMInput inMat;
                IString myString;
            
                MatrixXd tempMat;
                
        public : 
        
                void my_super_fct();
};
```

* But if you need to initialize attribute member with Input/Output parameters, you have to do in **setparameters** or **prerun** function.
* Depend on what you need !**prerun**

 1. if you need output information : you can initialize your attribute in **setparameters**
 2. if you need input informaton : you have to initailize your attribute in **prerun**


* For example, you add a MatrixXd **tempMat** with the same size that **output Matrix**.
* To initialize **tempMat** you ony need to add : 

```javascript
void HelloFunct::setparameters() 
{
        inMat.setMultiple(true);   // inMat can now receive many iLink

        Kernel::iBind(inMat,"inMat", getUuid());
        Kernel::iBind(myString,"myString", getUuid());
        
        tempMat = MatrixXd::Constant(output.rows(),output.cols(),0);  // create a tempMat with the same dimension that output with 0 everywhere
}
```
* Now, you need to create a tempMat with the same dimension than Input dimension.

```javascript
void HelloFunct::prerun() 
{
    tempMat = MatrixXd::Constant(inMat().i().rows(),inMat().i().cols(),0);  // create a tempMat with the same dimension that Input with 0 everywhere
}
```

* Notre : see the section below for operator explanation !

### Input and iLink operators ###

#### Input's operators ####

* Input are just an iLink container with a given type. 
* For example, ISInput is simply : 

```javascript 
typedef Input<iScalar> ISInput;
```

* Input defines some operator to acces to the iLinks : 
  1. size() : return the number of iLink contains in the input
  2. type() : return a hashcode defining the iLink type
  3. type_name() : return the name of the iLink type
  4. isMultiple() : return True if the Input accept more than one iLink
  5. i()  : return a reference to the first iLink [use this operaor when Input is not multiple]
  6. operator() : return a reference to the first iLink [use this operaor when Input is not multiple]
  7. i(int n)  : return a reference to the n iLink [use this operaor when Input is multiple]
  8. operator(int n) : return a reference to the n ilink  [use this operaor when Input is multiple]

* Example : 

```javascript 

    ISInput in;
    
    // This calls are equivalent : 
    in.i();            // Return a reference to the first iLink 
    in();              // Return a reference to the first iLink
    in.i(0);            // Return a reference to the first iLink 
    in(0);              // Return a reference to the first iLink


    // This both calls are equivalent : 
    in.i(2);            // Return a reference to the third iLink 
    in(2);              // Return a reference to the third iLink

```

#### iLink's operators ####

* iLink defines some common operators shared by iLink class (iScalar, iSMatrix and iMMatrix)
* Type of the iLink : 
  1. i_type() : return a hashcode defining the type of the iLink (SCALAR or MATRIX)
  2. i_type_name() : return the name of the type of the iLink (SCALAR or MATRIX)
  3. w_type() : return a hashcode defining the type of the iLink's weight (SCALAR or MATRIX)
  4. w_type_name() : return the name of the type of the iLink's weight (SCALAR or MATRIX)

* Output the iLink : [Becareful when using this functions ! Kernel can active or stop the topic when receiving externals orders !]
  1. active_publish(bool) : start to publish the iLink weight 
  2. is_publish_active() : return True or False if the topic is active.
  3. publish_message() : publish the output

* Access to the iLink's predecessor output data : [data are Read Only ! A function can't modify the outpu of a predecessor]
  1. i() : return a pointer to the data.

* Access to the iLink's weight :
  1. w() : return a reference to the weight
  2. w(W &w) : set the weight [Note : W is a template type, SCALAR or MATRIX]

* iScalar operators : 
  1. operator() :  return input * weight
  2. operator+=(double, iScalar) : return double + input * weight     
  3. operator-=(double, iScalar) : return double - input * weight 
  4. operator/=(double, iScalar) : return double / input * weight 
  5. operator*=(double, iScalar) : return double * input * weight 

* iSMatrix operators : 
  1. operator() : return input * weight  [Becareful : return a temporary MatrixXd ! So avoid this operator if possible]
  2. operator(MatrixXd & res) : copy in res =  input * weight  [Better than 1.]
  2. operator+=(double, iSMatrix) : copy  double + input * weight     
  3. operator-=(double, iSMatrix) : copy  double - input * weight 
  4. operator/=(double, iSMatrix) : copy  double / input * weight 
  5. operator*=(double, iSMatrix) : copy  double * input * weight 

* **HOW TO USE THIS OPERATORS** :

* Example : Weighted sum of N iScalar 
```javascript 
    ISInput in;   // Input containing some iScalar 
  
    // Here get the first iScalar and apply operator() to compute input * weight, then call basic opertor=(double,double)      
    double output = in(0)();   // equivalent to inScalar()()
    

    for(unsigned int n=1; i < in.size(); i++)
    {
        // Best practice 
        // call operator(int n) of Input class to get the N iScalar
        // then call operator+= for iScalar class [ double + input * weight ]
        output += in(n) ;    
        
        // Alternative :  
        // call operator(int n) of Input class to get the N iScalar
        // then call operator() for iScalar class [return input * weight]
        // then call basic operator+=(double,double) 
        output += in(n)()   
        
        // Alternative :  
        output += in(n).i() * in(n).w();  // or in.i(n).i() * in.i(n).w(); 
     }
```

* Example : Weighted sum of N iSMatrix 

```javascript 
    ISMInput in;   // Input containing some iScalar 
  
    MatrixXd output; //Declare an Eigne Matrix
    
    // Good solution : 
    // Here get the first iMarix and apply operator(MatrixXd) to compute input * weight and copy the result in output. 
    in(0)(output);   

    // Wrong solution !! 
    output = in(0)();  // Here you use a temporary MatrixXd before copying into output !

    for(unsigned int n=1; i < in.size(); i++)
    {
        // Best practice 
        // use operator(int n) of Input class to get the N iSMatrix
        // then call operator+= for iSMatrix class [ double + input * weight ]
        output += in(n) ;    
        
       // Wrong solution !! 
        output += in(n)()   // Here you use a temporary MatrixXd before copying into output !
        
        // Alternative :  
        output += in(n).i() * in(n).w();   // or in.i(n).i() * in.i(n).w(); 
         
     }
```

#### iMMatrix operators ####

* We hardly recommand to read and try Eigen Library before using iMMatrix !

* iMMatrix is a more complicate iLink, this class define a lot of operator to manipulate the Weighted Matrix. 


* Representation : 
* Matrix IN [From the Function predecessor]  --> Matrix OUT [output of the current Function]  

* Input Matrix have IRows and ICols dimensions
* Output Matrix have ORows and OCols dimensions
* Weight Matrix have IRows * ICols  and ORows * OCols
* Filter Matrix have IRows * ICols  and ORows * OCols

* Input API : you can use i() operator to acces the Matrix from predecessor Function
* But iMMatrix defines some useful operator : 
  1. irow() : return an Eigen Map in row form [one line Matrix]
  2. icol() : return an Eigen Map in col form [one col Matrix]
  3. ivect() : return an Eigne Map in vector form [a col Vector]

* Weight API : [you can use w() and w(MatrixXd&) operator, but iMMatrix defines some BETTER operators]
  1. wref(const Ref<const MatrixXd>& weight) : gives more generalization abilities and better performance than w(MatrixXd)
  2. wm() : 
  3.           // Get Weight matrix for the output neuron (oRows,oCols)
                // The Matrix have (iRows,iCols) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXd> wj(unsigned int oRows,unsigned int oCols);

                // Get Weight colons for the output neuron (wCols)
                // The Matrix have (wRows,1) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXd> wj(unsigned int wCols);

                // Get Weight colons for the output neuron (wCols)
                // Becareful : the returned Map is writable !
                Map<VectorXd> wj_vect(unsigned int wCols);

                double wij(unsigned int wRows,unsigned int wCols);

                //Set Weight Value
                void wij(double weight, unsigned int wRow, unsigned int wCol);
                void wj(const Ref<VectorXd> &weight,unsigned int wCol);



## Create its own repository ##

* TODO 
* See demofct example in user_src
  1. CMakeList.txt 

