![Alt text](icon/alexandria_icon.png?raw=true "Title")

# README #

## I Description ##

* Alexandria is a part of the Kheops/Papyrus software.  
  1. Alexandria contains a collection of Functions for the neural networks simulator [Kheops](https://github.com/instar-robotics/kheops)
  2. Alexandria contains XML description files for the [Papyrus](https://github.com/instar-robotics/papyrus) GUI

## II Installation ##

Please see [Papyrus's how-to-install](https://github.com/instar-robotics/papyrus/blob/master/README.org#how-to-install) tutorial for a Kheops/Papyrus full installation.

or see [Kheops's standalone-install] tutorial for a Kheops standalone installation

### Dependancy ###

* Alexandria requiers  :
   1. [Kheops](https://github.com/instar-robotics/kheops/blob/master/README.md) and all its dependancies (included ROS)
   2. the ROS joy package (on ubuntu/debian : ros-melodic-joy)
   3. the ROS tf2 package (on ubuntu/debian : ros-melodic-tf2)
   4. the ROS nav_msgs package (on ubuntu/debian : ros-melodic-nav-msgs)
   5. the ROS sensor_msgs package (on ubuntu/debian : ros-melodic-sensor_msgs)
   6. the ROS geometry_msgs package (on ubuntu/debian : ros-melodic-geometry_msgs)
   
### Libraries path ###

* For **Papyrus users** : 
  1. By default, libraries are copied in $CMAKE_INSTALL_PREFIX/lib/alexandria
  3. CMAKE_INSTALL_PREFIX default value is the install directory in your catkin_workspace 
  4. You can set CMAKE_INSTALL_PREFIX before running **_catkin_make_ install**
  5. You have to give this path if you run Kheops in standalone mode :
  
```console
$> rosrun kheops kheops -s MyScript.xml -l $CMAKE_INSTALL_PREFIX/lib/alexandria
```

  1. Alexandria copies XML description files in $CMAKE_INSTALL_PREFIX/share/alexandria/description
  2. When you launch Payrus for the first time, Papyrus asks you to set this path.

* For **Functions Developpers** : 
  1. If you code ant test some news functions you probably doesn't want to install Alexandria at each build time.
  2. So, you could only run : 

```console
$> catkin_make
```

  1. Libraries are copied in $CATKIN_DEVEL_PREFIX/lib/alexandria
  2. XML desription files are copied in $CATKIN_DEVEL_PREFIX/share/alexandria/description
  3. If you want update XML Description without rebuild all lib, you can run : 

```console
$> catkin_make all_desc
```

## Functions developper's guide ##

* Note : kheops uses Eigen library to manage Linear Algebra. 
* We hardly recommand to read Eigen's doc and try the library before writing Alexandria's Function ! 

### Write a first Function : General description ###

* Functions are defined by some objects : 
  1. .h file : contains the C++ header of the Function
  2. .cpp file : contains the C++ code of the Function
  3. .xml file : contains the description of the Function (For papyrus)
  4. icons directory : contains the SVG file for Function icon in payrus

* First you have to create a directory and the 3 empties files in the **lib** directory of Alexandria
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

* The code is pretty simple. We have a HelloFunct class which print "Hello world" on the standard output at each kernel iteration.
* Macro **_REGISTER_FUNCTION_** is mandatory to register the HelloFunct into the kernel.

* At this time, you should be able to compile the Function and find your **_libhellofunct.so_** 
* Just run, in your catkin workspace : 
  
**_catkin_make_** 

* And look in devel/lib/alexandria, you should find the **_libhellofunct.so_** 

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
* The output variable could be linked on another input Function using an iLink.

### Add Inputs to the Function ###

* Kheops defines some **Inputs** which could be add to **Functions**.
* In papyrus, Neural developpers can link the output of a Function with an input of another Function by adding an **iLink** between them.
* Each iLink have a *type* (**SCALAR** or **MATRIX**) and an associated **weight** also with a type (**SCALAR** or **MATRIX**).
* Becareful, **iLink type** (call **itype**) and **weight type** (call **wtype**) could be different !
* List of the iLink's type : **[WTYPE_ITYPE]** 
  1. **IScalar : [SCALAR_SCALAR]**  iLink from **SCALAR type** with a **SCALAR weight**.
  2. **ISMatrix : [SCALAR_MATRIX]**  iLink from **SCALAR type** with a **SCALAR weight** and the weight is global for every neurons of the Matrix.
  3. **IMMatrix : [MATRIX_MATRIX]** iLink from **SCALAR type** with a **SCALAR weight** (each neuron have it own weight)

* Kheops is **strongly typed**, so you can only link an output on an input with the same type.
* Input could be : 
  1. **IString** : Input receive a string.
  2. **ISInput** : Input receive **SCALAR_SCALAR** iLink.
  2. **ISMInput** : Input receive **SCALAR_MATRIX** iLink.   
  3. **IMMInput** : Input receive **MATRIX_MATRIX** iLink.  

* Input receive iLink to give acces to Function's output :
   1. iLink contains a **const pointer** to the output of the Function predecessor.
   2. iLink are defined in the XML script file generated by Papyrus 
   3. kheops build the graph when reading this file.

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
 3. Third parameter : the UUID of the function  

* Note : in kheops all the objects in the graph are identified with UUID (see kernel documentation)

### Update XML Description file ###

* So, now we have to add the Input description in the XML file :

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

* We first add an <inputs> sectio containing all the input definition
* Then for each input we had a <input> section containing the definition of this input.
* An Input has to define a **type** attribute (**STRING**, **SCALAR_SCALAR**, **SCALAR_MATRIX**, **MATRIX_MATRIX**)
* And a **name** field.

* NOTE : The **name value** MUST BE identical to the **second attribute** given to the **iBind** function !  

### Input with multiple iLink ###

* By default, an Input could receive only one iLink
* If you want to add more than one iLink on an Input you have to specify that the Input is multiple.
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
     <input type="STRING">
       <name>myString</name>
     </input> 
```

### Check Size for Input ###

* By default when you add an input to a function, kheops check that the dimension of object from iLink is egal to the dimension of the function output.
* By default : **checkSize** is **true**
* For Scalar input, check size is ignore (because it is a non-sense to check 1-D object)
* You could inhibate this behaviour by calling **setCheckSize** in **setparameters** function : 

```javascript
void HelloFunct::setparameters() 
{
        inMat.setCheckSize(false);   // kheops will not check size for inMat

        Kernel::iBind(inMat,"inMat", getUuid());
        Kernel::iBind(myString,"myString", getUuid());
}
```

* And in the XML File : 

```xml
     <input type="SCALAR_MATRIX"  checkSize="false">
        <name>inMat</name>
     </input> 
     <input type="STRING">
       <name>myString</name>
     </input> 
```

### kernel loading parameters ###

* We describe **setparameters** and **prerun** functions.
* Each function is called by the kernel at a specific moment.

* **setparameters** :
  1. is called when the kernel load the Function. 
  2. At this moment, the graph is not built (iLink is not created)
  3. So. Input variable can't acces to the Output data from the predecessor Function (from iLink) 
  4. In case of **FMatrix**, the output dimension is set an the MatrixXd is initialize 
  5. Output is set to zero (SCALAR and MATRIX)

* **prerun** :
  1. is called after the kernel load the graph
  2. At this moment, the graph is complete and each input can acces to the Output data from the predecessor Function (from iLink)

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
* Depend on what you need :
 1. if you need only output information : you can initialize your attribute in **setparameters**
 2. if you need some input informaton : you have to initailize your attribute in **prerun**

* For example, you add a MatrixXd **tempMat** with the same size that **output Matrix**.
* To initialize **tempMat** you ony need to add : 

```javascript
void HelloFunct::setparameters() 
{
        inMat.setMultiple(true);   // inMat can now receive many iLink

        Kernel::iBind(inMat,"inMat", getUuid());
        Kernel::iBind(myString,"myString", getUuid());
        
        tempMat = MatrixXd::Constant(output.rows(),output.cols(),0);  // create a tempMat with the same dimension that output 
}
```
* Now, you need to create a tempMat with the same dimension than Input dimension.

```javascript
void HelloFunct::prerun() 
{
    tempMat = MatrixXd::Constant(inMat().i().rows(),inMat().i().cols(),0);  // create a tempMat with the same dimension that Input 
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
 
```javascript 

    ISInput in;

    int size = in.size();                // return the number of iLink contains in the input
    size_t type = in.type();             // return a hashcode defining the iLink type   
    std::string tname = in.type_name();  // return the name of the iLink type
    bool mul = in.isMultiple()           // return True if the Input accept more than one iLink
    inMat.setMultiple(true);             // set Multiple flag to True
    
    // This calls are equivalent : 
    in.i();                           // Return a reference to the first iLink 
    in();                             // Return a reference to the first iLink
    in.i(0);                          // Return a reference to the first iLink 
    in(0);                            // Return a reference to the first iLink


    // This both calls are equivalent : 
    in.i(2);                          // Return a reference to the third iLink 
    in(2);                            // Return a reference to the third iLink

```
* Once you could acces to iLink inside the Input container, you need operator to acces to input and weight values.

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

* Access to the iLink's predecessor output data : [data are Read Only ! A function can't modify the output of a predecessor]
  1. i() : return a const pointer to the data.

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
  1. operator() : return input * weight  
  2. operator(MatrixXd & res) : copy in res =  input * weight  [Better than 1.]
  2. operator+=(double, iSMatrix) : copy  double + input * weight     
  3. operator-=(double, iSMatrix) : copy  double - input * weight 
  4. operator/=(double, iSMatrix) : copy  double / input * weight 
  5. operator*=(double, iSMatrix) : copy  double * input * weight 

* **HOW TO USE THIS OPERATORS** :

* Example : Weighted sum of N iScalar 
```javascript 
    ISInput in;   // Input containing some iScalar 
  
    // Here get the first iScalar 
    // then apply operator() to compute input * weight
    // then call basic opertor=(double,double)      
    double output = in(0)();   // equivalent to in()()
    
    for(unsigned int n=1; i < in.size(); i++)
    {
        // Best practice 
        // call operator(int n) of Input class to get the N iScalar
        // then call operator+= for iScalar class 
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
  
    MatrixXd output; //Declare an Eigen Matrix
    
    output = in(0)();  // Return I * W and init output

    // Another solution : Here get the first iMarix and apply operator(MatrixXd) to compute input * weight and copy the result in output. 
    // in(0)(output);   

    
    for(unsigned int n=1; i < in.size(); i++)
    {
        //This 3 syntax are equivalent in performance
        
        // use operator(int n) of Input class to get the N iSMatrix
        // then call operator+= for iSMatrix class [ double + input * weight ]
        output += in(n) ;    
        
       // You can use :  
        output += in(n)()   
        
        // Alternative :  
        output += in(n).i() * in(n).w();   // or in.i(n).i() * in.i(n).w(); 
         
     }
```

#### iMMatrix operators ####

* We hardly recommand to read and try Eigen Library before using iMMatrix !

* iMMatrix is a more complicate iLink, this class defines a lot of operator to manipulate the Weighted Matrix. 
* **Input API** : you can use i() operator to acces the Matrix from predecessor Function
* But iMMatrix defines some useful operator : 
  1. **irow()** : return an Eigen Map in row form [one line Matrix]
  2. **icol()** : return an Eigen Map in col form [one col Matrix]
  3. **ivect()** : return an Eigne Map in vector form [a col Vector] 


* We assume that Matrix linked by a iMMatrix link could have different dimensions :
  1. Input Matrix have IRows and ICols dimensions
  2. Output Matrix have ORows and OCols dimensions

* The dimensions of the weighted matrix are : 
  1. **WRow =  IRows * ICols**
  2. **WCol =  ORows * OCols**

* **Weight API** : [you can use w() and w(MatrixXd&) operator, but iMMatrix defines some BETTER operators]
  1. wref(const Ref<const MatrixXd>& weight) : gives more generalization abilities and better performance than w(MatrixXd)
  2. wm() :  Return a Map of the weight Matrix 
  3. wj(oRows, oCols) : Return a Map for the neuron oRows,oCols. The returned Matrix have iRows,iCols dimension           
  4. wj(wCols) :  Return a weight colonn for the output neuron (wCols). The Matrix have (wRows,1) dimension
  5. wj_vect(unsigned int wCols); Return a weight colonn for the output neuron (wCols). The returned object is a Vector
  6. wij(wRows,wCols) : return the value of the weighted neuron (wRows,wCols)
  7. wij(double weight, unsigned int wRow, unsigned int wCol) : set the value of the weigthed neuron wRow,wCol
  8. wj(const Ref<VectorXd> &weight,unsigned int wCol) : set the value for the weigthed colonn wCol

* **getMap Functions**: 
* To help the Matrix manipulation we provide getMap functions.
* This functions built map of a given Matrix in 3 differents forms : 
  1. **getMapRow**( Matrix )  : return a Map in row form (1, Row*Col) 
  2. **getMapCol**( Matrix )  : return a Map in col form (Row*Col, 1)
  3. **getMapVect**( Matrix )  : return a Map in Vector form 
* We also provide const Form for each a this function : **getCMapRow**, **getCMapCol**, **getCMapVect**

* Example :

```javascript 

 // A very common operation in Neural Network is computed the Weighed Sum of an Input Matrix and a Weigthed Matrix
 
 IMMInput im;  // The container of all the input Matrix
 MatrixXd out = MatrixXd::Constant(oRow,oCol,0) ;  // output with oRow and oCol dimension 
 
 ...

 auto mout = getMapRow(output);  // We transform the square output Matrix in 1 row Matrix (Vector)

  // Compute output activity
  mout = im(0).irow()  * im(0).w();    // We use Matricial Product which provide directly the weigthed sum !
  for(unsigned int i=1; i < im.size(); i++)
  {
       mout += im(i).irow() * im(i).w();   // We repeat the operation for each iMMatrix
  }

  std::cout << out << std::endl;       // We print the result 
```

* NOTE : a Map share the same data that the original Matrix 

* iMMatrix is used to implements all the connectivity type : 
  1. **ONE_TO_ONE** : point to point connection
  2. **ONE_TO_ALL** : input neurons are linked with all output neurons
  3. **ONE_TO_NEI** : inuts neurosn have sparse hybrid connectivity

* To express the connectivity we use a **Filter Matrix**.
* A **Filter Matrix** is a boolean Matrix with the same dimension that the **Weighted Matrix** :
  1. The i,j value inside the Filter Matrix is egal to True, if the input neuron j is connected with the output neuron i
  2. And False otherwise.
* Filter Matrix is automatically built by the kernel follow the connectivity flag : 
    1. **ONE_TO_ONE** : The Filter Matrix is the identity matrix
    2. **ONE_TO_ALL** : The Filter Matrix is a full matrix
    3. **ONE_TO_NEI** : depend on the neighborhood

* To maintain a corherent **Weigthed Matrix** you have to apply manually the **filter** operator.
* You can do it : 
  1. When you access to the weight 
  2. When you update the weight
* There are not an absolute good strategy, depends on the different algorithms.


* The **Filter Matrix** allow devellopers to write algorithms without manage the connectivity problems.
* Example :

```javascript 

    // A very classical algorithm is performing a gradiant descent 
    // Knowing an input X, an output value and an example Y of a dataset

        double learning_rate
        MatrixXd out, Y, grad, weight;
        iMMatrix X;
        
        ...
        
        // Compute gradiant
        grad = learning_rate * (Y - out);
        auto vgrad = getMapRow(grad);

        auto ve = X.icol(); 
        auto w = X.wm();
        auto f = X.fm();

        // Update weight 
        w.noalias() = filter( ve * vgrad , f );  // compute the gradiant descent ve * grad, then apply filter f and copy the result in w.
    
```

#### ONE_TO_NEI connectivity ####

* **ONE_TO_NEI** connectivity must be describe in each link using a kind of regex expression.
* In each IMMATRIX link, when you choose a ONE_TO_NEI connectivity, you can add expression to describe the connectivity.
* The number of expression for each link is unlimited.

* The expression have the following shape : 
```javascript 
 [(src)op(dst)](prop{offset};prop{offset})rep
```

* src : describe a block in the source matrix
* dst : describe a block in the destination matrix
* op : describe the connectivity between the source and destination block
* prop : describe the propagation method in the matrix. left prop is for source matrix, right prop is for the destination matrix.
* rep : describe how many times the block are propagated 

* NOTE : propagation part is optional. Without source is only copie once.

* src/dst :  row,col,height,width  [each value is ann integer]
* op : "." or "x"   
   1. "." : **ONE_TO_ONE** connectivity
   2. "x" : **ONE_TO_ALL** connectivity
* prop : "r","c","d","n"
   1. "r" : row propagation
   2. "c" : col propagation
   3. "d" : diagonnal propagation
   4. "n" : no propagation
* {offset} : is optionnal. without value, offset is egal to "height" or "width" according to prop option. 
* rep : the number of time block are propagated. Could be an integer or "*" (repeat until reaching the end of matrix). 
 
```javascript 

// extract the third value of the source row vector on the first neuron of the destination matrix
[(0,2,1,1).(0,0,1,1)]

// modulo : copy the first two neuron of the source matrix on the first two neuron of the destination matrix
// then offset the source matrix of 2 two columns and repeat the copy until reaching the end of the source matrix
[(0,0,1,2).(0,0,1,2)](c,n)*

```
