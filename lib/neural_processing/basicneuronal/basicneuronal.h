/*
Copyright INSTAR Robotics

Author: Pierre Delarboulas

This software is governed by the CeCILL v2.1 license under French law and abiding by the rules of distribution of free software. 
You can use, modify and/ or redistribute the software under the terms of the CeCILL v2.1 license as circulated by CEA, CNRS and INRIA at the following URL "http://www.cecill.info".
As a counterpart to the access to the source code and  rights to copy, modify and redistribute granted by the license, 
users are provided only with a limited warranty and the software's author, the holder of the economic rights,  and the successive licensors have only limited liability.  
In this respect, the user's attention is drawn to the risks associated with loading, using, modifying and/or developing or reproducing the software by the user in light of its specific status of free software, 
that may mean  that it is complicated to manipulate, and that also therefore means that it is reserved for developers and experienced professionals having in-depth computer knowledge. 
Users are therefore encouraged to load and test the software's suitability as regards their requirements in conditions enabling the security of their systems and/or data to be ensured 
and, more generally, to use and operate it in the same conditions as regards security. 
The fact that you are presently reading this means that you have had knowledge of the CeCILL v2.1 license and that you accept its terms.
*/


#ifndef _BASIC_NEURONAL_H_
#define _BASIC_NEURONAL_H_

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

/*

// SPARSE_MATRIX -> Nouveau type de liens 
neurons concatenation -> OK 
extract neurons -> OK 

projection (
	neuron-to-vector -> SCALAR-SCALAR 
	vector-to-neuron  -> SCALAR_MATRIX
) (circularity option)

-> Concaténation + extraction : meme opération de projection : on fait une seule boite de projection qui prend en input une sparse matrice définissant les régles de projections 


convolution operator (circularity option) -> OK simple 

N_MAX/N_MIN (reviens à un sort si N = taille entrée)  -> implémenter plus tard si besoin
-> SORT ?? Est-ce que ce n'est pas plus malin de l'appeler SORT ? Avec N -> N premier (soit max soit min)

*/

/*******************************************************************************************************/
/*********************************************  Keep_Max   *********************************************/
/*******************************************************************************************************/

// Keep N max in the output Matrix
// Output must have same dimension that inMatrix
// nMax : number of max keep in output
class KeepMax : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput nMax;

        public :

                virtual ~KeepMax(){}

                virtual void compute();
                virtual void setparameters();
};

// Keep N max in the output Matrix
// Output must have same dimension that inMatrix
// nMax : number of min keep in output
class KeepMin : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput nMin;

        public :

                virtual ~KeepMin(){}

                virtual void compute();
                virtual void setparameters();
};


/*******************************************************************************************************/
/*********************************************  ActToPop   *********************************************/
/*******************************************************************************************************/
/*
 * Activity to population :  Single Neurons to Vector 
 * Value must be in [0,1] interval
 * Output must be a Vector (rows or cols)
*/
class ActToPop : public FMatrix
{
	private : 
                
		ISInput activity;
		MatrixXd::Index lastIndex;
        
	public :

		ActToPop() : FMatrix(VECTOR), lastIndex(0) {}
                virtual ~ActToPop(){}

                virtual void compute();
                virtual void setparameters();
};


/*******************************************************************************************************/
/********************************************  VActToPop   *********************************************/
/*******************************************************************************************************/

/*
 * Activity to population :  Vector to Matrix (each neurons from the input Vector is discretize on a row or a col of the output Matrix [ROW or COL depend on the input] 
 * Each Value from the input vectur must be in [0,1] interval
 * Output must be a Matrix with rows (or cols) egal to the size of the input Vector
*/

enum PROJ{COLPROJ,ROWPROJ,SINGLEV};

class  VActToPop : public FMatrix
{
	private : 
                
		ISMInput activities;

		unsigned int proj;
		VectorXd lastIndex;
        
	public :

		VActToPop() : proj(0) {}
                virtual ~VActToPop(){}

                virtual void compute();
                virtual void setparameters();
		virtual void prerun();
};

/*******************************************************************************************************/
/*********************************************  PopToAct   *********************************************/
/*******************************************************************************************************/
/*
 * Population to Activity : Take the index of the max coefficient and convert the index into a scalar
 * If there are more than one max coefficient, takes the index of the first one
 * Output is a scalar between [0,1]
 * Input must be a Vector [ROW or COL]
 *
 * row/col projection is choosen according to the dimension of output. if output is a ROW Vector : col projection, if output is a COL Vector : row projection
*/
class PopToAct : public FScalar
{
	private : 
		ISMInput population;
	
	public :

                virtual ~PopToAct(){}

                virtual void compute();
                virtual void setparameters();
		virtual void prerun();
};

/*******************************************************************************************************/
/********************************************  PopToVAct   *********************************************/
/*******************************************************************************************************/
/*
 * Population to activities : 
 * For each row (or col), take the index of the max coefficient and convert the index into a scalar
 * If there are more than one max coefficient, takes the index of the first one
 * Output is a vector where each neurons have an activity between [0,1]
 * Input must be a Matrix with rows (or cols) egals to the dimensions of the output vector
 *
 * row/col projection is choosen according to the dimension of output. if output is a ROW Vector : col projection, if output is a COL Vector : row projection
*/
class  PopToVAct : public FMatrix
{
        private :
                ISMInput population;
		unsigned int proj;

        public :

		PopToVAct() : FMatrix(VECTOR),proj(0) {}
                virtual ~PopToVAct(){}

                virtual void compute();
                virtual void setparameters();
                virtual void prerun();
};

/*******************************************************************************************************/
/*******************************************  Convolution   ********************************************/
/*******************************************************************************************************/

class Convolution : public FMatrix
{
	private : 
		ISMInput inMatrix;
		ISMInput mask;
		
        public :
		Convolution() {}

		virtual ~Convolution(){}
		virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/**********************************************  Shift   ***********************************************/
/*******************************************************************************************************/

template<class ArgType>
class Shift_functor {
  const ArgType &input;
  const typename ArgType::Index &sx;
  const typename ArgType::Index &sy;
  const typename ArgType::Index &max_x;
  const typename ArgType::Index &max_y;
  const typename ArgType::Index &inv;
public:
  Shift_functor(const ArgType &input , const typename ArgType::Index& sx , const typename ArgType::Index& sy,  const typename ArgType::Index& max_x , const typename ArgType::Index& max_y, const typename ArgType::Index &inv) : input(input), sx(sx),sy(sy),max_x(max_x), max_y(max_y), inv(inv) {}
  
  const  typename ArgType::Scalar operator() (Index row, Index col) const {
	const typename ArgType::Index  sr = (row - (sy * inv) + max_y) % max_y ; 
	const typename ArgType::Index  sc = (col - (sx * inv) + max_x) % max_x ; 

        return  input(sr,sc);
  }
};

class Shift : public FMatrix
{
	private :

		ISMInput inMatrix;
		ISMInput mask;

	public : 

		virtual ~Shift(){}
		virtual void compute();
                virtual void setparameters();
};

class ShiftInv : public FMatrix
{
	private :

		ISMInput inMatrix;
		ISMInput mask;

	public : 

		virtual ~ShiftInv(){}
		virtual void compute();
                virtual void setparameters();
};

#endif // _BASIC_NEURONAL_H_
