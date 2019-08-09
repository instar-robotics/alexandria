/*
  Copyright (C) INSTAR Robotics

  Author: Pierre Delarboulas
 
  This file is part of alexandria <https://github.com/instar-robotics/alexandria>.
 
  alexandria is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  alexandria is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _BASIC_NEURONAL_H_
#define _BASIC_NEURONAL_H_

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

/*
TODO : 
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
		MATRIX::Index lastIndex;
        
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
		VectorXs lastIndex;
        
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

template<class ArgType>
class Conv_functor{

	const ArgType &I;
	const ArgType &mask;
	bool circular;

	public : 

	Conv_functor(const ArgType& I, const ArgType& mask, bool circular) : I(I), mask(mask), circular(circular) {}

	const typename ArgType::Scalar operator() (Index irow, Index icol) const {

		typename ArgType::Index mrows = mask.rows();
		typename ArgType::Index mcols = mask.cols();

		typename ArgType::Index irows = I.rows();
		typename ArgType::Index icols = I.cols();

		typename ArgType::Scalar value = 0.0;

	   	for( typename ArgType::Index i = 0; i < mrows; i++)
         	{
                 for( typename ArgType::Index j = 0; j < mcols; j++)
                 {
												typename ArgType::Index zrow = irow + i+mrows%2 - (mrows+mrows%2)/2.0 ; // rule differs whether mrows is odd (mrows%2!=0) or even (mrows%2=0)
                        typename ArgType::Index zcol = icol + j+mcols%2 - (mcols+mcols%2)/2.0 ; // same for mcols

                        if( circular )
                        {
                                zrow = (int)(zrow + irows) % irows;
                                zcol = (int)(zcol + icols) % icols;
                        }

			if( zrow >= 0 && zrow < irows && zcol >= 0 && zcol < icols)
			{
				value += mask(i,j) * I(zrow,zcol);	
			}
		 }
		}
		return value;
	}
};

class Convolution : public FMatrix
{
	private : 
		ISInput circular;
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
	typename ArgType::Index  sr = ((row - (sy * inv) ) % max_y + max_y ) % max_y ; 
	typename ArgType::Index  sc = ((col - (sx * inv) ) % max_x + max_x ) % max_x ; 

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

/*******************************************************************************************************/
/************************************************  Copy  ***********************************************/
/*******************************************************************************************************/

template<class ArgType>
class Copy_functor {
  const ArgType &input;
  const typename ArgType::Index &sx;
  const typename ArgType::Index &sy;
  const typename ArgType::Index &max_x;
  const typename ArgType::Index &max_y;
public:
  Copy_functor(const ArgType &input , const typename ArgType::Index& sx , const typename ArgType::Index& sy,  const typename ArgType::Index& max_x , const typename ArgType::Index& max_y) : input(input), sx(sx),sy(sy),max_x(max_x), max_y(max_y) {}

  const  typename ArgType::Scalar operator() (Index row, Index col) const {
        typename ArgType::Index  sr = ((row + sy ) % max_y + max_y ) % max_y ;
        typename ArgType::Index  sc = ((col + sx ) % max_x + max_x ) % max_x ;

        return  input(sr,sc);
  }
};


class Copy : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISMInput dirac;

        public :

                virtual ~Copy(){}
                virtual void compute();
                virtual void setparameters();
};


/*******************************************************************************************************/
/********************************************  Projection  *********************************************/
/*******************************************************************************************************/

class Projection : public FMatrix
{	
	private :
		IMMInput inMatrix;
	
	public :
		virtual ~Projection(){}
		virtual void compute();
                virtual void setparameters();
};

#endif // _BASIC_NEURONAL_H_
