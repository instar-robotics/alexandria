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


#include "basicmaths.h"
#include <cmath>
#include <algorithm>

/*******************************************************************************************************/
/**********************************************  ArgMax  ***********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(ArgMax1D);

void ArgMax1D::upreload()
{
	if( inMatrix().i().rows() != 1 && inMatrix().i().cols() != 1 ) 
	{
		throw std::invalid_argument("ArgMax1D Function : input have to be a vector (1D Matrix) !");
	}
}

void ArgMax1D::compute()
{
	MatrixXd::Index maxRow, maxCol;

	inMatrix().i().maxCoeff(&maxRow, &maxCol);

        output = std::max(maxRow,maxCol) ;
}

void ArgMax1D::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

REGISTER_FUNCTION(ArgMax2D);

void ArgMax2D::upreload()
{
	if( inMatrix().i().rows() <= 1 || inMatrix().i().cols() <= 1 ) 
	{
		throw std::invalid_argument("ArgMax2D Function : input have to be a matrix with dimension 2 !");
	}
}

void ArgMax2D::compute()
{
	MatrixXd::Index maxRow, maxCol;
 	auto mOut = getMapVect(output) ;

        inMatrix().i().maxCoeff(&maxRow, &maxCol);
	mOut(0) = maxRow;
	mOut(1) = maxCol;
}

void ArgMax2D::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
	
	if( output.size() != 2 ) 
	{
		throw std::invalid_argument("ArgMax2D Function : output must have only 2 neurons !");
	}
}


/*******************************************************************************************************/
/**********************************************  ArgMin  ***********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(ArgMin1D);

void ArgMin1D::upreload()
{
	if( inMatrix().i().rows() != 1 && inMatrix().i().cols() != 1 )
        {
                throw std::invalid_argument("ArgMin1D Function : input have to be a vector (1D Matrix) !");
        }
}

void ArgMin1D::compute()
{
	MatrixXd::Index minRow, minCol;

        inMatrix().i().minCoeff(&minRow, &minCol);

        output = std::max(minRow,minCol) ;
}

void ArgMin1D::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

REGISTER_FUNCTION(ArgMin2D);

void ArgMin2D::upreload()
{
	if( inMatrix().i().rows() <= 1 || inMatrix().i().cols() <= 1 )
        {
                throw std::invalid_argument("ArgMin2D Function : input have to be a 2D Matrix !");
        }
}

void ArgMin2D::compute()
{
	MatrixXd::Index minRow, minCol;
        auto mOut = getMapVect(output) ;

        inMatrix().i().minCoeff(&minRow, &minCol);
        mOut(0) = minRow;
        mOut(1) = minCol;
}

void ArgMin2D::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        
	if( output.size() != 2 )
        {
                throw std::invalid_argument("ArgMin2D Function : output must have only 2 neurons !");
        }
}

/*******************************************************************************************************/
/*******************************************  MaxCoeff  ************************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MaxCoeff);

void MaxCoeff::compute()
{
	output = inMatrix().i().maxCoeff() * inMatrix().w() ;
}

void MaxCoeff::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

/*******************************************************************************************************/
/*******************************************  MinCoeff   ***********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MinCoeff);

void MinCoeff::compute()
{
	output = inMatrix().i().minCoeff() * inMatrix().w() ;
}

void MinCoeff::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}


/*******************************************************************************************************/
/************************************************  Min   ***********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(SMin);

void SMin::compute()
{
	output = inScalar()();

	for(unsigned int i = 1; i < inScalar.size(); i++)
	{
		output = std::min(output,inScalar(i)());
	}
}

void SMin::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

REGISTER_FUNCTION(FMin);

void FMin::compute()
{
	output = inMatrix()();

	for(unsigned int i = 1; i < inMatrix.size(); i++)
        {
                output = output.array().min(inMatrix(i)().array());
        }
}

void FMin::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

/*******************************************************************************************************/
/************************************************  Max   ***********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(SMax);

void SMax::compute()
{
	output = inScalar()();

	for(unsigned int i = 1; i < inScalar.size(); i++)
	{
		output = std::max(output,inScalar(i)());
	}
}

void SMax::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

REGISTER_FUNCTION(FMax);

void FMax::compute()
{
	output = inMatrix()();

	for(unsigned int i = 1; i < inMatrix.size(); i++)
        {
                output = output.array().max(inMatrix(i)().array());
	}
}

void FMax::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

/*******************************************************************************************************/
/************************************************  Abs   ***********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MAbs);
REGISTER_FUNCTION(SAbs);

void MAbs::compute()
{
	output = inMatrix()().array().abs() ;
}

void MAbs::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SAbs::compute()
{
	output = fabs(inScalar()());
}

void SAbs::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

/*******************************************************************************************************/
/*********************************************   MODULO   **********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MModulo);
REGISTER_FUNCTION(MSModulo);
REGISTER_FUNCTION(SModulo);

void MModulo::compute()
{
  output = inMatrix()().array() - modulo()().array() * ( (inMatrix()().array()/modulo()().array()).floor() );
}

void MModulo::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(modulo,"modulo", getUuid());
}

void MSModulo::compute()
{
  output = inMatrix()().array() - modulo()() * ( (inMatrix()().array()/modulo()()).floor() );
}

void MSModulo::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(modulo,"modulo", getUuid());
}

void SModulo::compute()
{
  output  =   inScalar()() - modulo()() * floor ( inScalar()() * modulo()() );
}

void SModulo::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
        Kernel::iBind(modulo,"modulo", getUuid());
}

/*******************************************************************************************************/
/******************************************   Derivative   *********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MDerivative);
REGISTER_FUNCTION(SDerivative);
REGISTER_FUNCTION(MZ_1);
REGISTER_FUNCTION(SZ_1);

void MDerivative::compute()
{
	output =  inMatrix()() - z_1;
  	z_1 = inMatrix()(); 
}

void MDerivative::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        z_1 = MatrixXd::Constant( output.rows(), output.cols(), 0  );
}

void SDerivative::compute()
{
	output = inScalar()() - z_1 ;
	z_1 = inScalar()();
}

void SDerivative::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
	z_1 = 0;
}

void MZ_1::compute()
{
	output = z_1 ; 
	z_1 = inMatrix()();
}

void MZ_1::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        z_1 = MatrixXd::Constant( output.rows(), output.cols(), 0  );
}

void SZ_1::compute()
{
	output = z_1 ; 
	z_1 = inScalar()();
}

void SZ_1::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
	z_1 = 0;
}

