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


#include "arithmetic.h"

/*******************************************************************************************************/
/*********************************************   SUM   *************************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MSum);
REGISTER_FUNCTION(SSum);
REGISTER_FUNCTION(MSSum);

void MSum::compute()
{
	output = inMatrix(0)();	

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output += inMatrix(i) ;	
	}
}

void  MSum::setparameters()
{
	inMatrix.setMultiple(true);
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SSum::compute()
{
	output = inScalar(0)();

	for(unsigned int i=1; i < inScalar.size(); i++)
	{
		output += inScalar(i) ; 
	}
}

void  SSum::setparameters()
{
	inScalar.setMultiple(true);
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

void MSSum::compute()
{
	double sSum=0;

	output = inMatrix(0)();	

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output += inMatrix(i);	
	}

	sSum = inScalar(0)();	

	for(unsigned int i=1; i < inScalar.size(); i++)
	{
		sSum += inScalar(i);	
	}

	output.array()+=sSum;
}

void  MSSum::setparameters()
{
	inScalar.setMultiple(true);
	inMatrix.setMultiple(true);
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(inScalar,"inScalar", getUuid());
}


/*******************************************************************************************************/
/****************************************   Substraction   *********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MMSub);
REGISTER_FUNCTION(MSSub);
REGISTER_FUNCTION(SSSub);

void MMSub::compute()
{
	output = diminuende()() - subtrahend()();
}

void MMSub::setparameters()
{
        Kernel::iBind(diminuende,"diminuende", getUuid());
        Kernel::iBind(subtrahend,"subtrahend", getUuid());
}

void MSSub::compute()
{
	output = diminuende()().array() -subtrahend()();
}

void MSSub::setparameters()
{
        Kernel::iBind(diminuende,"diminuende", getUuid());
        Kernel::iBind(subtrahend,"subtrahend", getUuid());
}

void SSSub::compute()
{
        output = diminuende()() - subtrahend()();
}

void SSSub::setparameters()
{
        Kernel::iBind(diminuende,"diminuende", getUuid());
        Kernel::iBind(subtrahend,"subtrahend", getUuid());
}

/*******************************************************************************************************/
/******************************************   Product   ************************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MMul);
REGISTER_FUNCTION(SMul);
REGISTER_FUNCTION(MSMul);

void MMul::compute()
{
        output = inMatrix(0)();

        for(unsigned int i=1; i < inMatrix.size(); i++)
        {
		output *= inMatrix(i);
        }
}

void  MMul::setparameters()
{
        inMatrix.setMultiple(true);
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SMul::compute()
{
        output =  inScalar(0)();
        for(unsigned int i=1; i < inScalar.size(); i++)
        {
                output *= inScalar(i);
        }
}

void  SMul::setparameters()
{
        inScalar.setMultiple(true);
        Kernel::iBind(inScalar,"inScalar", getUuid());
}


void MSMul::compute()
{
        double sMul=0;

        sMul = inScalar(0)();

        for(unsigned int i=1; i < inScalar.size(); i++)
        {
                sMul *= inScalar(i);
        }

        output = inMatrix(0)();

        for(unsigned int i=1; i < inMatrix.size(); i++)
        {
		output *= inMatrix(i);
        }
        output *= sMul;
}

void  MSMul::setparameters()
{
        inScalar.setMultiple(true);
        inMatrix.setMultiple(true);
        Kernel::iBind(inScalar,"inScalar", getUuid());
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

/*******************************************************************************************************/
/*****************************************   Division   ************************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MMDiv);
REGISTER_FUNCTION(MSDiv);
REGISTER_FUNCTION(SSDiv);

void MMDiv::compute()
{
	output = numerator()().array() / denumerator()().array();
}

void  MMDiv::setparameters()
{
        Kernel::iBind(numerator,"numerator", getUuid());
        Kernel::iBind(denumerator,"denumerator", getUuid());
}

void MSDiv::compute()
{
        output = numerator()().array() / denumerator()();
}

void  MSDiv::setparameters()
{
        Kernel::iBind(numerator,"numerator", getUuid());
        Kernel::iBind(denumerator,"denumerator", getUuid());
}

void SSDiv::compute()
{
        output = numerator()() / denumerator()();
}

void  SSDiv::setparameters()
{
        Kernel::iBind(numerator,"numerator", getUuid());
        Kernel::iBind(denumerator,"denumerator", getUuid());
}

/*******************************************************************************************************/
/******************************************  Normalization   *******************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(Norm);
REGISTER_FUNCTION(SquaredNorm);
REGISTER_FUNCTION(Normalize);

void Norm::compute()
{
        output = inMatrix()().norm();
}

void Norm::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}


void SquaredNorm::compute()
{
        output = inMatrix()().squaredNorm();
}

void SquaredNorm::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void Normalize::compute()
{
        output = inMatrix()().normalized();
}

void Normalize::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

/*******************************************************************************************************/
/**********************************************  Transpose   *******************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MTranspose);

void MTranspose::compute()
{
	output = inMatrix()().transpose();
}

void MTranspose::setparameters()
{
	inMatrix.setCheckSize(false);
	Kernel::iBind(inMatrix, "inMatrix", getUuid());
}

void MTranspose::prerun()
{
	if( (inMatrix().i().rows() != output.cols()) || (inMatrix().i().cols() != output.rows()) )
	{
		throw std::invalid_argument("Transpose : input and output sizes mismatch.");
	}
}

/*******************************************************************************************************/
/************************************  Matrix/Dot/Cross/Outer Product  *********************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MatrixProd);
REGISTER_FUNCTION(DotProd);
REGISTER_FUNCTION(CrossProd);
REGISTER_FUNCTION(OuterProd);

void MatrixProd::compute()
{
        output.noalias() = inMatrix1()() * inMatrix2()();
}

void MatrixProd::setparameters()
{
        inMatrix1.setCheckSize(false);
        inMatrix2.setCheckSize(false);

        Kernel::iBind(inMatrix1,"inMatrix1", getUuid());
        Kernel::iBind(inMatrix2,"inMatrix2", getUuid());
}

void MatrixProd::prerun()
{
        if( inMatrix1().i().rows() != output.rows() )
        {
                throw std::invalid_argument("MatrixProd : First input and output rows mismatch.");
        }
        if( inMatrix2().i().cols() != output.cols() )
        {
                throw std::invalid_argument("MatrixProd : Second input and output columns mismatch.");
        }
        if( inMatrix1().i().cols() != inMatrix2().i().rows() )
        {
                throw std::invalid_argument("MatrixProd : inputs sizes mismatch.");
        }
}

void DotProd::compute()
{
        auto v1 = getCMapVect(inVector1().i());
        auto v2 = getCMapVect(inVector2().i());
        output = (v1 * inVector1().w()).dot( v2 * inVector2().w());
}

void DotProd::setparameters()
{
        Kernel::iBind(inVector1,"inVector1", getUuid());
        Kernel::iBind(inVector2,"inVector2", getUuid());
}

void DotProd::prerun()
{
        if( !inVector1().isVect() )
        {
                throw std::invalid_argument("DotProd : Input 1 is not a vector.");
        }
        if( !inVector2().isVect() )
        {
                throw std::invalid_argument("DotProd : Input 2 is not a vector.");
        }
        if( inVector1().iSize()  !=  inVector2().iSize() )
        {
                throw std::invalid_argument("DotProd : Inputs size mismatch.");
        }
}

void CrossProd::compute()
{
        Map<const RowVector3d> v1(inVector1().i().data(), 3);
        Map<const RowVector3d> v2(inVector2().i().data(), 3);
        output = v1.cross(v2) * inVector1().w() * inVector2().w();
}

void CrossProd::setparameters()
{
        Kernel::iBind(inVector1,"inVector1", getUuid());
        Kernel::iBind(inVector2,"inVector2", getUuid());
}

void CrossProd::prerun()
{
        if( !inVector1().isVect() )
        {
                throw std::invalid_argument("CrossProd : Input 1 is not a vector.");
        }
        if( !inVector2().isVect() )
        {
                throw std::invalid_argument("CrossProd : Input 2 is not a vector.");
        }
        if( inVector1().iSize() != 3 )
        {
                throw std::invalid_argument("CrossProd : Input 1 is not of size 3.");
        }
        if( inVector2().iSize() != 3 )
        {
                throw std::invalid_argument("CrossProd : Input 2 is not of size 3.");
        }
}

void OuterProd::compute()
{
	output.noalias() =  inVector1()() * inVector2()();
}

void OuterProd::setparameters()
{
        Kernel::iBind(inVector1,"inVector1", getUuid());
        Kernel::iBind(inVector2,"inVector2", getUuid());
}

void OuterProd::prerun()
{
        if( !inVector1().isColVect() )
        {
                throw std::invalid_argument("OuterProd : Input 1 is not a col vector.");
        }
        if( !inVector2().isRowVect() )
        {
                throw std::invalid_argument("OuterProd : Input 2 is not a row vector.");
        }

	if( output.size() != inVector1().iSize() * inVector2().iSize() )
	{
                throw std::invalid_argument("OuterProd : Output dimension should be egal to inVector1 dimension * inVector2 dimension.");
	}
}
