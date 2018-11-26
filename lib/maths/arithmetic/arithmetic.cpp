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
REGISTER_FUNCTION(MatrixProd);

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

void MatrixProd::compute()
{
        output = inMatrix1(0)() * inMatrix2(0)();
}

void MatrixProd::setparameters()
{
	inMatrix1.setCheckSize(false);
	inMatrix2.setCheckSize(false);

	Kernel::iBind(inMatrix1,"inMatrix1", getUuid());
	Kernel::iBind(inMatrix2,"inMatrix2", getUuid());
}

void MatrixProd::uprerun()
{
	if( inMatrix1().i().rows() != output.rows() )
	{
		throw std::invalid_argument("MMMul : First input and output rows mismatch.");
	}
	if( inMatrix2().i().cols() != output.cols() )
	{
		throw std::invalid_argument("MMMul : Second input and output columns mismatch.");
	}
	if( inMatrix1().i().cols() != inMatrix2().i().rows() )
	{
		throw std::invalid_argument("MMMul : inputs sizes mismatch.");
	}
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

void MTranspose::uprerun()
{
	if( (inMatrix().i().rows() != output.cols()) || (inMatrix().i().cols() != output.rows()) )
	{
		throw std::invalid_argument("MTranspose : input and output sizes mismatch.");
	}
}
