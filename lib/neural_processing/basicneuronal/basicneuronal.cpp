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


#include "basicneuronal.h"

/*******************************************************************************************************/
/************************************************ KeepMax  *********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(KeepMax);

void KeepMax::compute()
{
	output = MATRIX::Constant(output.rows(),output.cols(), 0);
	for(unsigned int i = 0; i < nMax()(); i++)
	{
		MATRIX::Index maxRow, maxCol;
  		SCALAR max = inMatrix().i().maxCoeff(&maxRow, &maxCol);
	
		output(maxRow,maxCol) = max * inMatrix().w();
	}
}

void KeepMax::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(nMax,"nMax", getUuid());
}

/*******************************************************************************************************/
/************************************************ KeepMin  *********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(KeepMin);

void KeepMin::compute()
{
	output = MATRIX::Constant(output.rows(),output.cols(), 0);
	for(unsigned int i = 0; i < nMin()(); i++)
	{
		MATRIX::Index minRow, minCol;
  		SCALAR min = inMatrix().i().minCoeff(&minRow, &minCol);
	
		output(minRow,minCol) = min * inMatrix().w();
	}
}

void KeepMin::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(nMin,"nMax", getUuid());
}

/*******************************************************************************************************/
/*********************************************  ActToPop   *********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(ActToPop);

void ActToPop::compute()
{
	SCALAR value = activity()();

	// Threshold betwen 0 and 1
	if( value < 0 ) value = 0;
	if( value >= 1 ) value = 1 - std::numeric_limits<SCALAR>::epsilon();

	auto vect = getMapVect(output);
	MATRIX::Index index = value * SCALAR(vect.size()) ;
	
	vect(lastIndex) = 0;
	vect(index) = 1;

	lastIndex = index ;
}

void ActToPop::setparameters()
{
        Kernel::iBind(activity,"activity", getUuid());
}

/*******************************************************************************************************/
/********************************************  VActToPop   *********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(VActToPop);

void VActToPop::compute()
{
	if( proj == ROWPROJ )
	{
		auto vact = getCMapVect(activities().i());

		for( unsigned int i = 0; i < vact.size(); i++)
		{
			SCALAR value =  vact(i) * activities().w();

			// Threshold betwen 0 and 1
			if( value < 0 ) value = 0;
			if( value >= 1 ) value = 1 - std::numeric_limits<SCALAR>::epsilon();

			MATRIX::Index index = value * SCALAR(output.cols());

			output( i, lastIndex(i) ) = 0;
			output( i , index ) = 1;

			lastIndex(i) = index;
		}
	}
	else if( proj == COLPROJ )
	{
		auto vact = getCMapVect(activities().i());

		for( unsigned int i = 0; i < vact.size(); i++)
		{
			SCALAR value =  vact(i) * activities().w();

			// Threshold betwen 0 and 1
			if( value < 0 ) value = 0;
			if( value >= 1 ) value = 1 - std::numeric_limits<SCALAR>::epsilon();

			MATRIX::Index index = value * SCALAR(output.rows());
			output( lastIndex(i), i ) = 0;
			output( index , i ) = 1;

			lastIndex(i) = index;
		}
	}
	else if( proj == SINGLEV)
	{
		SCALAR value = activities()()(0,0);

		// Threshold betwen 0 and 1
		if( value < 0 ) value = 0;
		if( value >= 1 ) value = 1 - std::numeric_limits<SCALAR>::epsilon();

		auto vect = getMapVect(output);
		MATRIX::Index index = value * SCALAR(vect.size()) ;

		vect(lastIndex(0)) = 0;
		vect(index) = 1;

		lastIndex(0) = index ;
	}
}

void VActToPop::setparameters()
{
	activities.setCheckSize(false); 

        Kernel::iBind(activities,"activities", getUuid());
}

void VActToPop::prerun()
{
	if( ! activities().isVect() ) 
	{
		throw std::invalid_argument("VAcToPop : Input \"activities\" must be a vector [ROW or COL]");	
	}
	
	if( activities().isPoint())
	{
		if(!isVect()) 
		{
			throw std::invalid_argument("VActToPop : \"activities\" dimension is egal to one, so output must be a vector [ROW or COL]");	
		}

		//Single Neuron : Projection depend on output [ROW or COL]
		proj = SINGLEV;
		lastIndex = VectorXs::Constant( 1 , 0);
	
	}
	else if( activities().isRowVect() )
	{
		if(output.cols() != activities().iCols() )
		{
			throw std::invalid_argument("VAcToPop : \"activities\" input is a ROW Vector so, activities cols and output cols must be egal !");	
		}

		// COL Projection
		proj = COLPROJ;
		lastIndex = VectorXs::Constant( activities().iCols() , 0);
	}
	else if( activities().isColVect())  
	{
		if( output.rows() != activities().i().rows() )
		{
			throw std::invalid_argument("VAcToPop : \"activities\" input is a COL Vector so, activities rows and output rows must be egal !");	
		}

		// ROW Projection
		proj = ROWPROJ;
		lastIndex = VectorXs::Constant( activities().iRows() , 0);
	}
}

/*******************************************************************************************************/
/*********************************************  PopToAct   *********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(PopToAct);

// Note : compute VALUE :
// value = i / size + 1 / (2*size) 
// 	or
// value = i / ( size - 1) 

void PopToAct::compute()
{
	auto vect = getCMapVect(population().i());
	
	MATRIX::Index i;
	vect.maxCoeff(&i); 

//	output = population().w() * ( SCALAR(i)/SCALAR(vect.size())  + 1.0/(2.0*vect.size()));
	output = population().w() * SCALAR(i) / (SCALAR(vect.size())-1.0);
}

void PopToAct::setparameters()
{
	population.setCheckSize(false); 
        Kernel::iBind(population,"population", getUuid());
}

void PopToAct::prerun()
{
	// Check Input dimension : 
	if( !population().isVect()) 
	{
		throw std::invalid_argument("PopToAct : Input \"population\" must be a vector [ROW or COL]");	
	}
}

/*******************************************************************************************************/
/**********************************************  PopToVAct   *******************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(PopToVAct);

void PopToVAct::compute()
{
	if( proj == COLPROJ )
	{
                auto vpop = getMapVect(output);

                for( unsigned int i = 0; i < vpop.size(); i++)
                {
			MATRIX::Index ind;
			population().i().col(i).maxCoeff(&ind);		

                        vpop(i) = population().w() * SCALAR(ind)/(SCALAR(population().i().rows()) - 1.0);
                }
	}
	else if( proj == ROWPROJ )
	{
                auto vpop = getMapVect(output);

                for( unsigned int i = 0; i < vpop.size(); i++)
                {
			MATRIX::Index ind;
			population().i().row(i).maxCoeff(&ind);		

                        vpop(i) = population().w() * SCALAR(ind)/(SCALAR(population().i().cols()) - 1.0);
                }
	}
	else if( proj == SINGLEV )
	{
		auto vect = getCMapVect(population().i());
		
		MATRIX::Index i;
		vect.maxCoeff(&i); 

		output(0,0) = population().w() * SCALAR(i) / (SCALAR(vect.size())-1.0) ;
	}
}

void PopToVAct::setparameters()
{
	population.setCheckSize(false); 
        Kernel::iBind(population,"population", getUuid());
}

void PopToVAct::prerun()
{
	if(isPoint())
	{
		if( !population().isVect() )
        	{
                	throw std::invalid_argument("PopToAct : Output is a One-One Matrix, so input \"population\" must be a vector [ROW or COL]");
        	}
		proj = SINGLEV;
	}	
	else if( isRowVect()) 
	{
		if( output.cols() != population().iCols() )
		{
			throw std::invalid_argument("PopToVAct : Ouput is a ROW vector so, Population cols and output cols must be egal !");	
		}
		proj = COLPROJ;
	}
	else if( isColVect() )
	{
		if( output.rows() != population().iRows() )
		{
			throw std::invalid_argument("PopToVAct : Ouput is a COL vector so, Population rows and output rows must be egal !");	
		}
		proj = ROWPROJ;
	}
}

/*******************************************************************************************************/
/*******************************************  Convolution   ********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(Convolution);

void Convolution::compute()
{
	bool isCircular = false;
        if( circular()() >= 0.5 ) isCircular = true;

        output = MATRIX::NullaryExpr( output.rows(), output.cols() , Conv_functor<MATRIX>( inMatrix()(), mask()(), isCircular ));
}

void Convolution::setparameters()
{
	mask.setCheckSize(false); 
        Kernel::iBind(circular,"circular", getUuid());
        Kernel::iBind(mask,"mask", getUuid());
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

/*******************************************************************************************************/
/**********************************************  Shift   ***********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(Shift);
REGISTER_FUNCTION(ShiftInv);

void Shift::compute()
{
	MATRIX::Index mRow, mCol;
	mask().i().maxCoeff(&mRow, &mCol);
	output = MATRIX::NullaryExpr( output.rows(), output.cols(), Shift_functor<MATRIX>( inMatrix()(),  mCol, mRow , output.cols(), output.rows(), 1 ));
}

void Shift::setparameters()
{
	mask.setCheckSize(false); 
        Kernel::iBind(mask,"mask", getUuid());
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void ShiftInv::compute()
{
        MATRIX::Index mRow, mCol;
        mask().i().maxCoeff(&mRow, &mCol);
        output = MATRIX::NullaryExpr( output.rows(), output.cols(), Shift_functor<MATRIX>( inMatrix()(),  mCol, mRow , output.cols(), output.rows(), -1 ));
}

void ShiftInv::setparameters()
{
        mask.setCheckSize(false);
        Kernel::iBind(mask,"mask", getUuid());
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

/*******************************************************************************************************/
/********************************************* Projection **********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(Projection);

void Projection::compute()
{
	auto mout = getMapRow(output);

	mout.noalias() =  inMatrix(0).irow() * filter( inMatrix(0).w() , inMatrix(0).f() );

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		mout.noalias() +=  inMatrix(i).irow() * filter( inMatrix(i).w() , inMatrix(i).f() );

	}
}

void Projection::setparameters()
{
	inMatrix.setMultiple(true);
        inMatrix.setCheckSize(false);
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

