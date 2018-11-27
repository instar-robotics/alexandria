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

#include "basicneuronal.h"

/*******************************************************************************************************/
/************************************************ KeepMax  *********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(KeepMax);

void KeepMax::compute()
{
	output = MatrixXd::Constant(output.rows(),output.cols(), 0);
	for(unsigned int i = 0; i < nMax()(); i++)
	{
		MatrixXd::Index maxRow, maxCol;
  		double max = inMatrix().i().maxCoeff(&maxRow, &maxCol);
	
		output(maxRow,maxCol) = max * inMatrix().w();
	}
}

void KeepMax::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(nMax,"nMax", getUuid());
}

/*******************************************************************************************************/
/************************************************ KeepMin  *********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(KeepMin);

void KeepMin::compute()
{
	output = MatrixXd::Constant(output.rows(),output.cols(), 0);
	for(unsigned int i = 0; i < nMin()(); i++)
	{
		MatrixXd::Index minRow, minCol;
  		double min = inMatrix().i().minCoeff(&minRow, &minCol);
	
		output(minRow,minCol) = min * inMatrix().w();
	}
}

void KeepMin::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(nMin,"nMax", getUuid());
}

/*******************************************************************************************************/
/*********************************************  ActToPop   *********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(ActToPop);

void ActToPop::compute()
{
	double value = activity()();

	// Threshold betwen 0 and 1
	if( value < 0 ) value = 0;
	if( value >= 1 ) value = 1 - std::numeric_limits<double>::epsilon();

	auto vect = getMapVect(output);
	MatrixXd::Index index = value * double(vect.size()) ;
	
	vect(lastIndex) = 0;
	vect(index) = 1;

	lastIndex = index ;
}

void ActToPop::setparameters()
{
        Kernel::instance().bind(activity,"activity", getUuid());

	// Check Output dimension : 
	if( output.rows() != 1 && output.cols() != 1 ) 
	{
		throw std::invalid_argument("ActToPop : Output must be a vector [ROW or COL]");	
	}
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
			double value =  vact(i) * activities().w();

			// Threshold betwen 0 and 1
			if( value < 0 ) value = 0;
			if( value >= 1 ) value = 1 - std::numeric_limits<double>::epsilon();

			MatrixXd::Index index = value * double(output.cols());

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
			double value =  vact(i) * activities().w();

			// Threshold betwen 0 and 1
			if( value < 0 ) value = 0;
			if( value >= 1 ) value = 1 - std::numeric_limits<double>::epsilon();

			MatrixXd::Index index = value * double(output.rows());
			output( lastIndex(i), i ) = 0;
			output( index , i ) = 1;

			lastIndex(i) = index;
		}
	}
	else if( proj == SINGLEV)
	{
		double value = activities()()(0,0);

		// Threshold betwen 0 and 1
		if( value < 0 ) value = 0;
		if( value >= 1 ) value = 1 - std::numeric_limits<double>::epsilon();

		auto vect = getMapVect(output);
		MatrixXd::Index index = value * double(vect.size()) ;

		vect(lastIndex(0)) = 0;
		vect(index) = 1;

		lastIndex(0) = index ;
	}
}

void VActToPop::setparameters()
{
	activities.setCheckSize(false); 

        Kernel::instance().bind(activities,"activities", getUuid());
}

void VActToPop::uprerun()
{
	if( activities().i().rows() != 1 && activities().i().cols() != 1 ) 
	{
		throw std::invalid_argument("VAcToPop : Input \"activities\" must be a vector [ROW or COL]");	
	}
	
	if( activities().i().rows() == 1 && activities().i().cols() == 1)
	{
		if( output.rows() != 1 && output.cols() != 1 ) 
		{
			throw std::invalid_argument("VActToPop : \"activities\" dimension is egal to one, so output must be a vector [ROW or COL]");	
		}

		//Single Neuron : Projection depend on output [ROW or COL]
		proj = SINGLEV;
		lastIndex = VectorXd::Constant( 1 , 0);
	
	}
	else if( activities().i().rows() == 1 )
	{
		if(output.cols() != activities().i().cols() )
		{
			throw std::invalid_argument("VAcToPop : \"activities\" input is a ROW Vector so, activities cols and output cols must be egal !");	
		}

		// COL Projection
		proj = COLPROJ;
		lastIndex = VectorXd::Constant( activities().i().cols() , 0);
	}
	else if( activities().i().cols() == 1)  
	{
		if( output.rows() != activities().i().rows() )
		{
			throw std::invalid_argument("VAcToPop : \"activities\" input is a COL Vector so, activities rows and output rows must be egal !");	
		}

		// ROW Projection
		proj = ROWPROJ;
		lastIndex = VectorXd::Constant( activities().i().rows() , 0);
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
	
	MatrixXd::Index i;
	vect.maxCoeff(&i); 

//	output = population().w() * ( double(i)/double(vect.size())  + 1.0/(2.0*vect.size()));
	output = population().w() * double(i) / (double(vect.size())-1.0);
}

void PopToAct::setparameters()
{
	population.setCheckSize(false); 
        Kernel::instance().bind(population,"population", getUuid());
}

void PopToAct::uprerun()
{
	// Check Input dimension : 
	if( population().i().rows() != 1 && population().i().cols() != 1 ) 
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
			MatrixXd::Index ind;
			population().i().col(i).maxCoeff(&ind);		

                        vpop(i) = population().w() * double(ind)/(double(population().i().rows()) - 1.0);
                }
	}
	else if( proj == ROWPROJ )
	{
                auto vpop = getMapVect(output);

                for( unsigned int i = 0; i < vpop.size(); i++)
                {
			MatrixXd::Index ind;
			population().i().row(i).maxCoeff(&ind);		

                        vpop(i) = population().w() * double(ind)/(double(population().i().cols()) - 1.0);
                }
	}
	else if( proj == SINGLEV )
	{
		auto vect = getCMapVect(population().i());
		
		MatrixXd::Index i;
		vect.maxCoeff(&i); 

		output(0,0) = population().w() * double(i) / (double(vect.size())-1.0) ;
	}
}

void PopToVAct::setparameters()
{
	population.setCheckSize(false); 
        Kernel::instance().bind(population,"population", getUuid());
	
	if( output.rows() != 1 && output.cols() != 1 ) 
	{
		throw std::invalid_argument("PopToVAct : Ouput must be a vector [ROW or COL]");	
	}
}

void PopToVAct::uprerun()
{
	if(  output.rows() == 1 &&  output.cols() == 1)
	{
		if( population().i().rows() != 1 && population().i().cols() != 1 )
        	{
                	throw std::invalid_argument("PopToAct : Output is a One-One Matrix, so input \"population\" must be a vector [ROW or COL]");
        	}
		proj = SINGLEV;
	}	
	else if( output.rows() == 1 ) 
	{
		if( output.cols() != population().i().cols() )
		{
			throw std::invalid_argument("PopToVAct : Ouput is a ROW vector so, Population cols and output cols must be egal !");	
		}
		proj = COLPROJ;
	}
	else if( output.cols() == 1 )
	{
		if( output.rows() != population().i().rows() )
		{
			throw std::invalid_argument("PopToVAct : Ouput is a COL vector so, Population rows and output rows must be egal !");	
		}
		proj = ROWPROJ;
	}
}
