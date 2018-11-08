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

#include "boolean.h"


// Note : 
//  Input lower than 0.5 -> 0 logic
//  Input greater than 0.5 -> 1 logic


/********************************************************************************************************/
/*************************************************  AND   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MAND);
REGISTER_FUNCTION(MSAND);
REGISTER_FUNCTION(SAND);


void MAND::compute()
{
	output = inMatrix(0)().unaryExpr([](double elem)
	{
		return elem < 0.5 ? 0.0 : 1.0; 
	});

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output += inMatrix(i)().unaryExpr([](double elem)
    		{
			return elem < 0.5 ? 0.0 : 1.0; 
    		});
	}		
	
	output = output.unaryExpr(NormFunc<double>(inMatrix.size()));
}

void MAND::setparameters()
{
	inMatrix.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}


void MSAND::compute()
{
	double sSumAND=0;	

	output = inMatrix(0)().unaryExpr([](double elem)
	{
		return elem < 0.5 ? 0.0 : 1.0; 
	});

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output += inMatrix(i)().unaryExpr([](double elem)
    		{
			return elem < 0.5 ? 0.0 : 1.0; 
    		});
	}

	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		if(inScalar(i)() >= 0.5) sSumAND += 1;  
	}

	output.array()+=sSumAND;

	output = output.unaryExpr(NormFunc<double>(inMatrix.size()+inScalar.size()));
}

void MSAND::setparameters()
{
        inMatrix.setMultiple(true);
        inScalar.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}


void SAND::compute()
{
	output = 0;

	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		if(inScalar(i)() >= 0.5) output += 1;  
	}
		
	if(output/inScalar.size() < 1)	output = 0;
	else output = 1;
	
}

void SAND::setparameters()
{
        inScalar.setMultiple(true);
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}


/********************************************************************************************************/
/*************************************************  OR   ************************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MOR);
REGISTER_FUNCTION(MSOR);
REGISTER_FUNCTION(SOR);

void MOR::compute()
{
	output = inMatrix(0)().unaryExpr([](double elem)
	{
		return elem < 0.5 ? 0.0 : 1.0; 
	});

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output += inMatrix(i)().unaryExpr([](double elem)
    		{
			return elem < 0.5 ? 0.0 : 1.0; 
    		});
	}		
	
	output = output.unaryExpr([](double elem)
	{
		return elem < 0.5 ? 0.0 : 1.0; 
	});
}

void MOR::setparameters()
{
        inMatrix.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void MSOR::compute()
{
	double sSumOR=0;	

	output = inMatrix(0)().unaryExpr([](double elem)
	{
		return elem < 0.5 ? 0.0 : 1.0; 
	});

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output += inMatrix(i)().unaryExpr([](double elem)
    		{
			return elem < 0.5 ? 0.0 : 1.0; 
    		});
	}

	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		if(inScalar(i)() >= 0.5) sSumOR += 1;  
	}

	output.array()+=sSumOR;

	output = output.unaryExpr([](double elem)
	{
		return elem < 0.5 ? 0.0 : 1.0; 
	});

}

void MSOR::setparameters()
{
        inMatrix.setMultiple(true);
        inScalar.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}


void SOR::compute()
{
	output = 0;

	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		if(inScalar(i)() >= 0.5) output = 1;  
	}
	
}

void SOR::setparameters()
{
        inScalar.setMultiple(true);
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/************************************************  XOR   ************************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MXOR);
REGISTER_FUNCTION(MSXOR);
REGISTER_FUNCTION(SXOR);

void MXOR::compute()
{
	output = inMatrix(0)().unaryExpr([](double elem)
	{
		return elem < 0.5 ? 0.0 : 1.0; 
	});

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output += inMatrix(i)().unaryExpr([](double elem)
    		{
			return elem < 0.5 ? 0.0 : 1.0; 
    		});
	}		
	
	output = output.unaryExpr([](double elem)
	{
		return elem < 0.5 || elem > 1 ? 0.0 : 1.0; 
	});
}

void MXOR::setparameters()
{
        inMatrix.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void MSXOR::compute()
{
	double sSumXOR=0;	

	output = inMatrix(0)().unaryExpr([](double elem)
	{
		return elem < 0.5 ? 0.0 : 1.0; 
	});

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output += inMatrix(i)().unaryExpr([](double elem)
    		{
			return elem < 0.5 ? 0.0 : 1.0; 
    		});
	}

	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		if(inScalar(i)() >= 0.5) sSumXOR += 1;  
	}

	output.array()+=sSumXOR;

	output = output.unaryExpr([](double elem)
	{
		return elem < 0.5 || elem > 1 ? 0.0 : 1.0; 
	});
}

void MSXOR::setparameters()
{
        inMatrix.setMultiple(true);
        inScalar.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}


void SXOR::compute()
{
	output = 0;

	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		if(inScalar(i)() >= 0.5) output += 1;  
	}
		
	if(output > 0 && output/inScalar.size() < 1)	output = 1;
	else output = 0;

}

void SXOR::setparameters()
{
        inScalar.setMultiple(true);
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/************************************************  NOT   ************************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MNOT);
REGISTER_FUNCTION(SNOT);


void MNOT::compute()
{
 	output = inMatrix()().unaryExpr([](double elem)
    	{
		return elem < 0.5 ? 1.0 : 0.0; 
    	});
}

void MNOT::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}


void SNOT::compute()
{
	if(inScalar()() >= 0.5)	output = 0;  
	else output = 1;
}

void SNOT::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

