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


#include <limits>
#include "boolean.h"

// Note : 
//  Input lower than 0.5 -> 0 logic
//  Input greater than 0.5 -> 1 logic

/*******************************************************************************************************/
/************************************************  AND   ***********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MAND);
REGISTER_FUNCTION(MSAND);
REGISTER_FUNCTION(SAND);

void MAND::compute()
{
	output = inMatrix(0)().unaryExpr([](SCALAR elem)
	{
		return elem < 0.5 ? 0.0 : 1.0; 
	});

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output = inMatrix(i)().binaryExpr(output, [](SCALAR e1, SCALAR e2)
		{
			SCALAR v = e1 < 0.5 ? 0.0 : 1.0;
			return ( v + e2 ) < 1 +std::numeric_limits<SCALAR>::epsilon() ? 0.0 : 1.0 ;
		});
	}		
}

void MAND::setparameters()
{
	inMatrix.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}


void MSAND::compute()
{
	SCALAR sAND = inScalar(0)() < 0.5 ? 0.0 : 1.0;

	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		SCALAR sig = inScalar(i)() < 0.5 ? 0.0 : 1.0;

		sAND = ( sig + sAND ) < 1 +std::numeric_limits<SCALAR>::epsilon() ? 0.0 : 1.0 ;
	}
	
	if( sAND >  0.5 )  
	{
		output = inMatrix(0)().unaryExpr([](SCALAR elem)
		{
			return elem < 0.5 ? 0.0 : 1.0; 
		});

		for(unsigned int i=1; i < inMatrix.size(); i++)
		{
			output = inMatrix(i)().binaryExpr(output, [](SCALAR e1, SCALAR e2)
			{
				SCALAR v = e1 < 0.5 ? 0.0 : 1.0;
				return ( v + e2 ) < 1 +std::numeric_limits<SCALAR>::epsilon() ? 0.0 : 1.0 ;
			});
		}
	}
	else
	{
		output = MATRIX::Constant( output.rows(), output.cols(), 0);
	}
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
	output = inScalar(0)() < 0.5 ? 0.0 : 1.0;

	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		SCALAR sig = inScalar(i)() < 0.5 ? 0.0 : 1.0;

		output = ( sig + output ) < 1 +std::numeric_limits<SCALAR>::epsilon() ? 0.0 : 1.0 ;
	}
}

void SAND::setparameters()
{
        inScalar.setMultiple(true);
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/*******************************************************************************************************/
/************************************************  OR  *************************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MOR);
REGISTER_FUNCTION(MSOR);
REGISTER_FUNCTION(SOR);

void MOR::compute()
{
	output = inMatrix(0)().unaryExpr([](SCALAR elem)
	{
		return elem < 0.5 ? 0.0 : 1.0; 
	});

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output = inMatrix(i)().binaryExpr(output, [](SCALAR e1, SCALAR e2)
		{
			SCALAR v = e1 < 0.5 ? 0.0 : 1.0;
			return std::max( v , e2 ) ;
		});
	}		
}

void MOR::setparameters()
{
        inMatrix.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void MSOR::compute()
{
	SCALAR sOR = inScalar(0)() < 0.5 ? 0.0 : 1.0;
	
	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		SCALAR sig = inScalar(i)() < 0.5 ? 0.0 : 1.0;

		sOR = std::max(sig, sOR);
	}

	output = inMatrix(0)().unaryExpr(FuncOR<SCALAR>(sOR) );

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output = inMatrix(i)().binaryExpr(output, [](SCALAR e1, SCALAR e2)
		{
			SCALAR v = e1 < 0.5 ? 0.0 : 1.0;
			return std::max( v , e2 ) ;
		});
	}
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
	output = inScalar(0)() < 0.5 ? 0.0 : 1.0;

	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		SCALAR sig = inScalar(i)() < 0.5 ? 0.0 : 1.0;

		output = std::max(sig, output);
	}
}

void SOR::setparameters()
{
        inScalar.setMultiple(true);
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/*******************************************************************************************************/
/************************************************  XOR  ************************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MXOR);
REGISTER_FUNCTION(MSXOR);
REGISTER_FUNCTION(SXOR);

void MXOR::compute()
{
	output = inMatrix(0)().unaryExpr([](SCALAR elem)
	{
		return elem < 0.5 ? 0.0 : 1.0; 
	});

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output += inMatrix(i)().unaryExpr([](SCALAR elem)
    		{
			return elem < 0.5 ? 0.0 : 1.0; 
    		});
	}		
	
	output = output.unaryExpr([](SCALAR elem)
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
	SCALAR sSumXOR=0;	

	output = inMatrix(0)().unaryExpr([](SCALAR elem)
	{
		return elem < 0.5 ? 0.0 : 1.0; 
	});

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		output += inMatrix(i)().unaryExpr([](SCALAR elem)
    		{
			return elem < 0.5 ? 0.0 : 1.0; 
    		});
	}

	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		if(inScalar(i)() >= 0.5) sSumXOR += 1;  
	}

	output.array()+=sSumXOR;

	output = output.unaryExpr([](SCALAR elem)
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

/*******************************************************************************************************/
/***********************************************  NOT   ************************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MNOT);
REGISTER_FUNCTION(SNOT);


void MNOT::compute()
{
 	output = inMatrix()().unaryExpr([](SCALAR elem)
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
	output = inScalar()() < 0.5 ? 1.0 : 0.0; 
}

void SNOT::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

