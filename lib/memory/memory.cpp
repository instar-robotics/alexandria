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

#include "memory.h"

REGISTER_FUNCTION(MStorage);
REGISTER_FUNCTION(SStorage);
REGISTER_FUNCTION(SMemory);
REGISTER_FUNCTION(MMemory);

/*******************************************************************************************************/
/********************************************** Storage ************************************************/
/*******************************************************************************************************/

void MStorage::compute()
{
	if( record()() > 0.5 ) 
	{
		MATRIX m = inMatrix()();

		if( index()() <=0  || (unsigned int)index()() > memory.size() )
		{	
			memory.push_back(m);
		}
		else
		{
			memory[(unsigned int) index()() - 1] = m;	
		}

	}
	
	
	if( index()() <=0  || (unsigned int)index()() > memory.size() )
	{
		output = MATRIX::Constant( output.rows(), output.cols() , 0);
	}	
	else
	{
		output = memory[(unsigned int) index()() - 1 ];	
	}
}

void MStorage::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(index,"index", getUuid());
        Kernel::iBind(record,"record", getUuid());
}

void SStorage::compute()
{
        if( record()() > 0.5 )
        {
                SCALAR s = inScalar()();

                if( index()() <=0  || (unsigned int)index()() > memory.size() )
                {
                        memory.push_back(s);
                }
                else
                {
                        memory[(unsigned int) index()() - 1] = s;
                }

        }


        if( index()() <=0  || (unsigned int)index()() > memory.size() )
        {
                output = 0;
        }
        else
        {
                output = memory[(unsigned int) index()() - 1 ];
        }
}

void SStorage::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
        Kernel::iBind(index,"index", getUuid());
        Kernel::iBind(record,"record", getUuid());
}


/*******************************************************************************************************/
/*********************************************** Memory ************************************************/
/*******************************************************************************************************/

/*
void MMemory::setparameters()
{
	inMatrix.setMultiple(true);
	Kernel::iBind(inMatrix,"inMatrix", getUuid());
	Kernel::iBind(reset,"reset", getUuid());
}


void MMemory::compute()
{

}


void SMemory::setparameters()
{

}


void SMemory::compute()
{

}

*/
