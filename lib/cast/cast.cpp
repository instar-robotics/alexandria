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


#include "cast.h"

REGISTER_FUNCTION(Cast_MToS);
REGISTER_FUNCTION(Cast_SToM);

/*******************************************************************************************************/
/*****************                             Cast_SToM                             *******************/
/*******************************************************************************************************/

void Cast_SToM::compute()
{
	output(0,0) = inScalar()();
}


void Cast_SToM::setparameters()
{
	Kernel::iBind(inScalar,"inScalar", getUuid());
}

/*******************************************************************************************************/
/*****************                             Cast_MToS                             *******************/
/*******************************************************************************************************/

void Cast_MToS::compute()
{
	output = inMatrix()()(0,0);
}


void Cast_MToS::setparameters()
{
	Kernel::iBind(inMatrix,"inMatrix", getUuid());
}


void Cast_MToS::prerun()
{
	     if( !inMatrix().isPoint() ) throw std::invalid_argument("Cast_MToS : Matrix Input dimension should be 1 !");
}

/*******************************************************************************************************/
/*****************                          Concat_STo3DV                            *******************/
/*******************************************************************************************************/

void Concat_STo3DV::compute()
{
	auto mout = getMapVect(output);
        mout(0) = x()();
        mout(1) = y()();
        mout(2) = z()();
}


void Concat_STo3DV::setparameters()
{
	if( output.size() != 3 ) throw std::invalid_argument("Concat_STo3DV : Output dimension should be 3 !");

        Kernel::iBind(x,"x", getUuid());
        Kernel::iBind(y,"y", getUuid());
        Kernel::iBind(z,"z", getUuid());
}

/*******************************************************************************************************/
/*****************                          Concat_STo2DV                            *******************/
/*******************************************************************************************************/

void Concat_STo2DV::compute()
{
        auto mout = getMapVect(output);
        mout(0) = x()();
        mout(1) = y()();
}


void Concat_STo2DV::setparameters()
{
        if( output.size() != 2 ) throw std::invalid_argument("Concat_STo2DV : Output dimension should be 2 !");

        Kernel::iBind(x,"x", getUuid());
        Kernel::iBind(y,"y", getUuid());
}
