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

