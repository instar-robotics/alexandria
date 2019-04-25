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


#include <cmath>
#include "trigo.h"


/********************************************************************************************************/
/*****************************************  SIN, COS, TAN   *********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MCos);
REGISTER_FUNCTION(SCos);
REGISTER_FUNCTION(MSin);
REGISTER_FUNCTION(SSin);
REGISTER_FUNCTION(MTan);
REGISTER_FUNCTION(STan);

void MCos::compute()
{
       output = inMatrix()().array().cos();
}

void MCos::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SCos::compute()
{
	output = cos(inScalar()()); 
}

void SCos::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

void MSin::compute()
{
       output = inMatrix()().array().sin();
}

void MSin::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SSin::compute()
{
	output = sin(inScalar()()); 
}

void SSin::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

void MTan::compute()
{
       output = inMatrix()(output).array().tan();
}

void MTan::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void STan::compute()
{
	output = tan(inScalar()()); 
}

void STan::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/****************************************  ASIN, ACOS, ATAN   *******************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MAcos);
REGISTER_FUNCTION(SAcos);
REGISTER_FUNCTION(MAsin);
REGISTER_FUNCTION(SAsin);
REGISTER_FUNCTION(MAtan);
REGISTER_FUNCTION(SAtan);

void MAcos::compute()
{
       output = inMatrix()().array().acos();
}

void MAcos::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SAcos::compute()
{
	output = acos(inScalar()()); 
}

void SAcos::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

void MAsin::compute()
{
       output = inMatrix()().array().asin();
}

void MAsin::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SAsin::compute()
{
	output = asin(inScalar()()); 
}

void SAsin::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

void MAtan::compute()
{
       output = inMatrix()().array().atan();
}

void MAtan::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SAtan::compute()
{
	output = atan(inScalar()()); 
}

void SAtan::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/****************************************  SINH, COSH, TANH   *******************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MCosh);
REGISTER_FUNCTION(SCosh);
REGISTER_FUNCTION(MSinh);
REGISTER_FUNCTION(SSinh);
REGISTER_FUNCTION(MTanh);
REGISTER_FUNCTION(STanh);

void MCosh::compute()
{
       output = inMatrix()().array().cosh();
}

void MCosh::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SCosh::compute()
{
	output = cosh(inScalar()()); 
}

void SCosh::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

void MSinh::compute()
{
       output = inMatrix()().array().sinh();
}

void MSinh::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SSinh::compute()
{
	output = sinh(inScalar()()); 
}

void SSinh::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

void MTanh::compute()
{
       output = inMatrix()().array().tanh();
}

void MTanh::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void STanh::compute()
{
	output = tanh(inScalar()()); 
}

void STanh::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}
