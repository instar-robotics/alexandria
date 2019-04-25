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


#include "pow.h"
#include <cmath>

/********************************************************************************************************/
/*************************************************  Exp   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MExp);
REGISTER_FUNCTION(SExp);

void MExp::compute()
{
	output = exponent()().array().exp() ;
}

void  MExp::setparameters()
{
        Kernel::iBind(exponent,"exponent", getUuid());
}

void SExp::compute()
{
	output = exp(exponent()());
}

void  SExp::setparameters()
{
        Kernel::iBind(exponent,"exponent", getUuid());
}


/********************************************************************************************************/
/*************************************************  Log   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MLog);
REGISTER_FUNCTION(SLog);
REGISTER_FUNCTION(MLog10);
REGISTER_FUNCTION(SLog10);

void MLog::compute()
{
	output = inMatrix()().array().log() ;
}

void MLog::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SLog::compute()
{
	output = log(inScalar()());
}

void SLog::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

void MLog10::compute()
{
	output = inMatrix()().array().log10();
}

void MLog10::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SLog10::compute()
{
	output = log10(inScalar()());
}

void SLog10::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/*************************************************  POW   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MPow);
REGISTER_FUNCTION(SPow);
REGISTER_FUNCTION(MSPow);
REGISTER_FUNCTION(SMPow);


void MPow::compute()
{
	output = base()().array().pow( exponent()().array() );
}

void MPow::setparameters()
{
        Kernel::iBind(base,"base", getUuid());
        Kernel::iBind(exponent,"exponent", getUuid());
}

void SPow::compute()
{
	output = pow( base()() , exponent()() );
}

void SPow::setparameters()
{
        Kernel::iBind(base,"base", getUuid());
        Kernel::iBind(exponent,"exponent", getUuid());
}

void MSPow::compute()
{
	output = base()().array().pow(  exponent()() );
}

void MSPow::setparameters()
{
        Kernel::iBind(base,"base", getUuid());
        Kernel::iBind(exponent,"exponent", getUuid());
}

void SMPow::compute()
{
	output = pow( base()(), exponent()().array() );
}

void SMPow::setparameters()
{
        Kernel::iBind(base,"base", getUuid());
        Kernel::iBind(exponent,"exponent", getUuid());
}


/*******************************************************************************************************/
/**********************************************   SQRT   ***********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MSqrt);
REGISTER_FUNCTION(SSqrt);


void MSqrt::compute()
{
	output = inMatrix()().array().sqrt();
}

void MSqrt::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SSqrt::compute()
{
	output =  sqrt(inScalar()());
}

void SSqrt::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

/*******************************************************************************************************/
/********************************************   SQUARE   ***********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MSquare);
REGISTER_FUNCTION(SSquare);


void MSquare::compute()
{
	output = inMatrix()().array().square();
}

void MSquare::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SSquare::compute()
{
	output = pow( inScalar()(), 2);
}

void SSquare::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/**********************************************   CUBE   ************************************************/
/********************************************************************************************************/


REGISTER_FUNCTION(MCube);
REGISTER_FUNCTION(SCube);


void MCube::compute()
{
	output = inMatrix()().array().cube();
}

void MCube::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SCube::compute()
{
	output = pow( inScalar()(), 3);
}

void SCube::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/*********************************************   Inverse   **********************************************/
/********************************************************************************************************/


REGISTER_FUNCTION(MInverse);
REGISTER_FUNCTION(SInverse);


void MInverse::compute()
{
	output = inMatrix()().array().inverse();
}

void MInverse::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}

void SInverse::compute()
{
	output = 1 / inScalar()();
}

void SInverse::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}

