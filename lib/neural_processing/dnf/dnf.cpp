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


#include "dnf.h"

REGISTER_FUNCTION(AmariDnf);

void AmariDnf::compute()
{
	bool isCircular = false;
	if( circular()() >= 0.5 ) isCircular = true;

	//output = MATRIX::NullaryExpr( output.rows(), output.cols() , Amari_functor<MATRIX>( output, kernel()(), input()(), h()() , tau()(), beta()() ,isCircular ));
	output = MATRIX::NullaryExpr( output.rows(), output.cols() , Amari_functor<MATRIX>( output, kernel()(), input()(), h()() , tau()() ,isCircular ));

}

void AmariDnf::setparameters()
{
	kernel.setCheckSize(false);

        Kernel::iBind(circular,"circular", getUuid());
        Kernel::iBind(tau,"tau", getUuid());
        //Kernel::iBind(beta,"beta", getUuid());
        Kernel::iBind(h,"h", getUuid());
        Kernel::iBind(input,"input", getUuid());
        Kernel::iBind(kernel,"kernel", getUuid());
}
