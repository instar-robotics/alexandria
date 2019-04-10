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


#include "lms.h"

REGISTER_FUNCTION(LMS);

void LMS::compute()
{	
	static auto mout = getMapRow(output);

	// Compute output activity
	mout.noalias() = conditionnals(0).irow()  * conditionnals(0).w();
	for(unsigned int i=1; i < conditionnals.size(); i++)
        {
		mout.noalias() +=  conditionnals(i).irow() * conditionnals(i).w();
        }

	// Compute gradiant
        grad = learning_rate()() * (unconditionnal()() - output);
	auto vgrad = getMapRow(grad); 

	for(unsigned int i=0; i < conditionnals.size(); i++)
	{
		auto ve = conditionnals(i).icol(); 
		auto w = conditionnals(i).wm(); 
		auto f = conditionnals(i).fm(); 

		// Update weight
		// noalias is used to avoid temporary 
		w.noalias() += filter( ve * vgrad , f );
	}
}

void  LMS::setparameters()
{
	conditionnals.setMultiple(true);
	conditionnals.setCheckSize(false);

        Kernel::iBind(learning_rate,"learning_rate", getUuid());
        Kernel::iBind(unconditionnal,"unconditionnal", getUuid());
        Kernel::iBind(conditionnals,"conditionnals", getUuid());
	
	grad = MATRIX::Constant(output.rows(),output.cols(),0);
}

