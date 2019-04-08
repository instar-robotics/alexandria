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


#ifndef __LMS_HPP__
#define __LMS_HPP__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

class LMS : public FMatrix
{
	private :

		ISInput learning_rate;
		ISMInput unconditionnal;
		IMMInput conditionnals;

		MatrixXd grad;
	
        public :
		
		virtual ~LMS(){}

		virtual void compute();
		virtual void setparameters();

};

#endif // __LMS_HPP__
