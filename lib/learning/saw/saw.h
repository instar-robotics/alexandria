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


#ifndef __SAW_HPP__
#define __SAW_HPP__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"
#include <iostream>

class Saw : public FMatrix
{
	protected :
		ISInput learning_rate;
		ISInput vigilence;
		IMMInput inputs;

		unsigned int nbn;

	public : 
		Saw() : nbn(0) {}
		virtual ~Saw(){}

		void updateWeight();
		virtual void setparameters();
		virtual void compute() = 0;
};

class Saw_L1 : public Saw
{
        public :
		
		Saw_L1() : Saw() {}
		virtual ~Saw_L1(){}

		virtual void compute();
};

class Saw_L2 : public Saw
{
        public :
		
		Saw_L2() : Saw() {}
		virtual ~Saw_L2(){}

		virtual void compute();
};

class Saw_Exp : public Saw
{
	private :

		ISInput sigma;

        public :

                Saw_Exp() : Saw() {}
                virtual ~Saw_Exp(){}

                virtual void compute();
		virtual void setparameters();
};

#endif // __SAW_HPP__
