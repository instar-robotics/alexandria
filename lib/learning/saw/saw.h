/*
Copyright INSTAR Robotics

Author: Pierre Delarboulas

This software is governed by the CeCILL v2.1 license under French law and abiding by the rules of distribution of free software. 
You can use, modify and/ or redistribute the software under the terms of the CeCILL v2.1 license as circulated by CEA, CNRS and INRIA at the following URL "http://www.cecill.info".
As a counterpart to the access to the source code and  rights to copy, modify and redistribute granted by the license, 
users are provided only with a limited warranty and the software's author, the holder of the economic rights,  and the successive licensors have only limited liability.  
In this respect, the user's attention is drawn to the risks associated with loading, using, modifying and/or developing or reproducing the software by the user in light of its specific status of free software, 
that may mean  that it is complicated to manipulate, and that also therefore means that it is reserved for developers and experienced professionals having in-depth computer knowledge. 
Users are therefore encouraged to load and test the software's suitability as regards their requirements in conditions enabling the security of their systems and/or data to be ensured 
and, more generally, to use and operate it in the same conditions as regards security. 
The fact that you are presently reading this means that you have had knowledge of the CeCILL v2.1 license and that you accept its terms.
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
