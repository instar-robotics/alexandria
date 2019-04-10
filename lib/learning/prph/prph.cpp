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


#include "prph.h"
#include "kheops/util/util.h"

REGISTER_FUNCTION(PrPh);

void PrPh::compute()
{	
	static auto mout = getMapRow(output);
	
	// For each neurons from PrPh
	for( unsigned int i = 0; i < mout.size(); i++ )
	{
		SCALAR PI = 1;
		SCALAR forget;
		SCALAR sum_input_activities = 0; 
		SCALAR nb_input_active = 0;
		SCALAR sum_neg = 0;
		
		// Compute Activity
		for( unsigned int j = 0; j < inSigma.size(); j++)
		{
			auto E = inSigma(j).ivec();
			auto W = inSigma(j).wj_col(i);
			auto F = inSigma(j).fj_col(i);

			SCALAR Sigma = (E.array() *  filter(W,F).array()).maxCoeff()  ;	
			SCALAR MaxInput = E.maxCoeff();

			if( Sigma < 0 ) Sigma = 0;
			if( MaxInput > 0 ){
			       	sum_input_activities += MaxInput;
				nb_input_active++;
			}

			PI = PI * rectification(Sigma, threshold()());
		}	
	
		// Compute inhibition
		for( unsigned int j=0; j < inhibitor.size() ; j++)
		{
			auto E = inhibitor(j).ivec();
			auto W = inhibitor(j).wj_col(i);
			auto F = inhibitor(j).fj_col(i);

			MATRIX in = E * filter(W,F);

			sum_neg += in(0,0);
			sum_input_activities += in(0,0);
		}

		if( vigilence()() < 0.5 &&  isequal( reset()(), 1.0) )
		{
			if( memory(i) > 0.)
			{
				forget = dicket()();
				memory(i)-= 1.0;
			}	
			else forget = 0;
		}
		else if( isequal( reset()(), 1.0) )		
		{
			forget = 0;
			memory(i) = 0;
		}
		else forget = 1.;	

		if( sum_neg < 0 ) mout(i) = 0;
		else mout(i) = mout(i) * forget ; 

		if( PI > mout(i) )
		{
			mout(i) = PI;
			memory(i) = timeout()(); 
		}

		mout(i) = rectification( mout(i) , threshold()());

		//Learn
		if(sum_input_activities/nb_input_active > 0.999 && mout(i) < 0.001 && isequal(learn()(), 1.0 ))
		{
			for( unsigned int j = 0; j < inSigma.size(); j++ )
			{
				auto E = inSigma(j).ivec();
	                        auto W = inSigma(j).wj_col(i);
				auto F = inSigma(j).fj_col(i);
				
				W =  (filter(E,F).array() > 0.99).cast<SCALAR>();
			}	
		} 
	}
}

void  PrPh::setparameters()
{
	inSigma.setCheckSize(false);
	inhibitor.setCheckSize(false);

        Kernel::iBind(reset,"reset", getUuid());
        Kernel::iBind(learn,"learn", getUuid());
        Kernel::iBind(dicket,"dicket", getUuid());
        Kernel::iBind(timeout,"timeout", getUuid());
        Kernel::iBind(threshold,"threshold", getUuid());
        Kernel::iBind(vigilence,"vigilence", getUuid());
        Kernel::iBind(inhibitor,"inhibitor", getUuid());
        Kernel::iBind(inSigma,"inSigma", getUuid());

	memory = VectorXb::Zero( output.size() ); 
}
