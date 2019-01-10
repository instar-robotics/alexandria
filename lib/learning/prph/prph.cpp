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

#include "prph.h"
#include "kheops/util/util.h"

REGISTER_FUNCTION(PrPh);

void PrPh::compute()
{	
	static auto mout = getMapRow(output);
	
	// For each neurons from PrPh
	for( unsigned int i = 0; i < mout.size(); i++ )
	{
		double PI = 1;
		double forget;
		double sum_input_activities = 0; 
		double nb_input_active = 0;
		double sum_neg = 0;
		
		// Compute Activity
		for( unsigned int j = 0; j < inSigma.size(); j++)
		{
			auto E = inSigma(j).ivec();
			auto W = inSigma(j).wj_col(i);
			auto F = inSigma(j).fj_col(i);

			double Sigma = (E.array() *  filter(W,F).array()).maxCoeff()  ;	
			double MaxInput = E.maxCoeff();

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

			MatrixXd in = E * filter(W,F);

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
				
				W =  (filter(E,F).array() > 0.99).cast<double>();
			}	
		} 
	}
}

void  PrPh::setparameters()
{
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
