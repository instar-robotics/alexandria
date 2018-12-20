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

#include "saw.h"

void Saw::updateWeight()
{
        int row_max,col_max;
        if( output.maxCoeff(&row_max,&col_max) < vigilence()() )
        {
                // Recruitement : 
                for( unsigned int j = 0; j < inputs.size(); j++ )
                {
                        auto F = inputs(j).fj_col(nbn);
                        auto W = inputs(j).wj_col(nbn);
                        auto E = inputs(j).icol();

                        W = filter(E,F);
                }
                nbn++;
        }
        else
        {
                // Update Weight :
                for( unsigned int j = 0; j < inputs.size(); j++ )
                {
                        auto F = inputs(j).fj_col(row_max,col_max);
                        auto W = inputs(j).wj_col(row_max,col_max);
                        auto E = inputs(j).icol();

                        W += filter((E - W) * learning_rate()(), F);
                }
        }
}

void Saw::setparameters()
{
	inputs.setMultiple(true);
	inputs.setCheckSize(false);

        Kernel::iBind(vigilence,"vigilence", getUuid());
        Kernel::iBind(inputs,"inputs", getUuid());
        Kernel::iBind(learning_rate,"learning_rate", getUuid());
}

/*******************************************************************************************************/
/*                                          SAW with L1 distance                                       */
/*******************************************************************************************************/

REGISTER_FUNCTION(Saw_L1);

void Saw_L1::compute()
{	
	static auto mout = getMapRow(output);

	// Compute Activities
	for( unsigned int i = 0; i < std::min((unsigned int)mout.size(),nbn) ; i++)
	{
		double N = 0;
		auto E = inputs(0).ivec();
		auto W = inputs(0).wj_col(i);
		auto F = inputs(0).fj_col(i);
		
		mout(i) = (filter( E - W, F) ).cwiseAbs().sum() ; 
		N += inputs(0).f().col(i).sum();
		
		for(unsigned int j = 1; j < inputs.size() ; j++ )
		{
			auto E = inputs(j).ivec();
			auto W = inputs(j).wj_col(i);
			auto F = inputs(j).fj_col(i);

			mout(i) += (filter(E - W, F)).cwiseAbs().sum() ; 
			N += inputs(0).f().col(i).sum();
		}
		if( N > 0 ) mout(i) = 1.0 - mout(i) / N;
	}
	updateWeight();
}

/*******************************************************************************************************/
/*                                          SAW with L2 distance                                       */
/*******************************************************************************************************/

REGISTER_FUNCTION(Saw_L2);

void Saw_L2::compute()
{
        static auto mout = getMapRow(output);

        // Compute Activities
        for( unsigned int i = 0; i < std::min((unsigned int)mout.size(),nbn) ; i++)
        {
        	double N = 0;
                auto E = inputs(0).ivec();
                auto W = inputs(0).wj_col(i);
                auto F = inputs(0).fj_col(i);

                mout(i) = (filter( E - W, F) ).array().square().sum() ;
                N += inputs(0).f().col(i).sum();

                for(unsigned int j = 1; j < inputs.size() ; j++ )
                {
                        auto E = inputs(j).ivec();
                        auto W = inputs(j).wj_col(i);
                        auto F = inputs(j).fj_col(i);

                        mout(i) += (filter(E - W, F)).array().square().sum() ;
                        N += inputs(0).f().col(i).sum();
                }
                if( N > 0 ) mout(i) = 1.0 - sqrt(mout(i)) / N;
        }
        updateWeight();
}

/*******************************************************************************************************/
/*                                   SAW with Exponential distance                                     */
/*******************************************************************************************************/

REGISTER_FUNCTION(Saw_Exp);

void Saw_Exp::compute()
{
        static auto mout = getMapRow(output);

        // Compute Activities
        for( unsigned int i = 0; i < std::min((unsigned int)mout.size(),nbn) ; i++)
        {
                auto E = inputs(0).ivec();
                auto W = inputs(0).wj_col(i);
                auto F = inputs(0).fj_col(i);

                mout(i) = (filter( E - W, F) ).array().square().sum() ;

                for(unsigned int j = 1; j < inputs.size() ; j++ )
                {
                        auto E = inputs(j).ivec();
                        auto W = inputs(j).wj_col(i);
                        auto F = inputs(j).fj_col(i);

                        mout(i) += (filter(E - W, F)).array().square().sum() ;
                }
                mout(i) = exp(  mout(i) * -1 / (2 * sigma()()));
        }
        updateWeight();
}

void Saw_Exp::setparameters()
{
	Saw::setparameters();
        Kernel::iBind(sigma,"sigma", getUuid());
}
