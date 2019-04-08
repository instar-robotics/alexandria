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
