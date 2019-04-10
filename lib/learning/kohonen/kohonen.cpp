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


#include "kohonen.h"

REGISTER_FUNCTION(Kohonen);

void Kohonen::compute()
{	
	static auto mout = getMapRow(output);

	for( unsigned int i = 0; i < mout.size(); i++)
	{
                auto E = input(0).ivec();
                auto W = input(0).wj_col(i);
                auto F = input(0).fj_col(i);

                mout(i) = (filter( E - W, F) ).array().square().sum() ;

                for(unsigned int j = 1; j < input.size() ; j++ )
                {
                        auto E = input(j).ivec();
                        auto W = input(j).wj_col(i);
                        auto F = input(j).fj_col(i);

                        mout(i) += (filter(E - W, F)).array().square().sum() ;
                }
           	mout(i) = 1.0 / (1.0 + sqrt(mout(i)));
	}

	// Update weight
	MATRIX::Index maxRow, maxCol;
 	SCALAR max = output.maxCoeff(&maxRow, &maxCol);

	for( MATRIX::Index i = 0; i < output.rows(); i++)
	{
		for( MATRIX::Index j = 0; j < output.cols(); j++)
		{
			SCALAR h = exp( -( sqrt( (i - maxRow) * (i-maxRow) + (j-maxCol) * (j-maxCol))  ) / (2 * neighborhood()() * neighborhood()())   );

        	        auto E = input(0).ivec();
                	auto W = input(0).wj_col(i,j);
                	auto F = input(0).fj_col(i,j);

			W += learning_rate()() * filter(E - W, F) * h;

			for( unsigned int n = 1; n < input.size(); n++ )
			{
				auto E = input(n).ivec();
				auto W = input(n).wj_col(i,j);
				auto F = input(n).fj_col(i,j);

				W += learning_rate()() * filter(E - W, F) * h;
			}
		}
	}
}

void  Kohonen::setparameters()
{
	input.setMultiple(true);
	input.setCheckSize(false);

        Kernel::iBind(learning_rate,"learning_rate", getUuid());
        Kernel::iBind(neighborhood,"neighborhood", getUuid());
        Kernel::iBind(input,"input", getUuid());
}

