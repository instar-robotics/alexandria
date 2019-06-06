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
#include <iostream>

void AmariDnf::compute()
{
	bool isCircular = false;
	if( circular()() >= 0.5 ) isCircular = true;

//	output = MATRIX::NullaryExpr( output.rows(), output.cols() , Amari_functor<MATRIX>( output, kernel()(), input()(), h()() , tau()() ,isCircular ));


	MATRIX::Scalar H = h()();
	MATRIX::Scalar T = tau()();
	const MATRIX& mask = kernel().i();
	MATRIX::Scalar wmask = kernel().w();

	MATRIX::Index urows = output.rows();
	MATRIX::Index ucols = output.cols();

	MATRIX::Index mrows = mask.rows();
	MATRIX::Index mcols = mask.cols();
		
	MATRIX U = output;
	
	const MATRIX& I = input().i(); 
	MATRIX::Scalar wi = input().w();
	
	//#pragma omp parallel for num_threads(NBTHREAD) collapse(2)
	//#pragma omp parallel for collapse(2)
	//#pragma omp parallel for num_threads(2) collapse(2)
	//#pragma omp parallel for collapse(2)
	//#pragma omp parallel for collapse(2) num_threads(4)
	//#pragma omp parallel for collapse(2) num_threads(4)
	#pragma omp parallel for num_threads(2) collapse(2)
	for( MATRIX::Index urow = 0; urow < urows ; urow++)
	{
		for( MATRIX::Index ucol = 0; ucol < ucols; ucol++)
		{
			MATRIX::Scalar neighbor = 0.0;

			for( MATRIX::Index i = 0; i < mrows; i++)
			{
				for( MATRIX::Index j = 0; j < mcols; j++)
				{
					MATRIX::Index zrow = urow + i - (MATRIX::Index)((mrows)/2.0);
					MATRIX::Index zcol = ucol + j - (MATRIX::Index)((mcols)/2.0); 

					if( isCircular )
					{
						zrow = (int)(zrow + urows) % urows;
						zcol = (int)(zcol + ucols) % ucols;
					}

					if( zrow >= 0 && zrow < urows && zcol >= 0 && zcol < ucols)
					{
						// Sigmoid as activation function
						//neighbor +=  mask(i,j) * (1 / (1 + exp(-beta * U(zrow,zcol))))  ;
						// classical ramp function
						SCALAR val =  U(zrow,zcol);
						if(  val > 1.0 ) val = 1.0;
						else if( val < 0.0 ) val = 0.0;
						neighbor += wmask * mask(i,j) * val;
					}
				 }
			 }
			 output(urow,ucol) = U(urow,ucol) + (T) * ( -U(urow,ucol) + wi * I(urow,ucol) + neighbor + H );
		}
	}
}

void AmariDnf::setparameters()
{
	kernel.setCheckSize(false);

        Kernel::iBind(circular,"circular", getUuid());
        Kernel::iBind(tau,"tau", getUuid());
        Kernel::iBind(h,"h", getUuid());
        Kernel::iBind(input,"input", getUuid());
        Kernel::iBind(kernel,"kernel", getUuid());
}
