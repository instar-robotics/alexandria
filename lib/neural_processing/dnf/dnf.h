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


#ifndef __DNF_HPP__
#define __DNF_HPP__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

template<class ArgType>
class Amari_functor{
      
  const ArgType &U;
  const ArgType &mask;
  const ArgType &I;
  const typename ArgType::Scalar &h;
  const typename ArgType::Scalar &tau;
  //const typename ArgType::Scalar &beta;
  bool circular;

  public :

 //Amari_functor(const ArgType &U, const ArgType &mask, const ArgType &I, const typename ArgType::Scalar &h, const typename ArgType::Scalar &tau, const typename ArgType::Scalar &beta ,bool circular ) : U(U), mask(mask), I(I), h(h), tau(tau), beta(beta), circular(circular){}
 Amari_functor(const ArgType &U, const ArgType &mask, const ArgType &I, const typename ArgType::Scalar &h, const typename ArgType::Scalar &tau ,bool circular ) : U(U), mask(mask), I(I), h(h), tau(tau), circular(circular){}

 const typename ArgType::Scalar operator() (Index urow, Index ucol) const{
	 typename ArgType::Index mrows = mask.rows(); 
	 typename ArgType::Index mcols = mask.cols();
	 
	 typename ArgType::Index urows = U.rows(); 
	 typename ArgType::Index ucols = U.cols();

	 typename ArgType::Scalar neighbor = 0.0;

	 for( typename ArgType::Index i = 0; i < mrows; i++)
	 {
		 for( typename ArgType::Index j = 0; j < mcols; j++)
		 {
			typename ArgType::Index zrow = urow + i - mrows/2.0 ;
			typename ArgType::Index zcol = ucol + j - mcols/2.0 ; 
		
			if( circular ) 
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
				else if( val < 0 ) val = 0.0;
				neighbor +=  mask(i,j) * val;
			}
		 }
	 }

	 return U(urow,ucol) + tau * ( -U(urow,ucol) + I(urow,ucol) + neighbor + h );
 }
};

class AmariDnf : public FMatrix
{
	private : 
	
		ISInput circular;
		//ISInput beta;
		ISInput tau;
		ISInput h;

		ISMInput input;
		ISMInput kernel;

	public  :

		virtual ~AmariDnf(){}

		virtual void compute();
		virtual void setparameters();

};

#endif  // __DNF_HPP__
