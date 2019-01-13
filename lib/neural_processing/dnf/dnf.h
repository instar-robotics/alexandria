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
  const typename ArgType::Scalar &beta;
  bool circular;

  public :

 Amari_functor(const ArgType &U, const ArgType &mask, const ArgType &I, const typename ArgType::Scalar &h, const typename ArgType::Scalar &tau, const typename ArgType::Scalar &beta ,bool circular ) : U(U), mask(mask), I(I), h(h), tau(tau), beta(beta), circular(circular){}

 const typename ArgType::Scalar operator() (Index urow, Index ucol) const{
	 static typename ArgType::Index mrows = mask.rows(); 
	 static typename ArgType::Index mcols = mask.cols();
	 
	 static typename ArgType::Index urows = U.rows(); 
	 static typename ArgType::Index ucols = U.cols();

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
				neighbor +=  mask(i,j) * (1 / (1 + exp(-beta * U(zrow,zcol))))  ;
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
		ISInput beta;
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
