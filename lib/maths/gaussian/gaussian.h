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

#ifndef _GAUSSIAN_H_
#define _GAUSSIAN_H_

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"


/*******************************************************************************************************/
/**********************************************  Gaussian   ********************************************/
/*******************************************************************************************************/

class SGauss : public FScalar
{
        private :

                ISInput inScalar;
                ISInput mu;
                ISInput sigma;

        public :
                virtual ~SGauss(){}

                virtual void compute();
                virtual void setparameters();
};

class MSGauss : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput mu;
                ISInput sigma;

        public :
                virtual ~MSGauss(){}

                virtual void compute();
                virtual void setparameters();
};

class MMGauss : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISMInput mu;
                ISMInput sigma;

        public :
                virtual ~MMGauss(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/**************************************************  DoG ***********************************************/
/*******************************************************************************************************/

class SDoG : public FScalar
{
        private :
                ISInput inScalar;
                ISInput mu1;
                ISInput sigma1;

                ISInput mu2;
                ISInput sigma2;

        public :
                virtual ~SDoG(){}

                virtual void compute();
                virtual void setparameters();
};

class MSDoG : public FMatrix
{
        private :
                ISMInput inMatrix;
                ISInput mu1;
                ISInput sigma1;

                ISInput mu2;
                ISInput sigma2;

        public :
                virtual ~MSDoG(){}

                virtual void compute();
                virtual void setparameters();
};

class MMDoG : public FMatrix
{
        private :
                ISMInput inMatrix;
                ISMInput mu1;
                ISMInput sigma1;

                ISMInput mu2;
                ISMInput sigma2;

        public :
                virtual ~MMDoG(){}

                virtual void compute();
                virtual void setparameters();
};

#endif //_GAUSSIAN_H_
