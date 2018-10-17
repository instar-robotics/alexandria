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


#ifndef _SIGNAL_H_
#define _SIGNAL_H_

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"


/********************************************************************************************************/
/*******************************************  Front Detection   *****************************************/
/********************************************************************************************************/

const IString FD_SUP = "up" ;
const IString FD_SDOWN = "down" ;
const IString FD_SBOTH = "both" ;

const unsigned int  FD_IUP = 1 ;
const unsigned int FD_IDOWN = 0 ;
const unsigned int FD_IBOTH = -1 ;

bool checkMode(const std::string & smode, int &mode);

class SFrontDetection : public FScalar
{
        private :

                ISInput inScalar;
                ISInput threshold;
		IString mode;
		
		double z_1;
		int imode;

        public :

                virtual ~SFrontDetection(){}

                virtual void uprerun();
                virtual void compute();
                virtual void setparameters();
};

class MFrontDetection : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput threshold;
		IString mode;

		MatrixXd z_1;
		int imode;

        public :

                virtual ~MFrontDetection(){}

                virtual void uprerun();
                virtual void compute();
                virtual void setparameters();
};

class MMFrontDetection : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISMInput threshold;
		IString mode;
		
		MatrixXd z_1;
		int imode;

        public :

                virtual ~MMFrontDetection(){}

                virtual void uprerun();
                virtual void compute();
                virtual void setparameters();
};


/*******************************************************************************************************/
/******************************************  PiecewiseLin   ********************************************/
/*******************************************************************************************************/

// bounded [0,1]
class MPiecewiseLin : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MPiecewiseLin(){}

                virtual void compute();
                virtual void setparameters();
};

class SPiecewiseLin : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SPiecewiseLin(){}

                virtual void compute();
                virtual void setparameters();
};

// bounded [min,max]
class MPiecewiseLinCustom : public FMatrix
{
        private :

                ISMInput thresMin;
                ISMInput thresMax;

                ISMInput inMatrix;

        public :

                virtual ~MPiecewiseLinCustom(){}

                virtual void compute();
                virtual void setparameters();
};

class SPiecewiseLinCustom : public FScalar
{
        private :

                ISInput thresMin;
                ISInput thresMax;

                ISInput inScalar;

        public :

                virtual ~SPiecewiseLinCustom(){}

                virtual void compute();
                virtual void setparameters();
};

class MSSPiecewiseLinCustom : public FMatrix
{
        private :

                ISMInput inMatrix;

                ISInput thresMin;
                ISInput thresMax;

        public :

                virtual ~MSSPiecewiseLinCustom(){}

                virtual void compute();
                virtual void setparameters();
};


class MMSPiecewiseLinCustom : public FMatrix
{
        private :
                ISMInput inMatrix;

                ISMInput thresMin;
                ISInput thresMax;

        public :

                virtual ~MMSPiecewiseLinCustom(){}

                virtual void compute();
                virtual void setparameters();
};

class MSMPiecewiseLinCustom : public FMatrix
{
        private :

                ISMInput inMatrix;

                ISInput thresMin;
                ISMInput thresMax;

        public :

                virtual ~MSMPiecewiseLinCustom(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/*******************************************  Heaviside   **********************************************/
/*******************************************************************************************************/

template<typename Scalar>
struct HeaviFunc {
  HeaviFunc(const Scalar& thres) : thres(thres) {}
  const Scalar operator()(const Scalar& x) const { return  x < thres ? 0.0 : 1.0;   }
  Scalar thres;
};


// bounded [0,1]
class MHeaviside : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MHeaviside(){}

                virtual void compute();
                virtual void setparameters();
};

class SHeaviside : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SHeaviside(){}

                virtual void compute();
                virtual void setparameters();
};



class SHeavisideCustom : public FScalar
{
        private :

                ISInput inScalar;
		ISInput thres;

        public :

                virtual ~SHeavisideCustom(){}

                virtual void compute();
                virtual void setparameters();
};

class MSHeavisideCustom : public FMatrix
{
        private :

                ISMInput inMatrix;
		ISInput thres;

        public :

                virtual ~MSHeavisideCustom(){}

                virtual void compute();
                virtual void setparameters();
};

class MMHeavisideCustom : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISMInput thres;

        public :

                virtual ~MMHeavisideCustom(){}

                virtual void compute();
                virtual void setparameters();
};

#endif // _SIGNAL_H_
