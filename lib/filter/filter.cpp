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


#include "filter.h"

/*******************************************************************************************************/
/******************************************  Front Detection   *****************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(SFrontDetection);
REGISTER_FUNCTION(MSFrontDetection);
REGISTER_FUNCTION(MMFrontDetection);

bool checkMode(const std::string & smode, int& mode)
{
	bool ret = false;
	if( smode == FD_SUP)
	{	
		mode = FD_IUP;
		ret = true;
	}
	else if( smode == FD_SDOWN) 
	{
		mode = FD_IDOWN;
		ret = true;
	}
	else if( smode == FD_SBOTH)
	{	
		mode = FD_IBOTH;
		ret = true;
	}

	return ret;
}

SCALAR getFront(const SCALAR& z_1,const  SCALAR& z,const SCALAR &thres, int mode)
{
	SCALAR res = 0.0;
	if( z_1 < thres && thres <= z)
        {
                if( mode == FD_IBOTH || mode == FD_IUP ) res = 1.0;
        }
        else if( z_1 >= thres && thres > z )
        {
                if( mode == FD_IBOTH || mode == FD_IDOWN ) res = -1.0;
        }
	return res;
}

void SFrontDetection::prerun()
{
	if( !checkMode(mode, imode) ) throw std::invalid_argument("SFrontDetection : failed to check mode. Should be : "+FD_SUP+" , "+FD_SDOWN+" or "+FD_SBOTH);
}

void SFrontDetection::compute()
{
	output = getFront( z_1, inScalar()(), threshold()(), imode);
	z_1 = inScalar()();
}

void SFrontDetection::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
        Kernel::iBind(threshold,"threshold", getUuid());
        Kernel::iBind(mode,"mode", getUuid());
	z_1 = 0;
}

/**************************************************************************/

void MSFrontDetection::prerun()
{
	if( !checkMode(mode, imode)) throw std::invalid_argument("SFrontDetection : failed to check mode. Should be : "+FD_SUP+" , "+FD_SDOWN+" or "+FD_SBOTH);
}

void MSFrontDetection::compute()
{
	output = MATRIX::NullaryExpr( output.rows(), output.cols(), MSFD_Functor<MATRIX>(imode, threshold()(), z_1, inMatrix()() ));

	z_1 = inMatrix()();
}

void MSFrontDetection::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(threshold,"threshold", getUuid());
        Kernel::iBind(mode,"mode", getUuid());
	z_1 = MATRIX::Constant( output.rows(), output.cols(), 0  );
}

/**************************************************************************/

void MMFrontDetection::prerun()
{
	if( !checkMode(mode, imode)) throw std::invalid_argument("MMFrontDetection : failed to check mode. Should be : "+FD_SUP+" , "+FD_SDOWN+" or "+FD_SBOTH);
}

void MMFrontDetection::compute()
{
	output = MATRIX::NullaryExpr( output.rows(), output.cols(), MMFD_Functor<MATRIX>(imode, threshold()(), z_1, inMatrix()() ));

        z_1 = inMatrix()();

}

void MMFrontDetection::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(threshold,"threshold", getUuid());
        Kernel::iBind(mode,"mode", getUuid());
	
	z_1 = MATRIX::Constant( output.rows(), output.cols(), 0  );
}

/*******************************************************************************************************/
/***************************************   PIECEWISE Linear   ******************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MPiecewiseLin);
REGISTER_FUNCTION(SPiecewiseLin);
REGISTER_FUNCTION(MPiecewiseLinCustom);
REGISTER_FUNCTION(SPiecewiseLinCustom);
REGISTER_FUNCTION(MSSPiecewiseLinCustom);
REGISTER_FUNCTION(MMSPiecewiseLinCustom);
REGISTER_FUNCTION(MSMPiecewiseLinCustom);

void MPiecewiseLin::compute()
{
        output = (inMatrix()().array().max(0)).min(1)  ;
}

void MPiecewiseLin::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}


void SPiecewiseLin::compute()
{
        if( inScalar()() < 0 ) output = 0;
        else if( inScalar()() > 1 ) output = 1;
        else output = inScalar()();
}

void SPiecewiseLin::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}


void MPiecewiseLinCustom::compute()
{
        output = (inMatrix()().array().max( thresMin()().array() )).min( thresMax()().array() );
}

void MPiecewiseLinCustom::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(thresMin,"thresMin", getUuid());
        Kernel::iBind(thresMax,"thresMax", getUuid());
}

void SPiecewiseLinCustom::compute()
{
        if( inScalar()() < thresMin()() ) output = thresMin()();
        else if( inScalar()() > thresMax()() ) output = thresMax()() ;
        else output = inScalar()();
}

void SPiecewiseLinCustom::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
        Kernel::iBind(thresMin,"thresMin", getUuid());
        Kernel::iBind(thresMax,"thresMax", getUuid());
}

void MSSPiecewiseLinCustom::compute()
{
        output = (inMatrix()().array().max( thresMin()()  )).min( thresMax()() )  ;
}

void MSSPiecewiseLinCustom::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(thresMin,"thresMin", getUuid());
        Kernel::iBind(thresMax,"thresMax", getUuid());
}

void MMSPiecewiseLinCustom::compute()
{
        output = (inMatrix()().array().max( thresMin()().array() )).min( thresMax()() );
}

void MMSPiecewiseLinCustom::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(thresMin,"thresMin", getUuid());
        Kernel::iBind(thresMax,"thresMax", getUuid());
}

void MSMPiecewiseLinCustom::compute()
{
        output = (inMatrix()().array().max( thresMin()() )).min( thresMax()().array() );
}

void MSMPiecewiseLinCustom::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(thresMin,"thresMin", getUuid());
        Kernel::iBind(thresMax,"thresMax", getUuid());
}

/*******************************************************************************************************/
/******************************************   HEAVISIDE   **********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(MHeaviside);
REGISTER_FUNCTION(SHeaviside);
REGISTER_FUNCTION(MMHeavisideCustom);
REGISTER_FUNCTION(MSHeavisideCustom);
REGISTER_FUNCTION(SHeavisideCustom);

void MHeaviside::compute()
{
 	output = inMatrix()().unaryExpr([](SCALAR elem)
    	{
		return elem < 1.0 ? 0.0 : 1.0; 
    	});
}

void MHeaviside::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}


void SHeaviside::compute()
{
        if( inScalar()() < 1.0 ) output = 0;
        else output = 1;
}

void SHeaviside::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}


void SHeavisideCustom::compute()
{
        if( inScalar()() < thres()() ) output = 0;
        else output = 1;
}

void SHeavisideCustom::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
        Kernel::iBind(thres,"thres", getUuid());
}

void MSHeavisideCustom::compute()
{
        output = inMatrix()().unaryExpr(HeaviFunc<SCALAR>(thres()()));
}

void MSHeavisideCustom::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(thres,"thres", getUuid());
}

void MMHeavisideCustom::compute()
{
        output = inMatrix()().binaryExpr(thres()(), [](SCALAR e1, SCALAR e2)
        {
                return e1 < e2 ? 0.0 : 1.0;
        });
}

void MMHeavisideCustom::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(thres,"thres", getUuid());
}

/*******************************************************************************************************/
/*******************************************   SIGMOID   ***********************************************/
/*******************************************************************************************************/
REGISTER_FUNCTION(SSigmoid);
REGISTER_FUNCTION(MSigmoid);
REGISTER_FUNCTION(SSigmoidLambda);
REGISTER_FUNCTION(MSSigmoidLambda);
REGISTER_FUNCTION(MMSigmoidLambda);

void SSigmoid::compute()
{
	output = 1 / (1 + exp( -inScalar()())); 
}

void SSigmoid::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
}


void MSigmoid::compute()
{
	output = 1 / (1 +  exp( - (inMatrix()().array()))) ; 
}

void MSigmoid::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}


void SSigmoidLambda::compute()
{
	output = 1 / (1 + exp( -(lambda()() * inScalar()())) ); 
}

void SSigmoidLambda::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
        Kernel::iBind(lambda,"lambda", getUuid());
}

void MSSigmoidLambda::compute()
{
	output = 1 / (1 +  exp( - (lambda()() * (inMatrix()().array())))) ; 
}

void MSSigmoidLambda::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(lambda,"lambda", getUuid());
}

void MMSigmoidLambda::compute()
{
	output = 1 / (1 +  exp( - (inMatrix()() * lambda()()).array())) ; 
}

void MMSigmoidLambda::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(lambda,"lambda", getUuid());
}

/*******************************************************************************************************/
/*********************************************  Gaussian   *********************************************/
/*******************************************************************************************************/

void SGauss::compute()
{
        output = (1 / (sigma()() * sqrt(2 * M_PI))) * exp( - pow(inScalar()() - mu()(), 2) / (2 * pow(sigma()(),2) ));
}

void SGauss::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
        Kernel::iBind(mu,"mu", getUuid());
        Kernel::iBind(sigma,"sigma", getUuid());
}


void MSGauss::compute()
{
        output = (1 / (sigma()() * sqrt(2 * M_PI))) * exp( - pow(inMatrix()().array() - mu()(), 2) / (2 * pow(sigma()(),2) ));
}

void MSGauss::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(mu,"mu", getUuid());
        Kernel::iBind(sigma,"sigma", getUuid());
}

void MMGauss::compute()
{
        output = (1 / (sigma()().array() * sqrt(2 * M_PI))) * exp( - (inMatrix()() -mu()()).array().square()  / (2 * sigma()().array().square()) ) ;
}

void MMGauss::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(mu,"mu", getUuid());
        Kernel::iBind(sigma,"sigma", getUuid());
}


/*******************************************************************************************************/
/***********************************************  DoG  *************************************************/
/*******************************************************************************************************/

void SDoG::compute()
{
        output = (1 / (sigma1()() * sqrt(2*M_PI))) * exp( -( pow(inScalar()() - mu1()(), 2) / (2*pow(sigma1()(),2)))) -  (1 / (sigma2()() * sqrt(2*M_PI))) * exp( -( pow(inScalar()() - mu2()(), 2) / (2*pow(sigma2()(),2)))) ;
}

void SDoG::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
        Kernel::iBind(mu1,"mu1", getUuid());
        Kernel::iBind(sigma1,"sigma1", getUuid());
        Kernel::iBind(mu2,"mu2", getUuid());
        Kernel::iBind(sigma2,"sigma2", getUuid());
}


void MSDoG::compute()
{
        output = (1 / (sigma1()() * sqrt(2*M_PI))) * exp( -( pow( inMatrix()().array() - mu1()(), 2) / (2*pow(sigma1()(),2)))) -  (1 / (sigma2()() * sqrt(2*M_PI))) * exp( -( pow( inMatrix()().array() - mu2()(), 2) / (2*pow(sigma2()(),2)))) ;
}

void MSDoG::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(mu1,"mu1", getUuid());
        Kernel::iBind(sigma1,"sigma1", getUuid());
        Kernel::iBind(mu2,"mu2", getUuid());
        Kernel::iBind(sigma2,"sigma2", getUuid());
}


void MMDoG::compute()
{
        output = (1 / (sigma1()().array() * sqrt(2*M_PI))) * exp( -( (inMatrix()() - mu1()()).array().square()  / (2 * sigma1()().array().square()  ))) -  (1 / (sigma2()().array().square() * sqrt(2*M_PI))) * exp( -( (inMatrix()() - mu2()()).array().square()  / ( 2 * sigma2()().array().square()  ))) ;
}

void MMDoG::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(mu1,"mu1", getUuid());
        Kernel::iBind(sigma1,"sigma1", getUuid());
        Kernel::iBind(mu2,"mu2", getUuid());
        Kernel::iBind(sigma2,"sigma2", getUuid());
}

