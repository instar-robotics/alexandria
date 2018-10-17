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

#include "signal.h"

/********************************************************************************************************/
/*******************************************  Front Detection   *****************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(SFrontDetection);
REGISTER_FUNCTION(MFrontDetection);
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


void SFrontDetection::uprerun()
{
	if( !checkMode(mode, imode) ) throw std::invalid_argument("SFrontDetection : failed to check mode. Should be : "+FD_SUP+" , "+FD_SDOWN+" or "+FD_SBOTH);
}

void SFrontDetection::compute()
{
        static double eps = std::numeric_limits<double>::epsilon();

	double grad_1 = threshold()() - z_1;	
	double grad = inScalar()() - threshold()();	
	
        output =((grad+eps)/abs(grad+eps) + (grad_1+eps)/abs(grad_1+eps))/2 ;

	switch(imode)
	{
		case FD_IUP : 
			output = std::max(output,0.0); 
			break;
		case FD_IDOWN :
			output = std::min(output,0.0); 
			break;
	}
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

void MFrontDetection::uprerun()
{
	if( !checkMode(mode, imode)) throw std::invalid_argument("SFrontDetection : failed to check mode. Should be : "+FD_SUP+" , "+FD_SDOWN+" or "+FD_SBOTH);
}

void MFrontDetection::compute()
{
        static double eps = std::numeric_limits<double>::epsilon();

        auto grad_1 = threshold()() - z_1.array() ;
        auto grad = inMatrix()().array() - threshold()() ;

        output =((grad+eps)/abs(grad+eps) + (grad_1+eps)/abs(grad_1+eps))/2 ;
	
	switch(imode)
	{
		case FD_IUP : 
			output.array() = output.array().max(0.0); 
			break;
		case FD_IDOWN :
			output.array() = output.array().min(0.0); 
			break;
	}

	inMatrix()(z_1);
}

void MFrontDetection::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(threshold,"threshold", getUuid());
        Kernel::iBind(mode,"mode", getUuid());
	z_1 = MatrixXd::Constant( output.rows(), output.cols(), 0  );
}

/**************************************************************************/

void MMFrontDetection::uprerun()
{
	if( !checkMode(mode, imode)) throw std::invalid_argument("SFrontDetection : failed to check mode. Should be : "+FD_SUP+" , "+FD_SDOWN+" or "+FD_SBOTH);
}

void MMFrontDetection::compute()
{
	static double eps = std::numeric_limits<double>::epsilon();
	
 	MatrixXd grad_1; 
	threshold()(grad_1);
	grad_1.array() = z_1;

	MatrixXd grad;
        inMatrix()(grad);
        grad -= threshold() ;

        output =((grad.array()+eps)/abs(grad.array()+eps) + (grad_1.array()+eps)/abs(grad_1.array()+eps))/2 ;

        switch(imode)
        {
                case FD_IUP :
                        output.array() = output.array().max(0.0);
                        break;
                case FD_IDOWN :
                        output.array() = output.array().min(0.0);
                        break;
        }

        inMatrix()(z_1);

}

void MMFrontDetection::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(threshold,"threshold", getUuid());
        Kernel::iBind(mode,"mode", getUuid());
	
	z_1 = MatrixXd::Constant( output.rows(), output.cols(), 0  );
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
        output = (inMatrix()(output).array().max(0)).min(1)  ;
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
        output = (inMatrix()(output).array().max( thresMin()().array() )).min( thresMax()().array() );
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
        output = (inMatrix()(output).array().max( thresMin()()  )).min( thresMax()() )  ;
}

void MSSPiecewiseLinCustom::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(thresMin,"thresMin", getUuid());
        Kernel::iBind(thresMax,"thresMax", getUuid());
}

void MMSPiecewiseLinCustom::compute()
{
        output = (inMatrix()(output).array().max( thresMin()().array() )).min( thresMax()() );
}

void MMSPiecewiseLinCustom::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(thresMin,"thresMin", getUuid());
        Kernel::iBind(thresMax,"thresMax", getUuid());
}

void MSMPiecewiseLinCustom::compute()
{
        output = (inMatrix()(output).array().max( thresMin()() )).min( thresMax()().array() );
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
 	output = inMatrix()(output).unaryExpr([](double elem)
    	{
		return elem < 0.0 ? 0.0 : 1.0; 
    	});
}

void MHeaviside::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
}


void SHeaviside::compute()
{
        if( inScalar()() < 0 ) output = 0;
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
        output = inMatrix()(output).unaryExpr(HeaviFunc<double>(thres()()));
}

void MSHeavisideCustom::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(thres,"thres", getUuid());
}

void MMHeavisideCustom::compute()
{
        output = inMatrix()(output).binaryExpr(thres()(), [](double e1, double e2)
        {
                return e1 < e2 ? 0.0 : 1.0;
        });
}

void MMHeavisideCustom::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(thres,"thres", getUuid());
}
