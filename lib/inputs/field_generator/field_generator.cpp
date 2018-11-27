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

#include "field_generator.h"


/*******************************************************************************************************/
/*******************************************  Dirac Field  *********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(DiracField1D);
REGISTER_FUNCTION(DiracField2D);

void DiracField1D::compute()
{
	auto vout = getMapVect(output);

	MatrixXd::Index ix = nearbyint( (x()() + N()()) * vout.size() / (2.0 * N()()) );

	if( ix >= vout.size()) ix = vout.size() - 1;
	if( ix < 0) ix = 0;

	vout( lastIndex ) = 0;
	vout( ix ) = 1 ;

	lastIndex = ix;
}

void DiracField1D::setparameters()
{
	if( output.rows() != 1 && output.cols() != 1) throw std::invalid_argument("DiracField1D : output Matrix must be a vector [ROW or COL]");

	Kernel::iBind(x,"x", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

void DiracField2D::compute()
{
	output = MatrixXd::Constant(output.rows(),output.cols() , 0);
	
	MatrixXd::Index ix = nearbyint( (x()() + N()()) * output.cols() / (2.0 * N()()) );
	MatrixXd::Index iy = nearbyint( (y()() + N()()) * output.rows() / (2.0 * N()()) );
	
	if( ix >= output.cols()) ix = output.cols() - 1;
	if( iy >= output.rows()) iy = output.rows() - 1;
	if( ix < 0) ix = 0;
	if( iy < 0) iy = 0;

	output( iy, ix ) = 1 ;
	output( lastIy, lastIx ) = 0 ;

	lastIy = iy;
	lastIx = ix;
}

void DiracField2D::setparameters()
{
	Kernel::iBind(x,"x", getUuid());
	Kernel::iBind(y,"y", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

/*******************************************************************************************************/
/*****************************************  Heaviside Field  *******************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(HeavisideField1D);
REGISTER_FUNCTION(HeavisideField2D);

void HeavisideField1D::compute()
{
	auto vout = getMapVect(output); 
	vout = VectorXd::NullaryExpr( vout.size() , Heavi1D_functor<VectorXd>( N()() , th()() , vout.size() ));
}

void HeavisideField1D::setparameters()
{
	if( output.rows() != 1 && output.cols() != 1) throw std::invalid_argument("HeavisideField1D : output Matrix must be a vector [ROW or COL]");

	Kernel::iBind(th,"th", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

void HeavisideField2D::compute()
{
	output = MatrixXd::NullaryExpr(output.rows(), output.cols(), Heavi2D_functor<MatrixXd>( N()() , thx()(),thy()(), output.cols() , output.rows() ));
}

void HeavisideField2D::setparameters()
{
	Kernel::iBind(thx,"thx", getUuid());
	Kernel::iBind(thy,"thy", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

/*******************************************************************************************************/
/*******************************************  Gate Field  **********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(GateField1D);
REGISTER_FUNCTION(GateField2D);

void GateField1D::compute()
{
        auto vout = getMapVect(output);
        vout = VectorXd::NullaryExpr( vout.size() , Gate1D_functor<VectorXd>( N()() , wth()(), vout.size()));
}

void GateField1D::setparameters()
{
        Kernel::iBind(wth,"wth", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

void GateField2D::compute()
{
        output = MatrixXd::NullaryExpr(output.rows(), output.cols(), Gate2D_functor<MatrixXd>( N()(), wthx()(),wthy()(), output.cols(), output.rows()));
}

void GateField2D::setparameters()
{
        Kernel::iBind(wthx,"wthx", getUuid());
        Kernel::iBind(wthy,"wthy", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

/*******************************************************************************************************/
/*****************************************  Triangular Field  ******************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(TriangularField1D);
REGISTER_FUNCTION(TriangularField2D);

void TriangularField1D::compute()
{
        auto vout = getMapVect(output);
        vout = VectorXd::NullaryExpr( vout.size() , Triangular1D_functor<VectorXd>( N()() , a()(), vout.size()));
}

void TriangularField1D::setparameters()
{
        Kernel::iBind(a,"a", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

void TriangularField2D::compute()
{
        output = MatrixXd::NullaryExpr( output.rows(), output.cols() , Triangular2D_functor<MatrixXd>( N()() , ax()(), ay()(), output.cols(), output.rows()));
}

void TriangularField2D::setparameters()
{
        Kernel::iBind(ax,"ax", getUuid());
        Kernel::iBind(ay,"ay", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

/*******************************************************************************************************/
/******************************************  Sinus Field  **********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(SinusField1D);
REGISTER_FUNCTION(SinusField2D);

void SinusField1D::compute()
{
	auto vout = getMapVect(output); 
	vout = VectorXd::NullaryExpr( vout.size() , Sin1D_functor<VectorXd>(freq()(), offset()(), vout.size()));
}

void SinusField1D::setparameters()
{
	if( output.rows() != 1 && output.cols() != 1) throw std::invalid_argument("SinusField1D : output Matrix must be a vector [ROW or COL]");

	Kernel::iBind(freq,"freq", getUuid());
	Kernel::iBind(offset,"offset", getUuid());
}

void SinusField2D::compute()
{
	output = MatrixXd::NullaryExpr( output.rows(), output.cols() , Sin2D_functor<MatrixXd>( freq_x()(), offset_x()(), freq_y()(), offset_y()() , output.cols() , output.rows() ));
}

void SinusField2D::setparameters()
{
	Kernel::iBind(freq_x,"freq_x", getUuid());
	Kernel::iBind(offset_x,"offset_x", getUuid());
	Kernel::iBind(freq_y,"freq_y", getUuid());
	Kernel::iBind(offset_y,"offset_y", getUuid());
}

/*******************************************************************************************************/
/****************************************  Cosinus Field  **********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(CosinusField1D);
REGISTER_FUNCTION(CosinusField2D);

void CosinusField1D::compute()
{
	auto vout = getMapVect(output); 
	vout = VectorXd::NullaryExpr( vout.size() , Cos1D_functor<VectorXd>(freq()(), offset()(), vout.size()));
}

void CosinusField1D::setparameters()
{
	if( output.rows() != 1 && output.cols() != 1) throw std::invalid_argument("CosinusField1D : output Matrix must be a vector [ROW or COL]");

	Kernel::iBind(freq,"freq", getUuid());
	Kernel::iBind(offset,"offset", getUuid());
}

void CosinusField2D::compute()
{
	output = MatrixXd::NullaryExpr( output.rows(), output.cols() , Cos2D_functor<MatrixXd>( freq_x()(), offset_x()(), freq_y()(), offset_y()() , output.cols() , output.rows() ));
}

void CosinusField2D::setparameters()
{
	Kernel::iBind(freq_x,"freq_x", getUuid());
	Kernel::iBind(offset_x,"offset_x", getUuid());
	Kernel::iBind(freq_y,"freq_y", getUuid());
	Kernel::iBind(offset_y,"offset_y", getUuid());
}

/*******************************************************************************************************/
/*****************************************  Gaussian Field  ********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(GaussianField1D);
REGISTER_FUNCTION(GaussianField2D);

void GaussianField1D::compute()
{
	auto vout = getMapVect(output); 
	
	double s = sigma()();
	if(s==0.0) s = std::numeric_limits<double>::epsilon();

	vout = VectorXd::NullaryExpr( vout.size()  ,Gauss1D_functor<VectorXd>(N()(), s, mu()() , vout.size() ));
}

void GaussianField1D::setparameters()
{
	if( output.rows() != 1 && output.cols() != 1) throw std::invalid_argument("GaussianField1D : output Matrix must be a vector [ROW or COL]");

	Kernel::iBind(mu,"mu", getUuid());
	Kernel::iBind(sigma,"sigma", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

void GaussianField2D::compute()
{
	auto vout = getMapVect(output); 
	
	double sx = sigma_x()();
	double sy = sigma_y()();
	if(sx == 0.0) sx = std::numeric_limits<double>::epsilon();
	if(sy == 0.0) sy = std::numeric_limits<double>::epsilon();

	output = MatrixXd::NullaryExpr( output.rows(), output.cols()  ,Gauss2D_functor<MatrixXd>(N()(), sx, mu_x()(), sy , mu_y()(), output.cols(), output.rows() ));
}

void GaussianField2D::setparameters()
{
	Kernel::iBind(mu_x,"mu_x", getUuid());
	Kernel::iBind(sigma_x,"sigma_x", getUuid());
	Kernel::iBind(mu_y,"mu_y", getUuid());
	Kernel::iBind(sigma_y,"sigma_y", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

/*******************************************************************************************************/
/*******************************************  DoG Field  ***********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(DoGField1D);
REGISTER_FUNCTION(DoGField2D);

void DoGField1D::compute()
{
	auto vout = getMapVect(output); 
	
	double s1 = sigma1()();
	double s2 = sigma2()();

	if( s1 == 0 ) s1 = std::numeric_limits<double>::epsilon();
	if( s2 == 0 ) s2 = std::numeric_limits<double>::epsilon();

	vout = VectorXd::NullaryExpr( vout.size() , DoG1D_functor<VectorXd>(N()() , s1, s2, vout.size()));
}

void DoGField1D::setparameters()
{
	if( output.rows() != 1 && output.cols() != 1) throw std::invalid_argument("DoGField1D : output Matrix must be a vector [ROW or COL]");

	Kernel::iBind(sigma1,"sigma1", getUuid());
	Kernel::iBind(sigma2,"sigma2", getUuid());
	Kernel::iBind(N,"N", getUuid());
}


void DoGField2D::compute()
{
        double s1_x = sigma1_x()();
        double s1_y = sigma1_y()();
        double s2_x = sigma2_x()();
        double s2_y = sigma2_y()();

        if( s1_x == 0 ) s1_x = std::numeric_limits<double>::epsilon();
        if( s1_y == 0 ) s1_y = std::numeric_limits<double>::epsilon();
        if( s2_x == 0 ) s2_x = std::numeric_limits<double>::epsilon();
        if( s2_y == 0 ) s2_y = std::numeric_limits<double>::epsilon();

        output = MatrixXd::NullaryExpr( output.rows(), output.cols() , DoG2D_functor<MatrixXd>(N()(), s1_x, s1_y , s2_x, s2_y, output.cols(), output.rows()));

}

void DoGField2D::setparameters()
{
        Kernel::iBind(sigma1_x,"sigma1_y", getUuid());
        Kernel::iBind(sigma1_y,"sigma1_y", getUuid());
        Kernel::iBind(sigma2_x,"sigma2_x", getUuid());
        Kernel::iBind(sigma2_y,"sigma2_y", getUuid());
        Kernel::iBind(N,"N", getUuid());
}

/*******************************************************************************************************/
/******************************************  Sinus Cardinal Field  *************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(SincField1D);
REGISTER_FUNCTION(SincField2D);

void SincField1D::compute()
{
        auto vout = getMapVect(output);
        vout = VectorXd::NullaryExpr( vout.size() , Sinc1D_functor<VectorXd>( N()(), freq()(),  vout.size() ));
}

void SincField1D::setparameters()
{
        if( output.rows() != 1 && output.cols() != 1) throw std::invalid_argument("SinusField1D : output Matrix must be a vector [ROW or COL]");

        Kernel::iBind(freq,"freq", getUuid());
        Kernel::iBind(N,"N", getUuid());
}

void SincField2D::compute()
{
        output = MatrixXd::NullaryExpr( output.rows(), output.cols() , Sinc2D_functor<MatrixXd>( N()(), freq_x()(), freq_y()(),   output.cols() , output.rows() ));
}

void SincField2D::setparameters()
{
        Kernel::iBind(freq_x,"freq_x", getUuid());
        Kernel::iBind(freq_y,"freq_y", getUuid());
        Kernel::iBind(N,"N", getUuid());
}

/*******************************************************************************************************/
/********************************************  Chinese Hat Field  **************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(ChineseHatField);

void ChineseHatField::compute()
{
	if( dim == 1)
	{
        	auto vout = getMapVect(output);
        	vout = VectorXd::NullaryExpr( vout.size(), ChineseHat1D_functor<VectorXd>( N()() ,vout.size() ));
	}
	else if( dim == 2 ) 
	{
		output = MatrixXd::NullaryExpr( output.rows(), output.cols() , 	ChineseHat2D_functor<MatrixXd>( N()(), output.cols(), output.rows()));
	}
}

void ChineseHatField::setparameters()
{
        if( output.rows() != 1 && output.cols() != 1) 
	{
		dim = 2 ;
	}
	else dim = 1;

        Kernel::iBind(N,"N", getUuid());
}

