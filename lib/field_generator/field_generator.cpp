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


#include "field_generator.h"


/*******************************************************************************************************/
/*******************************************  Dirac Field  *********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(DiracField1D);
REGISTER_FUNCTION(DiracField2D);

void DiracField1D::compute()
{
	auto vout = getMapVect(output);

	MATRIX::Index ix = nearbyint( (x()() + N()()) * vout.size() / (2.0 * N()()) );

	if( ix >= vout.size()) ix = vout.size() - 1;
	if( ix < 0) ix = 0;

	vout( lastIndex ) = 0;
	vout( ix ) = 1 ;

	lastIndex = ix;
}

void DiracField1D::setparameters()
{
	Kernel::iBind(x,"x", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

void DiracField2D::compute()
{
	output = MATRIX::Constant(output.rows(),output.cols() , 0);
	
	MATRIX::Index ix = nearbyint( (x()() + N()()) * output.cols() / (2.0 * N()()) );
	MATRIX::Index iy = nearbyint( (y()() + N()()) * output.rows() / (2.0 * N()()) );
	
	if( ix >= output.cols()) ix = output.cols() - 1;
	if( iy >= output.rows()) iy = output.rows() - 1;
	if( ix < 0) ix = 0;
	if( iy < 0) iy = 0;

	output( lastIy, lastIx ) = 0 ;
	output( iy, ix ) = 1 ;

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
	vout = VectorXs::NullaryExpr( vout.size() , Heavi1D_functor<VectorXs>( N()() , th()() , vout.size() ));
}

void HeavisideField1D::setparameters()
{
	Kernel::iBind(th,"th", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

void HeavisideField2D::compute()
{
	output = MATRIX::NullaryExpr(output.rows(), output.cols(), Heavi2D_functor<MATRIX>( N()() , thx()(),thy()(), output.cols() , output.rows() ));
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
        vout = VectorXs::NullaryExpr( vout.size() , Gate1D_functor<VectorXs>( N()() , wth()(), vout.size()));
}

void GateField1D::setparameters()
{
        Kernel::iBind(wth,"wth", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

void GateField2D::compute()
{
        output = MATRIX::NullaryExpr(output.rows(), output.cols(), Gate2D_functor<MATRIX>( N()(), wthx()(),wthy()(), output.cols(), output.rows()));
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
        vout = VectorXs::NullaryExpr( vout.size() , Triangular1D_functor<VectorXs>( N()() , a()(), vout.size()));
}

void TriangularField1D::setparameters()
{
        Kernel::iBind(a,"a", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

void TriangularField2D::compute()
{
        output = MATRIX::NullaryExpr( output.rows(), output.cols() , Triangular2D_functor<MATRIX>( N()() , ax()(), ay()(), output.cols(), output.rows()));
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
	vout = VectorXs::NullaryExpr( vout.size() , Sin1D_functor<VectorXs>(freq()(), offset()(), vout.size()));
}

void SinusField1D::setparameters()
{
	Kernel::iBind(freq,"freq", getUuid());
	Kernel::iBind(offset,"offset", getUuid());
}

void SinusField2D::compute()
{
	output = MATRIX::NullaryExpr( output.rows(), output.cols() , Sin2D_functor<MATRIX>( freq_x()(), offset_x()(), freq_y()(), offset_y()() , output.cols() , output.rows() ));
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
	vout = VectorXs::NullaryExpr( vout.size() , Cos1D_functor<VectorXs>(freq()(), offset()(), vout.size()));
}

void CosinusField1D::setparameters()
{
	Kernel::iBind(freq,"freq", getUuid());
	Kernel::iBind(offset,"offset", getUuid());
}

void CosinusField2D::compute()
{
	output = MATRIX::NullaryExpr( output.rows(), output.cols() , Cos2D_functor<MATRIX>( freq_x()(), offset_x()(), freq_y()(), offset_y()() , output.cols() , output.rows() ));
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
	
	SCALAR s = sigma()();
	if(s==0.0) s = std::numeric_limits<SCALAR>::epsilon();

	vout = VectorXs::NullaryExpr( vout.size()  ,Gauss1D_functor<VectorXs>(N()(), s, mu()() , vout.size() ));
}

void GaussianField1D::setparameters()
{
	Kernel::iBind(mu,"mu", getUuid());
	Kernel::iBind(sigma,"sigma", getUuid());
	Kernel::iBind(N,"N", getUuid());
}

void GaussianField2D::compute()
{
	SCALAR sx = sigma_x()();
	SCALAR sy = sigma_y()();
	if(sx == 0.0) sx = std::numeric_limits<SCALAR>::epsilon();
	if(sy == 0.0) sy = std::numeric_limits<SCALAR>::epsilon();

	output = MATRIX::NullaryExpr( output.rows(), output.cols()  ,Gauss2D_functor<MATRIX>(N()(), sx, mu_x()(), sy , mu_y()(), MATRIX::Scalar(output.cols()),MATRIX::Scalar( output.rows()) ));
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
	
	SCALAR s1 = sigma1()();
	SCALAR s2 = sigma2()();

	if( s1 == 0 ) s1 = std::numeric_limits<SCALAR>::epsilon();
	if( s2 == 0 ) s2 = std::numeric_limits<SCALAR>::epsilon();

	vout = VectorXs::NullaryExpr( vout.size() , DoG1D_functor<VectorXs>(N()() , s1, s2, vout.size()));
}

void DoGField1D::setparameters()
{
	Kernel::iBind(sigma1,"sigma1", getUuid());
	Kernel::iBind(sigma2,"sigma2", getUuid());
	Kernel::iBind(N,"N", getUuid());
}


void DoGField2D::compute()
{
        SCALAR s1_x = sigma1_x()();
        SCALAR s1_y = sigma1_y()();
        SCALAR s2_x = sigma2_x()();
        SCALAR s2_y = sigma2_y()();

        if( s1_x == 0 ) s1_x = std::numeric_limits<SCALAR>::epsilon();
        if( s1_y == 0 ) s1_y = std::numeric_limits<SCALAR>::epsilon();
        if( s2_x == 0 ) s2_x = std::numeric_limits<SCALAR>::epsilon();
        if( s2_y == 0 ) s2_y = std::numeric_limits<SCALAR>::epsilon();

        output = MATRIX::NullaryExpr( output.rows(), output.cols() , DoG2D_functor<MATRIX>(N()(), s1_x, s1_y , s2_x, s2_y, (output.cols()), (output.rows())));
}

void DoGField2D::setparameters()
{
        Kernel::iBind(sigma1_x,"sigma1_x", getUuid());
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
        vout = VectorXs::NullaryExpr( vout.size() , Sinc1D_functor<VectorXs>( N()(), freq()(),  vout.size() ));
}

void SincField1D::setparameters()
{
        Kernel::iBind(freq,"freq", getUuid());
        Kernel::iBind(N,"N", getUuid());
}

void SincField2D::compute()
{
        output = MATRIX::NullaryExpr( output.rows(), output.cols() , Sinc2D_functor<MATRIX>( N()(), freq()(), output.cols() , output.rows() ));
}

void SincField2D::setparameters()
{
        Kernel::iBind(freq,"freq", getUuid());
        Kernel::iBind(N,"N", getUuid());
}

/*******************************************************************************************************/
/********************************************  Chinese Hat Field  **************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(ChineseHatField);

void ChineseHatField::compute()
{
	if( isVect())
	{
        	auto vout = getMapVect(output);
        	vout = VectorXs::NullaryExpr( vout.size(), ChineseHat1D_functor<VectorXs>( N()() ,vout.size() ));
	}
	else 
	{
		output = MATRIX::NullaryExpr( output.rows(), output.cols() , 	ChineseHat2D_functor<MATRIX>( N()(), output.cols(), output.rows()));
	}
}

void ChineseHatField::setparameters()
{
        Kernel::iBind(N,"N", getUuid());
}



/*******************************************************************************************************/
/*****************************************  Random Field  **********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(RandomField);

void RandomField::compute()
{
	output = MATRIX::Random( output.rows(), output.cols() );
}

void RandomField::setparameters()
{

}


