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

	MatrixXd::Index ix = nearbyint( x()() );

	if( ix >= vout.size()) ix = vout.size() - 1;

	vout( lastIndex ) = 0;
	vout( ix ) = 1 ;

	lastIndex = ix;
}

void DiracField1D::setparameters()
{
	if( output.rows() != 1 && output.cols() != 1) throw std::invalid_argument("DiracField1D : output Matrix must be a vector [ROW or COL]");

	Kernel::iBind(x,"x", getUuid());
}

void DiracField2D::compute()
{
	output = MatrixXd::Constant(output.rows(),output.cols() , 0);
	
	MatrixXd::Index ix = nearbyint( x()() );
	MatrixXd::Index iy = nearbyint( y()() );
	
	if( ix >= output.cols()) ix = output.cols() - 1;
	if( iy >= output.rows()) iy = output.rows() - 1;

	output( iy, ix ) = 1 ;
	output( lastIy, lastIx ) = 0 ;

	lastIy = iy;
	lastIx = ix;
}

void DiracField2D::setparameters()
{
	Kernel::iBind(x,"x", getUuid());
	Kernel::iBind(y,"y", getUuid());
}

/*******************************************************************************************************/
/*****************************************  Heaviside Field  *******************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(HeavisideField1D);
REGISTER_FUNCTION(HeavisideField2D);

void HeavisideField1D::compute()
{
	auto vout = getMapVect(output); 

	MatrixXd::Index ithres = nearbyint( thres()() );
	vout = VectorXd::NullaryExpr( output.rows() * output.cols() , Heavi1D_functor<VectorXd>(ithres));
}

void HeavisideField1D::setparameters()
{
	if( output.rows() != 1 && output.cols() != 1) throw std::invalid_argument("HeavisideField1D : output Matrix must be a vector [ROW or COL]");

	Kernel::iBind(thres,"thres", getUuid());
}

void HeavisideField2D::compute()
{
	MatrixXd::Index ithres_x = nearbyint( thres_x()() );
	MatrixXd::Index ithres_y = nearbyint( thres_y()() );

	output = MatrixXd::NullaryExpr(output.rows(), output.cols(), Heavi2D_functor<MatrixXd>(ithres_x,ithres_y));
}

void HeavisideField2D::setparameters()
{
	Kernel::iBind(thres_x,"thres_x", getUuid());
	Kernel::iBind(thres_y,"thres_y", getUuid());
}

/*******************************************************************************************************/
/******************************************  Sinus Field  **********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(SinusField1D);

void SinusField1D::compute()
{
	auto vout = getMapVect(output); 

	vout = VectorXd::NullaryExpr( output.rows() * output.cols()  , Sin_functor<MatrixXd>( output.rows() * output.cols(), offset()(), freq()() ));
}

void SinusField1D::setparameters()
{
	if( output.rows() != 1 && output.cols() != 1) throw std::invalid_argument("SinusField1D : output Matrix must be a vector [ROW or COL]");

	Kernel::iBind(freq,"freq", getUuid());
	Kernel::iBind(offset,"offset", getUuid());
}

/*******************************************************************************************************/
/****************************************  Cosinus Field  **********************************************/
/*******************************************************************************************************/

REGISTER_FUNCTION(CosinusField1D);

void CosinusField1D::compute()
{
	auto vout = getMapVect(output); 

	vout = VectorXd::NullaryExpr( output.rows() * output.cols()  , Cosin_functor<MatrixXd>( output.rows() * output.cols(), offset()(), freq()() ));
}

void CosinusField1D::setparameters()
{
	if( output.rows() != 1 && output.cols() != 1) throw std::invalid_argument("CosinusField1D : output Matrix must be a vector [ROW or COL]");

	Kernel::iBind(freq,"freq", getUuid());
	Kernel::iBind(offset,"offset", getUuid());
}
