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

#include "gaussian.h"


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
/***************************************  Gaussian Difference  *****************************************/
/*******************************************************************************************************/

void SGaussDiff::compute()
{
        output = (1 / (sigma1()() * sqrt(2*M_PI))) * exp( -( pow(inScalar()() - mu1()(), 2) / (2*pow(sigma1()(),2)))) -  (1 / (sigma2()() * sqrt(2*M_PI))) * exp( -( pow(inScalar()() - mu2()(), 2) / (2*pow(sigma2()(),2)))) ;
}

void SGaussDiff::setparameters()
{
        Kernel::iBind(inScalar,"inScalar", getUuid());
        Kernel::iBind(mu1,"mu1", getUuid());
        Kernel::iBind(sigma1,"sigma1", getUuid());
        Kernel::iBind(mu2,"mu2", getUuid());
        Kernel::iBind(sigma2,"sigma2", getUuid());
}


void MSGaussDiff::compute()
{
        output = (1 / (sigma1()() * sqrt(2*M_PI))) * exp( -( pow( inMatrix()().array() - mu1()(), 2) / (2*pow(sigma1()(),2)))) -  (1 / (sigma2()() * sqrt(2*M_PI))) * exp( -( pow( inMatrix()().array() - mu2()(), 2) / (2*pow(sigma2()(),2)))) ;
}

void MSGaussDiff::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(mu1,"mu1", getUuid());
        Kernel::iBind(sigma1,"sigma1", getUuid());
        Kernel::iBind(mu2,"mu2", getUuid());
        Kernel::iBind(sigma2,"sigma2", getUuid());
}


void MMGaussDiff::compute()
{
        output = (1 / (sigma1()().array() * sqrt(2*M_PI))) * exp( -( (inMatrix()() - mu1()()).array().square()  / (2 * sigma1()().array().square()  ))) -  (1 / (sigma2()().array().square() * sqrt(2*M_PI))) * exp( -( (inMatrix()() - mu2()()).array().square()  / ( 2 * sigma2()().array().square()  ))) ;
}

void MMGaussDiff::setparameters()
{
        Kernel::iBind(inMatrix,"inMatrix", getUuid());
        Kernel::iBind(mu1,"mu1", getUuid());
        Kernel::iBind(sigma1,"sigma1", getUuid());
        Kernel::iBind(mu2,"mu2", getUuid());
        Kernel::iBind(sigma2,"sigma2", getUuid());
}

