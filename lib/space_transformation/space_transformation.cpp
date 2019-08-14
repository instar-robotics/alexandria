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

#include "space_transformation.h"

//using namespace Eigen;

REGISTER_FUNCTION(EulerToQuaternion);
REGISTER_FUNCTION(QuaternionToEuler);

/***************************************************************************************************/
/************************************   EulerToQuaternion   ****************************************/
/***************************************************************************************************/

void EulerToQuaternion::compute()
{
	MATRIX tmp = inEuler()();
	auto m_in = getMapVect( tmp );

	Quaternions q;
	q = AngleAxiss(m_in[0], Vector3s::UnitX()) * AngleAxiss(m_in[1], Vector3s::UnitY()) * AngleAxiss(m_in[2], Vector3s::UnitZ());

	output = q.coeffs();
}

void EulerToQuaternion::setparameters()
{
	if(output.size() != 4) throw std::invalid_argument("EulerToQuaternion : Output must be a 4D Vector.");

	inEuleur.setCheckSize(false);
	Kernel::iBind(inEuler,"inEuler", getUuid());
}

void EulerToQuaternion::prerun()
{
	if( inEuler().iSize() != 3 || !inEuler().isVect())  throw std::invalid_argument("EulerToQuaternion : inEuler must be a 3D Vector.");
}

/***************************************************************************************************/
/************************************   QuaternionToEuler   ****************************************/
/***************************************************************************************************/

void QuaternionToEuler::compute()
{
	MATRIX tmp = inQuater()();
	Quaternions q = Map<Quaternions>( tmp.data()) ;

	output = q.toRotationMatrix().eulerAngles(0, 1, 2);
}

void QuaternionToEuler::setparameters()
{
	if(output.size() != 3) throw std::invalid_argument("QuaternionToEuler : Output must be a 3D Vector.");

	inQuater.setCheckSize(false);
	Kernel::iBind(inQuater,"inQuater", getUuid());
}

void QuaternionToEuler::prerun()
{
	if( inQuater().iSize() != 3 || !inQuater().isVect())  throw std::invalid_argument("QuaternionToEuler : inQuater must be a 3D Vector.");
}

