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

#ifndef _TF2_LIST_H_
#define _TF2_LIST_H_

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

#include <tf2_ros/transform_listener.h>

/***************************************************************************************************/
/****************************************   TFListener   *******************************************/
/***************************************************************************************************/

class TF2Listener : public FMatrix
{
	private : 

		IString target_frame;
		IString source_frame;
		ISInput time;
		ISInput timeout;

	public : 

		TF2Listener() :  FMatrix(VECTOR) {}
		virtual ~TF2Listener(){}

		virtual void compute();
		virtual void setparameters();
};

/***************************************************************************************************/
/**************************************   TFListenerTrans   ******************************************/
/***************************************************************************************************/

class TF2ListenerTrans : public FMatrix
{
        private :

                IString target_frame;
                IString source_frame;
                ISInput time;
                ISInput timeout;

        public :

                TF2ListenerTrans() :  FMatrix(VECTOR) {}
                virtual ~TF2ListenerTrans(){}

                virtual void compute();
                virtual void setparameters();
};

/***************************************************************************************************/
/*************************************   TFListenerEuler   *****************************************/
/***************************************************************************************************/

class TF2ListenerEuler : public FMatrix
{
	        private :

                IString target_frame;
                IString source_frame;
                ISInput time;
                ISInput timeout;

        public :

                TF2ListenerEuler() :  FMatrix(VECTOR) {}
                virtual ~TF2ListenerEuler(){}

                virtual void compute();
                virtual void setparameters();

};

class TF2ListenerEulerYaw : public FScalar
{
                private :

                IString target_frame;
                IString source_frame;
                ISInput time;
                ISInput timeout;

        public :

                TF2ListenerEulerYaw() :  FScalar() {}
                virtual ~TF2ListenerEulerYaw(){}

                virtual void compute();
                virtual void setparameters();

};


/***************************************************************************************************/
/*************************************   TFListenerQuater   ****************************************/
/***************************************************************************************************/

class TF2ListenerQuater : public FMatrix
{
	private :

                IString target_frame;
                IString source_frame;
                ISInput time;
                ISInput timeout;

        public :

                TF2ListenerQuater() :  FMatrix(VECTOR) {}
                virtual ~TF2ListenerQuater(){}

                virtual void compute();
                virtual void setparameters();
};

#endif // _TF2_LIST_H_
