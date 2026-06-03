/******************************************************************************
*                              BeamAdapter plugin                             *
*                  (c) 2006 Inria, University of Lille, CNRS                  *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: see Authors.md                                                     *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <BeamAdapter/component/model/RodSteerableSection.h>
#include <BeamAdapter/component/model/BaseRodSectionMaterial.inl>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/BaseComponent.h>

namespace beamadapter
{

template <class DataTypes>
RodSteerableSection<DataTypes>::RodSteerableSection()
    : BaseRodSectionMaterial<DataTypes>()
    , d_spireDiameter(initData(&d_spireDiameter, (Real)0.1, "spireDiameter", "diameter of the spire"))
    , d_spireHeight(initData(&d_spireHeight, (Real)0.01, "spireHeight", "height between each spire"))
    , d_activeBending(initData(&d_activeBending, (bool)false, "activeBending", "Boolean activating the bending of the steerable catheter"))
    , d_deactiveBending(initData(&d_deactiveBending, (bool)false, "deactiveBending", "Boolean deactivating the bending of the steerable catheter"))
    , d_angleMax(initData(&d_angleMax, (Real)180.0, "angleMax", "Maximum angle that the catheter can reach \n (in degree [0-360])"))
    , d_flatAngle(initData(&d_flatAngle, (Real)1.0, "flatAngle", "Angle below which we consider the catheter as flat/n (Can't be zero)"))
    , d_bendingRate(initData(&d_bendingRate, (unsigned int)10, "bendingRate", "Nb of step needed to reach the maximum bending angle /n (the lower, the faster)"))

{
    this->f_listening.setValue(true);
}


template <class DataTypes>
bool RodSteerableSection<DataTypes>::initSection()
{
    const auto length = this->d_length.getValue();
    if (length <= Real(0.0))
    {
        msg_error() << "Length is 0 (or negative), check if d_length has been given or well computed.";
        return false;
    }

    return true;
}


template <class DataTypes>
void RodSteerableSection<DataTypes>::getRestTransformOnX(Transform& global_H_local, const Real x_used, const Real x_start)
{
    Real projetedLength = d_spireDiameter.getValue() * M_PI;
    Real lengthSpire = sqrt(d_spireHeight.getValue() * d_spireHeight.getValue() + projetedLength * projetedLength);
    // angle in the z direction
    Real phi = atan(d_spireHeight.getValue() / projetedLength);

    Quat Qphi;
    Qphi.axisToQuat(type::Vec3(0, 0, 1), phi);

    // spire angle (if theta=2*PI, there is a complete spire between startx and x)
    Real lengthCurve = x_used - x_start;
    Real numSpire = lengthCurve / lengthSpire;
    Real theta = 2 * M_PI * numSpire;

    // computation of the Quat
    Quat Qtheta;
    Qtheta.axisToQuat(type::Vec3(0, 1, 0), theta);
    Quat newSpireQuat = Qtheta * Qphi;

    // computation of the position
    Real radius = d_spireDiameter.getValue() / 2.0;
    type::Vec3 PosEndCurve(radius * sin(theta), numSpire * d_spireHeight.getValue(), radius * (cos(theta) - 1));
    type::Vec3 SpirePos = PosEndCurve + type::Vec3(x_start, 0, 0);

    global_H_local.set(SpirePos, newSpireQuat);    
}

template <class DataTypes>
void RodSteerableSection<DataTypes>::handleEvent(core::objectmodel::Event* event)
{
    if (sofa::core::objectmodel::KeypressedEvent::checkEventType(event))
    {
        sofa::core::objectmodel::KeypressedEvent* ke = static_cast<sofa::core::objectmodel::KeypressedEvent*>(event);
        //msg_info() << "HapticEmulator handleEvent gets character '" << ke->getKey() << "'. ";

        if (ke->getKey() == '8')
        {
			std::cout << "RodSteerableSection<DataTypes>::handleEvent + key pressed" << std::endl;
			auto val = d_spireDiameter.getValue() + 1.0f;
            //m_currentAngleRadian += m_incrementalAngleRadian;
            d_spireDiameter.setValue(val);
        }
        else if (ke->getKey() == '9')
        {
			std::cout << "RodSteerableSection<DataTypes>::handleEvent - key pressed" << std::endl;
            auto val = d_spireDiameter.getValue() - 1.0f;
            d_spireDiameter.setValue(val);
        }
        
    }
}

} // namespace beamadapter
