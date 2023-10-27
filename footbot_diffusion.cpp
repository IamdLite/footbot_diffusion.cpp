/* Include the controller definition */
#include "footbot_vfh.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CFootBotVFH::CFootBotVFH() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotVFH::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
   /*
    * Parse the configuration file
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CFootBotVFH::ControlStep() {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Calculate the VFH histogram */
   std::vector<Real> vecHistogram(tProxReads.size(), 0.0f);
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      if (tProxReads[i].Value > m_fDelta) {
         CRadians cAngle = tProxReads[i].Angle;
         size_t unBin = static_cast<size_t>((cAngle - m_cGoStraightAngleRange.GetMin()).GetValue() /
                                             (m_cGoStraightAngleRange.GetSpan().GetValue() / tProxReads.size()));
         vecHistogram[unBin] += tProxReads[i].Value;
      }
   }
   /* Find the direction with the least obstacle density */
   size_t unBestBin = 0;
   Real fMinDensity = vecHistogram[0];
   for(size_t i = 1; i < vecHistogram.size(); ++i) {
      if (vecHistogram[i] < fMinDensity) {
         fMinDensity = vecHistogram[i];
         unBestBin = i;
      }
   }
   /* Calculate the direction to move */
   CRadians cNewDirection = m_cGoStraightAngleRange.GetMin() +
                           (CRadians(unBestBin) * (m_cGoStraightAngleRange.GetSpan() / tProxReads.size()));
   /* Set the wheel speeds based on the new direction */
   m_pcWheels->SetLinearVelocity(m_fWheelVelocity, cNewDirection);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotVFH, "footbot_vfh_controller")
