/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

class CFootBotDiffusion : public CCI_Controller {

public:

   CFootBotDiffusion() :
      m_pcWheels(NULL),
      m_pcProximity(NULL),
      m_cAlpha(10.0f),
      m_fDelta(0.5f),
      m_fNormalSpeed(2.5f),  // Set your normal speed here
      m_fEscapeSpeed(5.0f),  // Set your escape speed here
      m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                              ToRadians(m_cAlpha)) {}

   virtual void Init(TConfigurationNode& t_node) {
      m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity");
      GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
      m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
      GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   }

   virtual void ControlStep() {
      /* Get readings from proximity sensor */
      const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
      /* Sum them together */
      CVector2 cAccumulator;
      for(size_t i = 0; i < tProxReads.size(); ++i) {
         cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
      }
      cAccumulator /= tProxReads.size();
      /* Check if an obstacle is detected */
      if (cAccumulator.Length() > m_fDelta) {
         /* Obstacle detected, set escape speed */
         m_pcWheels->SetLinearVelocity(m_fEscapeSpeed, m_fEscapeSpeed);
      }
      else {
         /* No obstacle detected, set normal speed */
         m_pcWheels->SetLinearVelocity(m_fNormalSpeed, m_fNormalSpeed);
      }
   }

   virtual void Reset() {}

private:

   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_FootBotProximitySensor* m_pcProximity;
   CRadians m_cAlpha;
   Real m_fDelta;
   Real m_fNormalSpeed;
   Real m_fEscapeSpeed;
   CRange<CRadians> m_cGoStraightAngleRange;
};

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")
