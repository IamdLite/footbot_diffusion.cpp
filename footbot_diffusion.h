#ifndef FOOTBOT_VFH_H
#define FOOTBOT_VFH_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/core/utility/math/range.h>

using namespace argos;

class CFootBotVFH : public CCI_Controller {
public:
   CFootBotVFH();

   virtual ~CFootBotVFH() {}

   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();

   virtual void Reset() {}

   virtual void Destroy() {}

private:
   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_FootBotProximitySensor* m_pcProximity;

   CDegrees m_cAlpha;
   Real m_fDelta;
   Real m_fWheelVelocity;
   CRange<CRadians> m_cGoStraightAngleRange;
};

#endif
