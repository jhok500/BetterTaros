#include "epuck_id_qtuser_functions.h"

#include <controllers/epuck_omega_algorithm/epuck_omega_algorithm.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>

/****************************************/
/****************************************/

CIDQTUserFunctions::CIDQTUserFunctions() {
   RegisterUserFunction<CIDQTUserFunctions,CEPuckEntity>(&CIDQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CIDQTUserFunctions::Draw(CEPuckEntity& c_entity) {

   CEPuckOmegaAlgorithm& controller = dynamic_cast<CEPuckOmegaAlgorithm&>(c_entity.GetControllableEntity().GetController());

   if(controller.draw_id)
   {
     /* Disable lighting, so it does not interfere with the chosen text color */
     glDisable(GL_LIGHTING);
     /* Disable face culling to be sure the text is visible from anywhere */
     glDisable(GL_CULL_FACE);
     /* Set the text color */
     CColor cColor(CColor::BLACK);
     glColor3ub(cColor.GetRed(), cColor.GetGreen(), cColor.GetBlue());
     /* The position of the text is expressed wrt the reference point of the e-puck
      * For a e-puck, the reference point is the center of its base.
      * See also the description in
      * $ argos3 -q e-puck
      */
     GetQTOpenGLWidget().renderText(0.0, 0.0, 0.1,             // position
                                  c_entity.GetId().c_str()); // text
     /* Restore face culling */
     glEnable(GL_CULL_FACE);
     /* Restore lighting */
     glEnable(GL_LIGHTING);
   }

   // Draw RAB range
   // if(controller.draw_sensor_range == "all" || controller.draw_sensor_range == c_entity.GetId())
   if(controller.draw_sensor_range == "all" || (controller.draw_sensor_range.find(c_entity.GetId()) != std::string::npos))
   {
     DrawCircle(CVector3(0, 0, 0.001),
                CQuaternion(),
                c_entity.GetRABEquippedEntity().GetRange(),
                CColor::BLACK,
                false,
                100);
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions, "epuck_id_qtuser_functions")
