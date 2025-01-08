/**
 * @author Emilio Leo
 */
#include "Intercept_ballProvider.h"
#include "Representations/BehaviorControl/Intercept_ball.h"
#include "Tools/Math/Eigen.h"
#include <iostream>
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Pose3f.h"

#define M_PIf 3.14159265358979323846f
using namespace std;

// Istanzio il modulo Intercept_ballProvider
MAKE_MODULE(Intercept_ballProvider, behaviorControl);
// istanzio l'oggetto Modulo Intercept_ballProvider
Intercept_ballProvider::Intercept_ballProvider(){
  
};
void Intercept_ballProvider::update(Intercept_ball &intercept_ball)
{
  // Costruzione grafica del mondo in cui modellare l'intercettazione del pallone
  // mondo costruito di tipo campo
  DECLARE_DEBUG_DRAWING("robot", "drawingOnField");
  //ROBOT("robot",);
  DECLARE_DEBUG_DRAWING3D("module:Intercept_ballProvider","field");

  //SPHERE3D("module:Intercept_ballProvider",1000,1000,0,200,ColorRGBA::orange);
  /*Pose3f p_right_1(-800,1000,0);
  Pose3f p_left_1(-800,800,0);

  FOOT3D("module:Intercept_ballProvider",p_right_1,false,ColorRGBA::yellow);
  FOOT3D("module:Intercept_ballProvider",p_left_1,true,ColorRGBA::yellow);

  Pose3f p_right_2(-1000,2000,0);
  Pose3f p_left_2(-1000,1800,0);

  FOOT3D("module:Intercept_ballProvider",p_right_2,false,ColorRGBA::blue);
  FOOT3D("module:Intercept_ballProvider",p_left_2,true,ColorRGBA::blue);*/

  Vector3f top_left(-2000,2500,0);
  Vector3f top_right(800,2500,0);
  Vector3f bottom_left(-2000,-400,0);
  Vector3f bottom_right(800,-400,0);

  QUAD3D2("module:Intercept_ballProvider",top_left,top_right,bottom_right,bottom_left,150,ColorRGBA::blue);
  
  //TRANSLATE3D("module:Intercept_ballProvider",20.5,20.5,0);

  /*POINT3D("module:Intercept_ballProvider",100,-200,40,20,ColorRGBA::white);
  POINT3D("module:Intercept_ballProvider",100,-1000,40,20,ColorRGBA::red);*/
  /*
     * A macro that sends a circle
     * @param id A drawing id
     * @param center_x The x coordinate of the center of the circle
     * @param center_y The y coordinate of the center of the circle
     * @param radius The radius of the circle
     * @param penWidth The width of the arc of the circle
     * @param penStyle The pen style of the arc of the circle (Drawings::PenStyle)
     * @param penColor The color of the arc of the circle
     * @param brushStyle The brush style of the circle
     * @param brushColor The brush color of the circle

  */
  // CIRCLE(id, center_x, center_y, radius, penWidth, penStyle, penColor, brushStyle, brushColor)
  //CIRCLE("module:Intercept_ballProvider", 0.0, 0.0, 110.0, 10, Drawings::solidPen, ColorRGBA::magenta, Drawings::solidBrush, ColorRGBA(0, 0, 255, 123));
  /*A macro that sends a RobotPose (Pose2f)
   * @param id A drawing id
   * @param p The desired Pose2f
   * @param dirVec The direction vector of the body
   * @param dirHeadVec The direction vector of the head
   * @param alphaRobot The alpha of the robot to draw
   * @param colorBody The color of the robot
   * @param colorDirVec The color of the direction vector of the body (Set alpha channel to 0, to disable this drawing)
   * @param colorDirHeadVec The color of the direction vector of the head (Set alpha channel to 0, to disable this drawing)
   */
  Pose2f P(222, 111);
  Vector2f V(2, 2);
  Vector2f D_H(2, 2);
  ROBOT("robot", P, V, D_H, 55.f, ColorRGBA::magenta, ColorRGBA::white, ColorRGBA::violet);
  /*
   A macro that sends an rectangle
  * @param x1,y1,x2,y2 The coordinates of 2 opposite corners
  * @param id A drawing id
  * @param penWidth The line width of the rectangle
    * @param penStyle The line style, e.g. dotted
    * @param penColor The color of the quadrangle
  */
  //RECTANGLE("module:Intercept_ballProvider", 0, 0, 80, 80, 20, Drawings::solidPen, ColorRGBA::yellow);

  // costruzione del campo

  // Aggiorno posizione palla,posizione robot,velocità palla all'istante corrente

  Intercept_ballProvider::position_ball = theBallModel.estimate.position;
  Intercept_ballProvider::position_robot = theRobotPose.translation;
  Intercept_ballProvider::velocity_ball = theBallModel.estimate.velocity;
  
  cout << "position_ball" << position_ball << endl;
  //cout << "position_robot" << position_robot;

  bool velocity_cost = true;
  //cout << "posizione palla " << position_ball;
  // cout << "velocity_ball " << velocity_ball << endl;
  Vector2f v_r;

  if ((velocity_ball.x() > 0 && velocity_ball.y() < 0) || (velocity_ball.x() > 0 && velocity_ball.y() >= 0))
  {
    v_r[0] = 100;
    v_r[1] = 100;
    // cout<< "caso1" << endl;
  }
  else if ((velocity_ball.x() < 0 && velocity_ball.y() < 0) || (velocity_ball.x() < 0 && velocity_ball.y() >= 0))
  {
    v_r[0] = -100;
    v_r[1] = 100;
    // cout << "caso2" << endl;
  }
  else if ((velocity_ball.x() == 0 && velocity_ball.y() < 0) || (velocity_ball.x() == 0 && velocity_ball.y() > 0))
  {
    v_r[0] = 0;
    v_r[1] = 100;
    // cout << "caso3" << endl;
  }

  float norm_velocity_ball = velocity_ball.norm();
  float norm_velocity_robot = v_r.norm();

  

  if (velocity_cost)
  {
    // Task1: Robot e palla si muovono a  velocità costante (moto rettilineo uniforme).
    //
    if (norm_velocity_ball != 0 && ((((v_r.x()>0 && v_r.y() >0) || (v_r.x()<0 && v_r.y()>0) || (velocity_ball.x() == 0 && velocity_ball.y() > 0)) && norm_velocity_ball < norm_velocity_robot) || (velocity_ball.x() == 0 && velocity_ball.y() < 0)))
    {
      // calcolo time per intercettare il pallone

      float diff_velocity = (v_r - velocity_ball).norm();
      // aggiusto time in ms
      intercept_ball.time_to_ball = (position_ball.norm() / diff_velocity);
      intercept_ball.robot_speed = v_r;

      cout << "time_to_ball " << intercept_ball.time_to_ball << endl;

      // calcolo i metri da percorrere affinchè avvenga l'urto 
     
     
      //float numerator =  ((intercept_ball.robot_speed.x() * position_ball.x()) + (intercept_ball.robot_speed.y() * position_ball.y()));
      float position_to_arrive_ball = norm_velocity_robot * intercept_ball.time_to_ball;

      cout << "position_to_arrive_ball: " << position_to_arrive_ball << endl;

      // calcolo angolazione del vettore spostamento
      if (intercept_ball.robot_speed.x() != 0)
        intercept_ball.angle_to_position_intercept = atan2f(intercept_ball.robot_speed.y(), intercept_ball.robot_speed.x());
      else
        intercept_ball.angle_to_position_intercept = M_PIf / 2;

      //cout << "angolazione per arrivare al pallone" << intercept_ball.angle_to_position_intercept << endl;

      // cout << "Posizione per arrivare al pallone: " << position_to_arrive_ball << endl;

      // aggiorno posizione di intercettazione costruendo un vettore in coordinate (x,y)
      float c_x = position_to_arrive_ball * cosf(intercept_ball.angle_to_position_intercept);
      float c_y = position_to_arrive_ball * sinf(intercept_ball.angle_to_position_intercept);
      Vector2f vec_to_arrive_ball(c_x, c_y);

      //cout << "vettore per arrivare al pallone" << vec_to_arrive_ball << endl;

      intercept_ball.position_intercept = vec_to_arrive_ball;
    }
    else
    {
      //cout << "I corpi non si incontreranno con successo oppure sono in stato di quiete " << endl;
      intercept_ball.robot_speed = Vector2f::Zero();
      intercept_ball.time_to_ball = -1;
      intercept_ball.position_intercept = Vector2f::Zero();
      intercept_ball.angle_to_position_intercept = 0;
    }
  }
  else
  {
    // Task 2:Palla decellera costantemente rispetto al robot

    // decellerazione costante
    float deceleration = theBallSpecification.friction * (1000);
    //palla decellera
    float velocity_ball_deceleration = norm_velocity_ball + deceleration * (1);
    //condizioni di urto
    if(norm_velocity_ball!=0 && norm_velocity_ball!=norm_velocity_robot && velocity_ball_deceleration < norm_velocity_robot)
    {
      // calcolo tempo di intercetto

      // calcolo: -(VR -VP)
      float diff_v = (v_r - velocity_ball).norm();
      // calcolo: (SP-SR)
      float diff_position = (position_ball).norm();

      // calcolo delta
      float delta = powf(diff_v, 2) - (deceleration * 2 * diff_position);
      float time_1 = 0, time_2 = 0;

      if (delta > 0)
      {
        time_1 = ((diff_v + sqrtf(delta)) / (deceleration));
        time_2 = ((diff_v - sqrtf(delta)) / (deceleration));
        //cout << "time1: " << time_1 << endl;
        //cout << "time2: " << time_2 << endl;
      }
      else if (delta == 0)
      {
        time_1 = (diff_v / deceleration);
        //cout << "time1: " << time_1 << endl;
      }
      else
      {
        //cout << "non esistono soluzioni" << endl;
        intercept_ball.time_to_ball=-1;
      }

      //si valuta se si incontreranno i corpi
      if (time_1 > 0 && time_2 < 0)
        intercept_ball.time_to_ball = time_1;
      else if (time_1 < 0 && time_2 > 0)
        intercept_ball.time_to_ball = time_2;
      else if (time_1 <0 && time_2 < 0){
        goto L;
      }else
      {
        if(time_1>=time_2)
          //scelgo l'istante di tempo più breve per fornire l'opportunità al robot di intercettare il pallone quando è più vicino a lui
          intercept_ball.time_to_ball=time_2;
        else
          intercept_ball.time_to_ball=time_1;
      }

      // calcolo posizione dell'urto del robot (assoluta)
      float finalposition = norm_velocity_robot * intercept_ball.time_to_ball;
      //calcolo posizione dell'urto

      
      cout << "time_to_ball" << intercept_ball.time_to_ball << endl;
      cout << "final_position: " << finalposition << endl;

      //calcolo angolazione 
      if (v_r.x() != 0)
        intercept_ball.angle_to_position_intercept = atan2f(v_r.y(), v_r.x());
      else
        intercept_ball.angle_to_position_intercept = M_PIf / 2;

      //cout << "angolazione per arrivare al pallone: " << intercept_ball.angle_to_position_intercept << endl;

      // aggiorno posizione di intercettazione costruendo un vettore in coordinate (x,y)
      
      float c_x = finalposition * cosf(intercept_ball.angle_to_position_intercept);
      float c_y = finalposition * sinf(intercept_ball.angle_to_position_intercept);

      Vector2f vec_to_arrive_ball(c_x, c_y);

      //cout << "vettore per arrivare al pallone: " << vec_to_arrive_ball << endl;

      intercept_ball.position_intercept = vec_to_arrive_ball;
      
    }
    else{
      L:
        //cout << "i corpi non si incontreranno con successo: " << endl;
        intercept_ball.robot_speed = Vector2f::Zero();
        intercept_ball.time_to_ball = -1;
        intercept_ball.position_intercept = Vector2f::Zero();
        intercept_ball.angle_to_position_intercept=0;
    }
  }
}

//---------------------------------------------------
