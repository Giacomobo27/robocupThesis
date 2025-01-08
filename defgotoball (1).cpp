/**
* @file defgotoball.cpp
*
* @author jin giacomo
*/
#include <cmath>
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h" //skill so dichiarate tutte qua
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
//#include "Representations/BehaviorControl/Libraries/LibDefender.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Intercept_ball.h"
#include <iostream>
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/FallDownState.h"
//#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"



#include <Eigen/Geometry>

using Line2 = Eigen::Hyperplane<float,2>;

CARD(defgotoball,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick), // Sono le skills che chiama 
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(LookLeftAndRight),
  CALLS(TurnAngle),
  CALLS(Say),
  CALLS(WalkToPoint),
  CALLS(LookAtBall),
  CALLS(GoToBallHeadControl),
  CALLS(TurnToPoint),
  CALLS(LookAtGlobalBall),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions), //rappresentazioni richieste
  REQUIRES(RobotPose),
  //REQUIRES(LibDefender),
  REQUIRES(LibMisc),
  REQUIRES(Intercept_ball),
  REQUIRES(BallModel),
  DEFINES_PARAMETERS(
  {,
    (float)(1.0f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(3000) ballNotSeenTimeout,
  }),

});


class defgotoball : public defgotoballBase



{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

//int tempo1;

 option

  {
  ///  theActivitySkill(BehaviorStatus::Defender);
    theActivitySkill(BehaviorStatus::Striker);

    initial_state(start){
     
        transition{
           
          if(state_time>initialWaitTime)
            goto howtoapproach;

        }
        action // fa action finche  non transisce a un altro stato
      {
      //  theSaySkill("ok",1.0f);
        theLookForwardSkill();
        theStandSkill();

      }
    }
state(howtoapproach){
    transition{
     // theSaySkill("ok",1.0f);
    if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
      goto seeBall;
 
      Vector2f target = theIntercept_ball.position_intercept;
    
       Angle angolo(theIntercept_ball.angle_to_position_intercept);
       
       Vector2f position_ball = theBallModel.estimate.position;
       //Vector2f target =position_ball;
       Angle res(atan2f(position_ball.y(), position_ball.x()));
       //cout << "Angle" << res << endl;
       Angle zero(0.0f);
        Angle zerom(-0.2f);
      Angle zerop(0.2f);

      if(!theFieldBall.ballWasSeen(100) && position_ball.norm()<230)
        goto still;
      
      //if(res>zerom && res<zerop && position_ball.norm()<90) goto KickBall;

         float distanza=target.norm();


         //if(angolo>zerom && angolo<zerop && distanza>0) goto walking;
         
         if(position_ball.norm()>0 && distanza>0) goto walking;
        
        // else if(position_ball.norm()>=600 && distanza>0) goto rotating;
  }
    action
      {
        
        theLookAtBallSkill();

        Vector2f position_ball = theBallModel.estimate.position;
       
        Angle res(atan2f(position_ball.y(), position_ball.x()));
       
        theTurnAngleSkill(res, 0_deg);

      }
}
state(walking){
    transition{
       

       Vector2f target = theIntercept_ball.position_intercept;
       if(target.norm()==0) goto howtoapproach; 
      // if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
      //goto seeBall;
    }
    action
      {
        Vector2f target = theIntercept_ball.position_intercept;
      
        theLookAtBallSkill();
        Angle angolo(theIntercept_ball.angle_to_position_intercept);
        
     // Pose2f targetpos(angolo,target);
        Pose2f targetpos(target.x(),target.y());
        theWalkToPointSkill(targetpos,1.0f);

      }
}

state(rotating){
  transition{
     Angle angolo(theIntercept_ball.angle_to_position_intercept);
      Angle zerom(-0.2f);
      Angle zerop(0.2f);
     if(angolo>zerom && angolo<zerop) goto walking;
  }
  action{
     Angle angolo(theIntercept_ball.angle_to_position_intercept);//da capire se devo fa +90_degree o no
    theLookAtBallSkill();
    Angle res;
    Angle novanta(1.57f);
    Angle zero(0.f);
    if(angolo>zero) res= angolo - novanta;
    else res= angolo + novanta;
    theTurnAngleSkill(res, 0_deg);

  }
}

state(still){
  transition{
     if(state_time>2000)
      goto howtoapproach;
   // if(theTurnAngleSkill.isDone() && !theFieldBall.ballWasSeen(ballNotSeenTimeout)) goto seeBall;
  }
  action{
      
     // Angle tutto(2.0f);
      //  theLookLeftAndRightSkill(/* startLeft: */ true, /* maxPan: */ 50_deg, /* tilt: */ 23_deg, /* speed: */ 30_deg);
    //  theTurnAngleSkill(tutto, 0_deg);
     theLookForwardSkill();
     theStandSkill();


  }
}


state(KickBall){
  transition{
    theSaySkill("inter",1.0f);
    if(!theFieldBall.ballWasSeen(ballNotSeenTimeout)) goto seeBall;
    if(state_time>3000) goto howtoapproach;
  }
  action{
      theLookForwardSkill();
       theStandSkill();

        // theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.8f, 0.8f));

  }
}
state(seeBall){
   transition{
      if(theFieldBall.ballWasSeen())
      goto howtoapproach;
     if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
      goto searchBall;
   }
    action{
      
       theLookLeftAndRightSkill(/* startLeft: */ true, /* maxPan: */ 30_deg, /* tilt: */ 10_deg, /* speed: */ 15_deg);
       theStandSkill();
    }
   }

state(searchBall){
  transition{
     if(theFieldBall.ballWasSeen())
      goto howtoapproach;
    }
  
  action{
    theLookLeftAndRightSkill(/* startLeft: */ true, /* maxPan: */ 30_deg, /* tilt: */ 10_deg, /* speed: */ 15_deg);
        
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.8f, 0.8f));
  }
}
  }
};
MAKE_CARD(defgotoball);
