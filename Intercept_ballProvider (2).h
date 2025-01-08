/** 
* @author Emilio Leo
*/
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/Intercept_ball.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Configuration/BallSpecification.h"
//Creo Intestazione modulo Intercept_ballProvider
MODULE(Intercept_ballProvider,
{
    ,
    REQUIRES(BallModel),
    REQUIRES(RobotPose),
    REQUIRES(BallSpecification),
    PROVIDES(Intercept_ball),
    
});
//Creo Classe Modulo Intercept_ballProvider
class Intercept_ballProvider : public Intercept_ballProviderBase
{
    private:
    Vector2f position_ball,position_robot,velocity_ball;
    public:
    Intercept_ballProvider();
    void update(Intercept_ball &Intercept_ball);
};
