/**
* @author Emilio Leo
*/
#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"
STREAMABLE(Intercept_ball,{,

    (Vector2f) (Vector2f::Zero()) robot_speed,//vettore velocità del robot
    
    (Vector2f) (Vector2f::Zero()) position_intercept,//vettore spostamento per arrivare all'intercetto del pallone

    (float) time_to_ball,//tempo impiegato per raggiungere il pallone

    (float) angle_to_position_intercept, //angolo di intercettazione 
});
