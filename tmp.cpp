#include "Aria.h"
#include <iostream>
#include <math.h>

/* 
 * Action that drives the robot forward, but stops if obstacles are
 * detected by sonar. 
 */
class ActionGo : public ArAction
{
public:
  // constructor, sets myMaxSpeed and myStopDistance
  ActionGo(double maxSpeed, double stopDistance);
  // destructor. does not need to do anything
  virtual ~ActionGo(void) {};
  // called by the action resolver to obtain this action's requested behavior
  virtual ArActionDesired *fire(ArActionDesired currentDesired);
  // store the robot pointer, and it's ArSonarDevice object, or deactivate this action if there is no sonar.
  virtual void setRobot(ArRobot *robot);
protected:
  // the sonar device object obtained from the robot by setRobot()
  ArRangeDevice *mySonar;


  /* Our current desired action: fire() modifies this object and returns
      to the action resolver a pointer to this object.
      This object is kept as a class member so that it persists after fire()
      returns (otherwise fire() would have to create a new object each invocation,
      but would never be able to delete that object).
  */
  ArActionDesired myDesired;

  double myMaxSpeed;
  double myStopDistance;
};


/* Action that turns the robot away from obstacles detected by the 
 * sonar. */
class ActionTurn : public ArAction
{
public:
  // constructor, sets the turnThreshold, and turnAmount
  ActionTurn(double turnThreshold, double turnAmount, double desX, double desY, double desTheta, double stopDistance);
  // destructor, its just empty, we don't need to do anything
  virtual ~ActionTurn(void) {};
  // fire, this is what the resolver calls to figure out what this action wants
  virtual ArActionDesired *fire(ArActionDesired currentDesired);
  // sets the robot pointer, also gets the sonar device, or deactivates this action if there is no sonar.
  virtual void setRobot(ArRobot *robot);
protected:
  // this is to hold the sonar device form the robot
  ArRangeDevice *mySonar;
  // what the action wants to do; used by the action resover after fire()
  ArActionDesired myDesired;
  // distance at which to start turning
  double myTurnThreshold;
  // amount to turn when turning is needed
  double myTurnAmount;
  // remember which turn direction we requested, to help keep turns smooth
  int myTurning; // -1 == left, 1 == right, 0 == none

  ArRobot* myRobot;
  double desX, desY, desTheta, curX = 0, curY = 0, curTheta = 0, desAngle = 0, stopDistance;
};

/*
  Note the use of constructor chaining with 
  ArAction(actionName). Also note how it uses setNextArgument, which makes it so that 
  other parts of the program could find out what parameters this action has, and possibly modify them.
*/
ActionGo::ActionGo(double maxSpeed, double stopDistance) :
  ArAction("Go")
{
  mySonar = NULL;
  myMaxSpeed = maxSpeed;
  myStopDistance = stopDistance;
  setNextArgument(ArArg("maximum speed", &myMaxSpeed, "Maximum speed to go."));
  setNextArgument(ArArg("stop distance", &myStopDistance, "Distance at which to stop."));
}

/*
  Override ArAction::setRobot() to get the sonar device from the robot, or deactivate this action if it is missing.
  You must also call ArAction::setRobot() to properly store
  the ArRobot pointer in the ArAction base class.
*/
void ActionGo::setRobot(ArRobot *robot)
{
  ArAction::setRobot(robot);
  mySonar = robot->findRangeDevice("sonar");
  if (robot == NULL)
    {
      ArLog::log(ArLog::Terse, "actionExample: ActionGo: Warning: I found no sonar, deactivating.");
      deactivate();
    }
}

/*
  This fire is the whole point of the action.
  currentDesired is the combined desired action from other actions
  previously processed by the action resolver.  In this case, we're
  not interested in that, we will set our desired 
  forward velocity in the myDesired member, and return it.

  Note that myDesired must be a class member, since this method
  will return a pointer to myDesired to the caller. If we had
  declared the desired action as a local variable in this method,
  the pointer we returned would be invalid after this method
  returned.
*/
ArActionDesired *ActionGo::fire(ArActionDesired currentDesired)
{
  double range;
  double speed;

  // reset the actionDesired (must be done), to clear
  // its previous values.
  myDesired.reset();

  // if the sonar is null we can't do anything, so deactivate
  if (mySonar == NULL)
  {
    deactivate();
    return NULL;
  }
  // get the range of the sonar
  range = mySonar->currentReadingPolar(-70, 70) - myRobot->getRobotRadius();
  // if the range is greater than the stop distance, find some speed to go
  if (range > myStopDistance)
  {
    // just an arbitrary speed based on the range
    speed = range * .3;
    // if that speed is greater than our max, cap it
    if (speed > myMaxSpeed)
      speed = myMaxSpeed;
    // now set the velocity
    myDesired.setVel(speed);
  }
  // the range was less than the stop distance, so request stop
  else
  {
    myDesired.setVel(0);
  }
  // return a pointer to the actionDesired to the resolver to make our request
  return &myDesired;
}


/*
  This is the ActionTurn constructor, note the use of constructor chaining 
  with the ArAction. also note how it uses setNextArgument, which makes 
  it so that other things can see what parameters this action has, and 
  set them.  It also initializes the classes variables.
*/
ActionTurn::ActionTurn(double turnThreshold, double turnAmount, double desX, double desY, double desTheta, double stopDistance) :
  ArAction("Turn")
{
  myTurnThreshold = turnThreshold;
  myTurnAmount = turnAmount;
  desX = desX;
  desY = desY;
  desTheta = desTheta;
  stopDistance = stopDistance;
  setNextArgument(ArArg("turn threshold (mm)", &myTurnThreshold, "The number of mm away from obstacle to begin turnning."));
  setNextArgument(ArArg("turn amount (deg)", &myTurnAmount, "The number of degress to turn if turning."));
  myTurning = 0;
}

/*
  Sets the myRobot pointer (all setRobot overloaded functions must do this),
  finds the sonar device from the robot, and if the sonar isn't there, 
  then it deactivates itself.
*/
void ActionTurn::setRobot(ArRobot *robot)
{
  ArAction::setRobot(robot);
  mySonar = robot->findRangeDevice("sonar");
  myRobot = robot;
  if (mySonar == NULL)
  {
    ArLog::log(ArLog::Terse, "actionExample: ActionTurn: Warning: I found no sonar, deactivating.");
    deactivate(); 
  }
}

/*
  This is the guts of the Turn action.
*/
ArActionDesired *ActionTurn::fire(ArActionDesired currentDesired)
{
  printf("%f, %f, %f\n", myRobot->getX(), myRobot->getY(), myRobot->getTh());
  double leftRange, rightRange;
  // reset the actionDesired (must be done)
  myDesired.reset();
  // if the sonar is null we can't do anything, so deactivate
  if (mySonar == NULL)
  {
    deactivate();
    return NULL;
  }
  // Get the left readings and right readings off of the sonar
  leftRange = (mySonar->currentReadingPolar(0, 100) - 
        myRobot->getRobotRadius());
  rightRange = (mySonar->currentReadingPolar(-100, 0) - 
        myRobot->getRobotRadius());
  // if neither left nor right range is within the turn threshold,
  // reset the turning variable and don't turn
std::cout << leftRange << " " << rightRange << std::endl;
  if (leftRange > myTurnThreshold && rightRange > myTurnThreshold)
  {
    myTurning = 0;
    //myDesired.setDeltaHeading(0);

    curX = myRobot->getX();
    curY = myRobot->getY();
    curTheta = myRobot->getTh();
    if(desX - curX < 0) {
      desAngle = atan((desY - curY) / (desX - curX)) + 180;
    }
    else {
      desAngle = atan((desY - curY) / (desX - curX));
    }
std::cout << desAngle << std::endl;
    if(myRobot->getClosestSonarRange(desAngle - 50, desAngle + 50) > stopDistance) {
      std::cout << 1 << std::endl;
      myDesired.setHeading(desAngle);
    }
    else if(myRobot->getClosestSonarRange(desAngle - 45, desAngle) > stopDistance) {
      std::cout << 2 << std::endl;
      myDesired.setHeading(desAngle - 20);
    }
    else if(myRobot->getClosestSonarRange(desAngle, desAngle + 45) > stopDistance) {
      std::cout << 3 << std::endl;
      myDesired.setHeading(desAngle + 20);
    }
/*
    else if(myRobot->getClosestSonarRange(desAngle - 90, desAngle - 45) > stopDistance) {
      myDesired.setHeading(desAngle - 25);
      while (1) {
        if (myRobot->isHeadingDone()) {
          printf("Finished turn\n");
          break;
        }
        ArUtil::sleep(100);
      }
    }
*/    
  }
  // if we're already turning some direction, keep turning that direction
  else if (myTurning)
  {
    myDesired.setDeltaHeading(myTurnAmount * myTurning);
  }
  // if we're not turning already, but need to, and left is closer, turn right
  // and set the turning variable so we turn the same direction for as long as
  // we need to
  else if (leftRange < rightRange)
  {
    myTurning = -1;
    myDesired.setDeltaHeading(myTurnAmount * myTurning);
  }
  // if we're not turning already, but need to, and right is closer, turn left
  // and set the turning variable so we turn the same direction for as long as
  // we need to
  else 
  {
    myTurning = 1;
    myDesired.setDeltaHeading(myTurnAmount * myTurning);
  }
  // return a pointer to the actionDesired, so resolver knows what to do
  return &myDesired;
}



int main(int argc, char** argv)
{
  double x, y, th;
  std::cin >> x >> y >> th;
  x *= 1000;
  y *= 1000;
  th = th * 180 / 3.14;
  Aria::init();

  ArSimpleConnector conn(&argc, argv);
  ArRobot robot;
  ArSonarDevice sonar;

  // Create instances of the actions defined above, plus ArActionStallRecover, 
  // a predefined action from Aria.
  ActionGo go(500, 350);
  ActionTurn turn(400, 10, x, y, th, 350);
  ArActionStallRecover recover;

    
  // Parse all command-line arguments
  if(!Aria::parseArgs())
  {
    Aria::logOptions();
    return 1;
  }
  
  // Connect to the robot
  if(!conn.connectRobot(&robot))
  {
    ArLog::log(ArLog::Terse, "actionExample: Could not connect to robot! Exiting.");
    return 2;
  }

  // Add the range device to the robot. You should add all the range 
  // devices and such before you add actions
  robot.addRangeDevice(&sonar);

 
  // Add our actions in order. The second argument is the priority, 
  // with higher priority actions going first, and possibly pre-empting lower
  // priority actions.
  robot.addAction(&recover, 100);
  robot.addAction(&go, 50);
  robot.addAction(&turn, 49);

  // Enable the motors, disable amigobot sounds
  robot.enableMotors();

  // Run the robot processing cycle.
  // 'true' means to return if it loses connection,
  // after which we exit the program.
  robot.run(true);
  
  Aria::shutdown();
  return 0;
}
