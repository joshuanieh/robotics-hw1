#include "Aria.h"
/*#include <stdio.h>
#include <stdlib.h>
#include<iostream>
using namespace std;

*/
ArRobot robot;
void up()
{
  robot.lock();
  robot.setVel(1000);
  robot.unlock();
}

void back()
{
  robot.lock();
  robot.setVel(-1000);
  robot.unlock();
}

void left()
{
  robot.lock();
  robot.setRotVel(20);
  robot.unlock();
}

void right()
{
  robot.lock();
  robot.setRotVel(-20);
  robot.unlock();
}

int main(int argc, char **argv)
{
  ArSonarDevice sonar;
  robot.addRangeDevice(&sonar);
  Aria::init();	
  ArSimpleConnector connector(&argc,argv);
  if (!connector.connectRobot(&robot)){
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    Aria::exit(1);
  }
  robot.comInt(ArCommands::ENABLE, 1);
  robot.runAsync(true);

  // Used to perform actions when keyboard keys are pressed
  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);

  // ArRobot contains an exit action for the Escape key. It also 
  // stores a pointer to the keyhandler so that other parts of the program can
  // use the same keyhandler.
  robot.attachKeyHandler(&keyHandler);
  printf("You may press escape to exit\n");

  // Start of controling

  ArGlobalFunctor upCB(&up);
  ArGlobalFunctor backCB(&back);
  ArGlobalFunctor leftCB(&left);
  ArGlobalFunctor rightCB(&right);
  keyHandler.addKeyHandler(ArKeyHandler::UP, &upCB);
  keyHandler.addKeyHandler(ArKeyHandler::DOWN, &backCB);
  keyHandler.addKeyHandler(ArKeyHandler::LEFT, &leftCB);
  keyHandler.addKeyHandler(ArKeyHandler::RIGHT, &rightCB);
  /*keyHandler.addKeyHandler(258, new ArFunctor1C<ArRobot, double>(robot, &ArRobot::setRotVel, 20));
  keyHandler.addKeyHandler(259, new ArFunctor1C<ArRobot, double>(robot, &ArRobot::setRotVel, -20));
  keyHandler.addKeyHandler(256, new ArFunctor1C<ArRobot, double>(robot, &ArRobot::setVel, 1000));
  keyHandler.addKeyHandler(257, new ArFunctor1C<ArRobot, double>(robot, &ArRobot::setVel, -1000));
  */
  while(true){
   
    robot.lock();
    robot.setRotVel(0);
    robot.setVel(0);
    robot.unlock();
  
    printf("%f %f %f\n", robot.getX(), robot.getY(), robot.getTh());
    ArUtil::sleep(300);
  }
  // End of controling


  Aria::shutdown();
  Aria::exit(0);
}
