#include "Aria.h"
/*#include <stdio.h>
#include <stdlib.h>
*/
#include <iostream>
#include <math.h>
/*using namespace std;

*/
ArRobot robot;
int safeDistance = 500;
void up()
{ 
  robot.lock();
  if(robot.getClosestSonarRange(-40, 40) < safeDistance) {
    robot.setVel(0);
    printf("Too closed\n");
  }
  else{
    robot.setVel(300);
  }
  robot.unlock();
}

void back()
{
  robot.lock();
  if(robot.getClosestSonarRange(140, -140) < safeDistance) {
    robot.setVel(0);
    printf("Too closed\n");
  }
  else{
    robot.setVel(-300);
  }
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
  double x, y, th;
  std::cin >> x >> y >> th;
  std::cout << x << y << th;
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
  // Used to access and process sonar range data
  ArSonarDevice sonarDev;

  // Attach sonarDev to the robot so it gets data from it.
  robot.addRangeDevice(&sonarDev);

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
  ArUtil::sleep(1000);
  printf("number of sonar = %d\n", robot.getNumSonar());
  robot.moveTo(ArPose(5090,3580,3093.97));
  x *= 1000;
  y *= 1000;
  th = th / 3.14 * 180;
  double tangent, theta, distance;
  tangent = (y - robot.getY()) / (x - robot.getX());
  theta = atan(tangent) / 3.14 * 180;
  if(x - robot.getX() < 0) theta += 180;
  distance = sqrt(pow(y - robot.getY(), 2) + pow(x - robot.getX(), 2));
  //std::cout << theta << " " << distance << std::endl;
  printf("starts at %f %f %f\n", robot.getX(), robot.getY(), robot.getTh());
  robot.lock();
  robot.setHeading(theta);
  robot.unlock();
  while (1) {
    robot.lock();
    if (robot.isHeadingDone()) {
      printf("Finished turn\n");
      robot.unlock();
      break;
    }
    robot.unlock();
    ArUtil::sleep(100);
  }
  
  robot.lock(); 
  robot.move(distance);
  robot.unlock();
  while (1) {
    robot.lock();
    if (robot.isMoveDone()) {
      printf("Finished distance\n");
      robot.unlock();
      break;
    }
    robot.unlock();
    ArUtil::sleep(50);
  }
  
  robot.lock();
  robot.setHeading(th);
  robot.unlock();
  while (1) {
    robot.lock();
    if (robot.isHeadingDone()) {
      printf("Finished turn\n");
      robot.unlock();
      break;
    }
    robot.unlock();
    ArUtil::sleep(100);
  }


  printf("ends at %f %f %f\n", robot.getX(), robot.getY(), robot.getTh());
  /*while(true){
    robot.lock();
    robot.setRotVel(10);
    robot.setVel(0);
    robot.unlock();
    }
    printf("%f %f %f\n", robot.getX(), robot.getY(), robot.getTh());
    ArUtil::sleep(300);
  }*/
  // End of controling


  Aria::shutdown();
  Aria::exit(0);
}
