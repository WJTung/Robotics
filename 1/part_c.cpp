#include "Aria.h"
#include <stdio.h>
#define max_Vel 600.0
#define max_RotVel 30.0
#define delta_Vel 30.0
#define delta_RotVel 5.0
#define safety_distance 2000
#define Accel 150.0
#define Decel 150.0
void up_CB(ArRobot *robot, ArRangeDevice *sonar)
{
    puts("UP pressed");
    double now_Vel = robot->getVel();
    double distance = sonar->currentReadingPolar(-45.0, 45.0);
    if(distance < safety_distance && now_Vel > 0)
    {
        printf("Warning! Obstacle in %f m forward.\n", distance);
        double new_Vel;
        if(now_Vel > Decel)
            new_Vel = now_Vel - Decel;
        else
            new_Vel = 0.0;
        printf("Change velocity from %f to %f to avoid collision.\n", now_Vel, new_Vel);
        robot->setVel(new_Vel);
    }
    else
    {
        if(now_Vel + delta_Vel <= max_Vel)
        {
            printf("Change velocity from %f to %f\n", now_Vel, now_Vel + delta_Vel);  
            robot->setVel(now_Vel + delta_Vel);
        }
        else
        {
            printf("Velocity = %f, achieve maximum velocity.\n", max_Vel);  
            robot->setVel(max_Vel);
        }
    }
    putchar('\n');
}
void down_CB(ArRobot *robot, ArRangeDevice *sonar)
{
    puts("DOWN pressed");
    double now_Vel = robot->getVel();
    double distance = sonar->currentReadingPolar(135.0, -135.0);
    if(distance < safety_distance && now_Vel < 0)
    {
        printf("Warning! Obstacle in %f m backward.\n", distance);
        double new_Vel;
        if(now_Vel < Accel * -1.0)
            new_Vel = now_Vel + Accel;
        else
            new_Vel = 0.0;
        printf("Change velocity from %f to %f to avoid collision.\n", now_Vel, new_Vel);
        robot->setVel(new_Vel);
    }
    else
    {
        if(now_Vel - delta_Vel >= max_Vel * -1.0)
        {
            printf("Change velocity from %f to %f\n", now_Vel, now_Vel - delta_Vel);  
            robot->setVel(now_Vel - delta_Vel);
        }
        else
        {
            printf("Velocity = %f, achieve maximum velocity.\n", max_Vel * -1.0);  
            robot->setVel(max_Vel * -1.0);
        }
        putchar('\n');
    }
}
void left_CB(ArRobot *robot)
{
    puts("LEFT pressed");
    double now_RotVel = robot->getRotVel();
    if(now_RotVel + delta_RotVel <= max_RotVel)
    {
        printf("Change rotational velocity from %f to %f\n", now_RotVel, now_RotVel + delta_RotVel);  
        robot->setRotVel(now_RotVel + delta_RotVel);
    }
    else
    {
        printf("Rotational velocity = %f, achieve maximum rotational velocity.\n", max_RotVel);  
        robot->setRotVel(max_RotVel);
    }
    putchar('\n');
}
void right_CB(ArRobot *robot)
{
    puts("RIGHT pressed");
    double now_RotVel = robot->getRotVel();
    if(now_RotVel - delta_RotVel >= max_RotVel * -1.0)
    {
        printf("Change rotational velocity from %f to %f\n", now_RotVel, now_RotVel - delta_RotVel);  
        robot->setRotVel(now_RotVel - delta_RotVel);
    }
    else
    {
        printf("Rotational velocity = %f, achieve maximum rotational velocity.\n", max_RotVel * -1.0);  
        robot->setRotVel(max_RotVel * -1.0);
    }
    putchar('\n');
}
int main(int argc, char **argv)
{
	ArRobot robot;
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

	robot.runAsync(false);

    // Used to perform actions when keyboard keys are pressed
    ArKeyHandler keyHandler;
    keyHandler.addKeyHandler(keyHandler.UP, new ArGlobalFunctor2<ArRobot *, ArRangeDevice *>(&up_CB, &robot, &sonar));
    keyHandler.addKeyHandler(keyHandler.DOWN, new ArGlobalFunctor2<ArRobot *, ArRangeDevice *>(&down_CB, &robot, &sonar));
    keyHandler.addKeyHandler(keyHandler.LEFT, new ArGlobalFunctor1<ArRobot *>(&left_CB, &robot));
    keyHandler.addKeyHandler(keyHandler.RIGHT, new ArGlobalFunctor1<ArRobot *>(&right_CB, &robot));
    
    Aria::setKeyHandler(&keyHandler);
    /*  ArRobot contains an exit action for the Escape key. It also 
        stores a pointer to the keyhandler so that other parts of the program can
        use the same keyhandler. */
    robot.attachKeyHandler(&keyHandler);
    printf("You may press escape to exit\n");

	robot.lock();

	robot.setVel(0);
    robot.setRotVel(0);

	robot.unlock();

	while(true){
        double now_Vel = robot.getVel();
        if(now_Vel > 0)
        {
            double distance = sonar.currentReadingPolar(-45.0, 45.0);
            if(distance < safety_distance)
            {
                printf("Warning! Obstacle in %f m forward.\n", distance);
                double new_Vel;
                if(now_Vel > Decel)
                    new_Vel = now_Vel - Decel;
                else
                    new_Vel = 0.0;
                printf("Change velocity from %f to %f to avoid collision.\n", now_Vel, new_Vel);
                robot.setVel(new_Vel);
            }
        }
        else if(now_Vel < 0)
        {
            double distance = sonar.currentReadingPolar(135.0, -135.0);
            if(distance < safety_distance)
            {
                printf("Warning! Obstacle in %f m backward.\n", distance);
                double new_Vel;
                if(now_Vel < Accel * -1.0)
                    new_Vel = now_Vel + Accel;
                else
                    new_Vel = 0.0;
                printf("Change velocity from %f to %f to avoid collision.\n", now_Vel, new_Vel);
                robot.setVel(new_Vel);
            }
        }
		printf("%f %f %f\n", robot.getX(), robot.getY(), robot.getTh());
		ArUtil::sleep(300);
	}
    
	Aria::shutdown();

	Aria::exit(0);
}
