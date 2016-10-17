#include "Aria.h"
#include <math.h>
#define PI acos(-1.0)
#define tolerance 100.0 // tolerance for distance with the goal
void rotate(ArRobot *robot, double direction)
{
    robot->setHeading(direction);
    bool finished = false;
    while(!finished)
    {
        if(robot->isHeadingDone())
            finished = true;
        else
            ArUtil::sleep(100);
    }
    return;
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

    double x_goal_m, y_goal_m, theta_goal_radian;
	scanf("%lf%lf%lf", &x_goal_m, &y_goal_m, &theta_goal_radian);
    double x_goal = x_goal_m * 1000.0;
    double y_goal = y_goal_m * 1000.0;
    double theta_goal = theta_goal_radian * 180 / PI;
    printf("OK! The goal is x = %f mm y = %f mm theta = %f radians (%f degrees)\n", x_goal, y_goal, theta_goal_radian, theta_goal);
	
    robot.lock();

    robot.setVel(0);
    robot.setRotVel(0);
    robot.setMoveDoneDist(10.0); 
    robot.setHeadingDoneDiff(1.0);

	robot.unlock();

    bool reach = false;
    while(!reach)
    {
        double theta = robot.getTh();
		printf("x = %f mm y = %f mm theta = %f degrees\n", robot.getX(), robot.getY(), theta);
        if(robot.isMoveDone())
        {
            robot.stop();
            while(robot.getVel() != 0)
                ArUtil::sleep(100);
            double x_now = robot.getX();
            double y_now = robot.getY();
            double del_x = x_goal - x_now;
            double del_y = y_goal - y_now;
            double distance = sqrt(del_x * del_x + del_y * del_y);
            if(distance < tolerance)
            {
                puts("Reach goal");
                reach = true;
            }
            else
            {
                double direction = atan(del_y / del_x) * 180.0 / PI;
                if(del_y < 0 && direction > 0)
                    direction -= 180.0;
                else if(del_y > 0 && direction < 0)
                    direction += 180.0;
                rotate(&robot, direction);
                double move_distance;
                if(distance > 5000) // it is best to restrict the distance to the range (5000mm, 5000mm] a time if possible 
                    move_distance = 5000.0;
                else
                    move_distance = distance;
                putchar('\n');
		        printf("now position : x = %f mm y = %f mm theta = %f degrees\n", robot.getX(), robot.getY(), theta);
                printf("I will go along %f degrees for %f mm\n", direction, move_distance);
                robot.move(move_distance);
            }
        }
        else
            ArUtil::sleep(300);
	}
    puts("Changing orientation");
    rotate(&robot, theta_goal);
    double theta = robot.getTh();
    double theta_radian = theta * PI / 180;
    printf("Final odometric pose : x = %f m y = %f m theta = %f radians (%f degrees)\n", robot.getX() / 1000, robot.getY() / 1000, theta_radian, theta);

	Aria::shutdown();

	Aria::exit(0);
}
