#include "Aria.h"
#include <math.h>
#define PI acos(-1.0)
#define tolerance 100.0
#define MOVE 0
#define FOLLOW 1
#define DETECT 2
#define safety_margin 1200.0
#define margin_error 200.0
#define move_distance 400.0

bool obstacle_judge(double *start_angle, double *end_angle, ArRangeDevice *sonar)
{
    bool obstacle = false;
    int area[2] = {0, 1};
    for(int i = 0; i < 2; i++)
    {
        double distance = sonar->currentReadingPolar(start_angle[area[i]], end_angle[area[i]]);
        if(distance < safety_margin)
            obstacle = true;
    }
    return obstacle;
}
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
void rotate_delta(ArRobot *robot, double delta)
{
    robot->setDeltaHeading(delta); // use this method to ensure the robot rotate direction
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
double find_min_angle(ArRobot *robot, ArRangeDevice *sonar, double *start_angle, double *end_angle)
{
    int delta_angle = 30;
    double min_distance = 1E5;
    double min_angle = -1;
    for(int i = 0; i < (360 / delta_angle); i++)
    {
        double distance = sonar->currentReadingPolar(start_angle[0], end_angle[0]);
        printf("Theta = %f, distance = %f\n", robot->getTh(), distance);
        if(distance < min_distance)
        {
            min_angle = robot->getTh();
            min_distance = distance;
        }
        rotate_delta(robot, delta_angle);
    }
    return min_angle;
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
    int status = MOVE;
    double margin;
    double start_angle[8];
    double end_angle[8];
    for(int i = 0; i < 8; i++) // devide 360 degrees into 8 region
    {
        start_angle[i] = 45.0 * i - 22.5;
        end_angle[i] = 45.0 * (i + 1) - 22.5;
        if(start_angle[i] > 180.0)
            start_angle[i] -= 360.0;
        if(end_angle[i] > 180.0)
            end_angle[i] -= 360.0;
    }
    double direction_goal;
    if(x_goal == 0)
    {
        if(y_goal > 0)
            direction_goal = 90.0;
        else 
            direction_goal = -90.0;
    }
    else
    {
        direction_goal = atan(y_goal / x_goal) * 180.0 / PI;
        if(y_goal < 0 && direction_goal > 0)
            direction_goal -= 180.0;
        else if(y_goal > 0 && direction_goal < 0)
            direction_goal += 180.0;
    }
    rotate(&robot, direction_goal);
    while(!reach)
    {
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
                if(status == MOVE)
                {
                    if(obstacle_judge(start_angle, end_angle, &sonar))
                    {   
                        puts("Obstacle found");
                        status = DETECT;
                    }
                    else
                        robot.move(move_distance);
                }
                else if(status == DETECT)
                {
                    puts("DETECT");
                    double min_angle = find_min_angle(&robot, &sonar, start_angle, end_angle);
                    printf("min angle = %f\n", min_angle);
                    rotate(&robot, min_angle + 90.0);
                    margin = sonar.currentReadingPolar(start_angle[6], end_angle[6]);
                    printf("now distance is %f\n", margin);
                    status = FOLLOW;
                }
                else
                {
                    puts("FOLLOW");
                    double current_margin = sonar.currentReadingPolar(start_angle[6], end_angle[6]);
                    if(obstacle_judge(start_angle, end_angle, &sonar))
                    {
                        puts("Obstacle found");
                        status = DETECT;
                    }
                    else if(current_margin - margin > margin_error)
                    {
                        puts("obstacle disappear");
                        rotate_delta(&robot, -45);
                    }
                    else if(margin > current_margin)
                    {
                        puts("Turn left");
                        rotate_delta(&robot, 5.0);
                    }
                    else
                    {
                        puts("Turn right");
                        rotate_delta(&robot, -5.0);
                    }
                    margin = current_margin;
                    double direction;
                    if(del_x == 0)
                    {
                        if(del_y > 0)
                            direction = 90.0;
                        else 
                            direction = -90.0;
                    }
                    else
                    {
                        direction = atan(del_y / del_x) * 180.0 / PI;
                        if(del_y < 0 && direction > 0)
                            direction -= 180.0;
                        else if(del_y > 0 && direction < 0)
                            direction += 180.0;
                    }  
                    /* To be completed : Change status to MOVE when direction is close enough to direction_goal */
                    robot.move(move_distance);
                }
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
