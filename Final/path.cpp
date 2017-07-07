#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
using namespace cv;

#define CIRCLE 0
#define SQUARE 1

#define GOAL_COEFFICIENT 1E3
#define OBSTACLE_COEFFICIENT 2E2

#define MARGIN 56

#define PI acos(-1.0)

const int MAX_WIDTH = 5E2;
const int MAX_HEIGHT = 5E2;

const int MAX_OBSTACLE_NUM = 1E2;

const int SENSOR_RANGE = 2E2; 

int step_size = 30;
int goal_step_size = 25;

using namespace std;
using namespace cv;

int obstacle_num;
int obstacle_x[MAX_OBSTACLE_NUM];
int obstacle_y[MAX_OBSTACLE_NUM];
int now_x = 20, now_y = 20;
int goal_x = 480, goal_y = 480;

double Euclidean_dist(int x_1, int y_1, int x_2, int y_2)
{
    double del_x = x_1 - x_2;
    double del_y = y_1 - y_2;
    return sqrt(del_x * del_x + del_y * del_y);
}

void obstacle_generate(Mat &image, int center_x, int center_y, int obstacle_size, int shape)
{
    if(shape == CIRCLE)
        circle(image, Point(center_x, center_y), obstacle_size, Scalar(0xFF, 0x80, 0x00), -1, 8);
    
    if(shape == SQUARE)
    {
        int up_x = center_x - (obstacle_size / 2);
        int down_x = center_x + obstacle_size;
        int left_y = center_y - (obstacle_size / 2);
        int right_y = center_y + obstacle_size;
        rectangle(image, Point(up_x, left_y), Point(down_x, right_y), Scalar(0xFF, 0x80, 0x00), -1, 8);
    }
} 

int obstacle_potential(double &F_x, double &F_y, int obstacle_x, int obstacle_y)
{
    double length = Euclidean_dist(now_x, now_y, obstacle_x, obstacle_y);
    F_x -= ((obstacle_x - now_x) / length) * (OBSTACLE_COEFFICIENT / length);
    F_y -= ((obstacle_y - now_y) / length) * (OBSTACLE_COEFFICIENT / length);
}
    
int goal_potential(double &F_x, double &F_y)  
{
    double length = Euclidean_dist(now_x, now_y, goal_x, goal_y);
    F_x += ((goal_x - now_x) / length) * (GOAL_COEFFICIENT / length);
    F_y += ((goal_y - now_y) / length) * (GOAL_COEFFICIENT / length);
}

void find_step(void)
{
    // calculate potential
    int i;
    double F_x = 0, F_y = 0;
    for(i = 0; i < obstacle_num; i++)
    {
        if(Euclidean_dist(now_x, now_y, obstacle_x[i], obstacle_y[i]) < SENSOR_RANGE) 
            obstacle_potential(F_x, F_y, obstacle_x[i], obstacle_y[i]);
    }
    goal_potential(F_x, F_y);
    printf("F_x = %f, F_y = %f\n", F_x, F_y);
    double length = sqrt(F_x * F_x + F_y * F_y);
    
    now_x += (int)(F_x / length * step_size);
    now_y += (int)(F_y / length * step_size);
    if(now_x < 0)
        now_x = 0;
    if(now_x >= MAX_WIDTH)
        now_x = MAX_WIDTH - 1;
    if(now_y < 0)
        now_y = 0;
    if(now_y >= MAX_HEIGHT)
        now_y = MAX_HEIGHT - 1;
}  
    
void move_goal(void)
{
    while(true)
    {
        int flag = 1;
        int theta = (rand() % 360) * PI / 180;
        int new_goal_x = goal_x + (int)(goal_step_size * cos(theta));
        int new_goal_y = goal_y + (int)(goal_step_size * sin(theta));
        
        if(new_goal_x < 0 || new_goal_x >= MAX_WIDTH)
            flag = 0;
        if(new_goal_y < 0 || new_goal_y >= MAX_HEIGHT)
            flag = 0;
        
        int i;
        for(i = 0; i < obstacle_num; i++)
            if(Euclidean_dist(new_goal_x, new_goal_y, obstacle_x[i], obstacle_y[i]) < MARGIN)
                flag = 0;
        
        if(flag)
        {   
            goal_x = new_goal_x;
            goal_y = new_goal_y;
            return;
        }
    }
}

int main()
{
    // make map     
    Mat image(MAX_WIDTH + 20, MAX_HEIGHT + 20, CV_8UC3, Scalar(255, 255, 255));
    
    srand(time(NULL));
    
    obstacle_num = 3;
    obstacle_generate(image, 160, 240, 20, SQUARE);
    obstacle_x[0] = 160;
    obstacle_y[0] = 240;
    obstacle_generate(image, 400, 360, 25, CIRCLE);
    obstacle_x[1] = 400;
    obstacle_y[1] = 360;
    obstacle_generate(image, 390, 100, 30, CIRCLE);
    obstacle_x[0] = 390;
    obstacle_y[0] = 100;
    
    int step_num = 0;
    while(Euclidean_dist(now_x, now_y, goal_x, goal_y) > step_size)
    {
        step_num++;
        printf("now_x = %d, now_y = %d\n", now_x, now_y);
        int original_x = now_x;
        int original_y = now_y;
        find_step();
        line(image, Point(original_x, original_y), Point(now_x, now_y), Scalar(0, 0, 0), 5, 8);
        circle(image, Point(goal_x, goal_y), 8, Scalar(0x00, 0x00, 0xFF), -1, 8);
        imshow("Image", image);
        char file_name[20];
        sprintf(file_name, "%d.jpg", step_num);
        imwrite(file_name, image);
        waitKey(0);
        line(image, Point(original_x, original_y), Point(now_x, now_y), Scalar(200, 200, 200), 5, 8);
        circle(image, Point(goal_x, goal_y), 8, Scalar(0xFF, 0xFF, 0xFF), -1, 8);
        move_goal();
    }
    
    line(image, Point(now_x, now_y), Point(goal_x, goal_y), Scalar(0, 0, 0), 5, 8);
    circle(image, Point(goal_x, goal_y), 8, Scalar(0x00, 0x00, 0xFF), -1, 8);
    imshow("Image",image);
    imwrite("goal.jpg", image);
    waitKey(0);
    return 0;
}

// g++ -I "D:\tools\OpenCV\opencv-build\install\include" -L "D:\tools\OpenCV\opencv-build\install\x64\mingw\lib" path.cpp -lopencv_core320 -lopencv_highgui320 -lopencv_imgproc320 -lopencv_imgcodecs320 -o pathWJ