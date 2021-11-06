#include "robot.h"
#include <stdbool.h>

// Robot parameters setup
void setup_robot(struct Robot *robot){
    robot->x = OVERALL_WINDOW_WIDTH/2-50;
    robot->y = OVERALL_WINDOW_HEIGHT-50;
    robot->true_x = OVERALL_WINDOW_WIDTH/2-50;
    robot->true_y = OVERALL_WINDOW_HEIGHT-50;
    robot->width = ROBOT_WIDTH;
    robot->height = ROBOT_HEIGHT;
    robot->direction = 0;
    robot->last_direction = 0;
    robot->angle = 0;
    robot->currentSpeed = 0;
    robot->crashed = 0;
    robot->auto_mode = 0;
    robot->initRot = 0;
    robot->found = false;
    robot->foundRight = false;
    robot->dead_frame = 0;

    printf("Press arrow keys to move manually, or enter to move automatically\n\n");

    // Simple 1
    //robot->x = 270; robot->y = 460; robot->true_x = 270; robot->true_y = 460; robot->angle = 0;

    // Simple 2
    //robot->x = 620; robot->y = 380; robot->true_x = 620; robot->true_y = 380; robot->angle = 270;

    // Simple 3
    //robot->x = 640-10-270; robot->y = 460; robot->true_x = 640-10-270; robot->true_y = 460; robot->angle = 0;

    // Simple 4
    //robot->x = 0; robot->y = 380; robot->true_x = 0; robot->true_y = 380; robot->angle = 90;

    // Complex 5
    //robot->x = 170; robot->y = 460; robot->true_x = 170; robot->true_y = 460; robot->angle = 0;

    // Complex 6
    //robot->x = 620; robot->y = 40; robot->true_x = 620; robot->true_y = 40; robot->angle = 270;

    // Complex 7
    //robot->x = 640-10-170; robot->y = 460; robot->true_x = 640-10-170; robot->true_y = 460; robot->angle = 0;

    // Complex 8
    //robot->x = 0; robot->y = 40; robot->true_x = 0; robot->true_y = 40; robot->angle = 90;
}

int robot_off_screen(struct Robot * robot){
    if(robot->x < 0 || robot-> y < 0){
        return 0;
    }
    if(robot->x > OVERALL_WINDOW_WIDTH || robot->y > OVERALL_WINDOW_HEIGHT){
        return 0;
    }
    return 1;
}

int checkRobotHitWall(struct Robot * robot, struct Wall * wall) {

    int overlap = checkOverlap(robot->x,robot->width,robot->y,robot->height,
                 wall->x,wall->width,wall->y, wall->height);

    return overlap;
}

int checkRobotHitWalls(struct Robot * robot, struct Wall_collection * head) {
   struct Wall_collection *ptr = head;
   int hit = 0;

   while(ptr != NULL) {
      hit = (hit || checkRobotHitWall(robot, &ptr->wall));
      ptr = ptr->next;
   }
   return hit;

}

int checkRobotReachedEnd(struct Robot * robot, int x, int y, int width, int height){

    int overlap = checkOverlap(robot->x,robot->width,robot->y,robot->height,
                 x,width,y,height);

    return overlap;
}

void robotCrash(struct Robot * robot) {
    robot->currentSpeed = 0;
    if (!robot->crashed)
        printf("Ouchies!!!!!\n\nPress space to start again\n");
    robot->crashed = 1;
}

void robotSuccess(struct Robot * robot, int msec) {
    robot->currentSpeed = 0;
    if (!robot->crashed){
        printf("Success!!!!!\n\n");
        printf("Time taken %d seconds %d milliseconds \n", msec/1000, msec%1000);
        printf("Press space to start again\n");
    }
    robot->crashed = 1;
}

int checkRobotSensor(int x, int y, int sensorSensitivityLength, struct Wall * wall)  {
    //viewing_region of sensor is a square of 2 pixels * chosen length of sensitivity
    int overlap = checkOverlap(x,2,y,sensorSensitivityLength,
                 wall->x,wall->width,wall->y, wall->height);

    return overlap;
}

// Front left
int checkRobotSensorFrontLeft(struct Robot * robot, struct Wall_collection * head) {
    struct Wall_collection *ptr, *head_store;
    int i;
    double xDir, yDir;
    int robotCentreX, robotCentreY, xTL, yTL;
    int score, hit;
    int sensorSensitivityLength = floor(SENSOR_VISION/5);
    float angle = (robot->angle)*PI/180;

    head_store = head;
    robotCentreX = robot->x+ROBOT_WIDTH/2;
    robotCentreY = robot->y+ROBOT_HEIGHT/2;
    score = 0;

    for (i = 0; i < 5; i++)
    {
        ptr = head_store;
        xDir = round(robotCentreX+(-ROBOT_WIDTH/2)*cos(angle)-(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*sin(angle));
        yDir = round(robotCentreY+(-ROBOT_WIDTH/2)*sin(angle)+(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*cos(angle));
        xTL = (int) xDir;
        yTL = (int) yDir;
        hit = 0;

        while(ptr != NULL) {
            hit = (hit || checkRobotSensor(xTL, yTL, sensorSensitivityLength, &ptr->wall));
            ptr = ptr->next;
        }
        if (hit)
            score = i;
    }
    return score;
}

// Front right
int checkRobotSensorFrontRight(struct Robot * robot, struct Wall_collection * head) {
    struct Wall_collection *ptr, *head_store;
    int i;
    double xDir, yDir;
    int robotCentreX, robotCentreY, xTL, yTL;
    int score, hit;
    int sensorSensitivityLength = floor(SENSOR_VISION/5);
    float angle = (robot->angle)*PI/180;

    head_store = head;
    robotCentreX = robot->x+ROBOT_WIDTH/2;
    robotCentreY = robot->y+ROBOT_HEIGHT/2;
    score = 0;

    for (i = 0; i < 5; i++)
    {
        ptr = head_store;
        xDir = round(robotCentreX + (ROBOT_WIDTH/2-2) * cos(angle) - (-ROBOT_HEIGHT/2 - SENSOR_VISION + sensorSensitivityLength * i) * sin(angle));
        yDir = round(robotCentreY + (ROBOT_WIDTH/2-2) * sin(angle) + (-ROBOT_HEIGHT/2 - SENSOR_VISION + sensorSensitivityLength * i) * cos(angle));
        xTL = (int) xDir;
        yTL = (int) yDir;
        hit = 0;

        while(ptr != NULL) {
            hit = (hit || checkRobotSensor(xTL, yTL, sensorSensitivityLength, &ptr->wall));
            ptr = ptr->next;
        }
        if (hit)
            score = i;
    }
    return score;
}

// Right Back sensor
int checkRobotSensorRightBack(struct Robot * robot, struct Wall_collection * head) {
    struct Wall_collection *ptr, *head_store;
    int i;
    double xDir, yDir;
    int robotCentreX, robotCentreY, xTL, yTL;
    int score, hit;
    int sensorSensitivityLength = floor(SENSOR_VISION/5);
    float angle = (robot->angle)*PI/180;

    head_store = head;
    robotCentreX = robot->x+ROBOT_WIDTH/2;
    robotCentreY = robot->y+ROBOT_HEIGHT/2;
    score = 0;

    // Check how many of [5] segments of laser the wall overlaps
    for (i = 0; i < 5; i++)
    {
        ptr = head_store;
        xDir = round(robotCentreX - (ROBOT_HEIGHT/2) * sin(angle) + (ROBOT_WIDTH/2 + SENSOR_VISION - sensorSensitivityLength * i) * cos(angle));
        yDir = round(robotCentreY + (ROBOT_HEIGHT/2) * cos(angle) + (ROBOT_WIDTH/2 + SENSOR_VISION - sensorSensitivityLength * i) * sin(angle));
        xTL = (int) xDir;
        yTL = (int) yDir;
        hit = 0;

        while(ptr != NULL) {
            hit = (hit || checkRobotSensor(xTL, yTL, sensorSensitivityLength, &ptr->wall));
            ptr = ptr->next;
        }
        if (hit)
            score = i;
    }
    return score;
}

// Right Front sensor
int checkRobotSensorRightFront(struct Robot * robot, struct Wall_collection * head) {
    struct Wall_collection *ptr, *head_store;
    int i;
    double xDir, yDir;
    int robotCentreX, robotCentreY, xTL, yTL;
    int score, hit;
    int sensorSensitivityLength = floor(SENSOR_VISION/5);
    float angle = (robot->angle)*PI/180;

    head_store = head;
    robotCentreX = robot->x+ROBOT_WIDTH/2;
    robotCentreY = robot->y+ROBOT_HEIGHT/2;
    score = 0;

    // Check how many of [5] segments of laser the wall overlaps
    for (i = 0; i < 5; i++)
    {
        ptr = head_store;
        xDir = round(robotCentreX + (ROBOT_HEIGHT/2) * sin(angle) + (ROBOT_WIDTH/2 + SENSOR_VISION - sensorSensitivityLength * i) * cos(angle));
        yDir = round(robotCentreY - (ROBOT_HEIGHT/2) * cos(angle) + (ROBOT_WIDTH/2 + SENSOR_VISION - sensorSensitivityLength * i) * sin(angle));
        xTL = (int) xDir;
        yTL = (int) yDir;
        hit = 0;

        while(ptr != NULL) {
            hit = (hit || checkRobotSensor(xTL, yTL, sensorSensitivityLength, &ptr->wall));
            ptr = ptr->next;
        }
        if (hit)
            score = i;
    }
    return score;
}

// DRAWING ROBOT AND VISION LINES
void robotUpdate(struct SDL_Renderer * renderer, struct Robot * robot){
    double xDir, yDir;

    int robotCentreX, robotCentreY, xTR, yTR, xTL, yTL, xBR, yBR, xBL, yBL;
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);

    //Rotating Square
    //Vector rotation to work out corners x2 = x1cos(angle)-y1sin(angle), y2 = x1sin(angle)+y1cos(angle)
    float angle = (robot->angle)*PI/180;
    robotCentreX = robot->x+ROBOT_WIDTH/2;
    robotCentreY = robot->y+ROBOT_HEIGHT/2;

    xDir = round(robotCentreX+(ROBOT_WIDTH/2)*cos(angle)-(-ROBOT_HEIGHT/2)*sin(angle));
    yDir = round(robotCentreY+(ROBOT_WIDTH/2)*sin(angle)+(-ROBOT_HEIGHT/2)*cos(angle));
    xTR = (int) xDir;
    yTR = (int) yDir;

    xDir = round(robotCentreX+(ROBOT_WIDTH/2)*cos(angle)-(ROBOT_HEIGHT/2)*sin(angle));
    yDir = round(robotCentreY+(ROBOT_WIDTH/2)*sin(angle)+(ROBOT_HEIGHT/2)*cos(angle));
    xBR = (int) xDir;
    yBR = (int) yDir;

    xDir = round(robotCentreX+(-ROBOT_WIDTH/2)*cos(angle)-(ROBOT_HEIGHT/2)*sin(angle));
    yDir = round(robotCentreY+(-ROBOT_WIDTH/2)*sin(angle)+(ROBOT_HEIGHT/2)*cos(angle));
    xBL = (int) xDir;
    yBL = (int) yDir;

    xDir = round(robotCentreX+(-ROBOT_WIDTH/2)*cos(angle)-(-ROBOT_HEIGHT/2)*sin(angle));
    yDir = round(robotCentreY+(-ROBOT_WIDTH/2)*sin(angle)+(-ROBOT_HEIGHT/2)*cos(angle));
    xTL = (int) xDir;
    yTL = (int) yDir;

    SDL_RenderDrawLine(renderer,xTR, yTR, xBR, yBR);
    SDL_RenderDrawLine(renderer,xBR, yBR, xBL, yBL);
    SDL_RenderDrawLine(renderer,xBL, yBL, xTL, yTL);
    SDL_RenderDrawLine(renderer,xTL, yTL, xTR, yTR);

    //Front left Sensor
    int sensor_sensitivity =  floor(SENSOR_VISION/5);
    int i;
    int lastX = round(robotCentreX+(-ROBOT_WIDTH/2)*cos(angle)-(-ROBOT_HEIGHT/2)*sin(angle));
    int lastY = round(robotCentreY+(-ROBOT_WIDTH/2)*sin(angle)+(-ROBOT_HEIGHT/2)*cos(angle));

    for (i = 0; i < 5 + 1; i++)
    {
        xDir = round(robotCentreX + (-ROBOT_WIDTH/2) * cos(angle) + (ROBOT_HEIGHT/2 + sensor_sensitivity * i) * sin(angle));
        yDir = round(robotCentreY + (-ROBOT_WIDTH/2) * sin(angle) - (ROBOT_HEIGHT/2 + sensor_sensitivity * i) * cos(angle));

        xTL = (int) xDir;
        yTL = (int) yDir;

        SDL_SetRenderDrawColor(renderer, 255, 60+(20*i), 60+(20*i), 255);
        if(i == 0){
            lastX = xTL;
            lastY = yTL;
            continue;
        }

        SDL_RenderDrawLine(renderer, xTL, yTL, lastX, lastY);
        lastX = xTL;
        lastY = yTL;
    }

    //Front right Sensor
    for (i = 0; i < 5 + 1; i++)
    {
        xDir = round(robotCentreX + (ROBOT_WIDTH/2) * cos(angle) + (ROBOT_HEIGHT/2 + sensor_sensitivity * i) * sin(angle));
        yDir = round(robotCentreY + (ROBOT_WIDTH/2) * sin(angle) - (ROBOT_HEIGHT/2 + sensor_sensitivity * i) * cos(angle));

        xTL = (int) xDir;
        yTL = (int) yDir;

        SDL_SetRenderDrawColor(renderer, 255, 60+(20*i), 60+(20*i), 255);
        if(i == 0){
            lastX = xTL;
            lastY = yTL;
            continue;
        }

        SDL_RenderDrawLine(renderer, xTL, yTL, lastX, lastY);
        lastX = xTL;
        lastY = yTL;
    }

    // Right back
    for (i = 0; i < 5 + 1; i++)
    {
        xDir = round(robotCentreX - (ROBOT_HEIGHT/2) * sin(angle) - (-ROBOT_WIDTH/2 - sensor_sensitivity * i) * cos(angle));
        yDir = round(robotCentreY + (ROBOT_HEIGHT/2) * cos(angle) - (-ROBOT_WIDTH/2 - sensor_sensitivity * i) * sin(angle));

        xTL = (int) xDir;
        yTL = (int) yDir;

        SDL_SetRenderDrawColor(renderer, 255, 60+(20*i), 60+(20*i), 255);
        if(i == 0){
            lastX = xTL;
            lastY = yTL;
            continue;
        }

        SDL_RenderDrawLine(renderer, xTL, yTL, lastX, lastY);
        lastX = xTL;
        lastY = yTL;
    }

    // Right front
    for (i = 0; i < 5 + 1; i++)
    {
        xDir = round(robotCentreX + (ROBOT_HEIGHT/2) * sin(angle) - (-ROBOT_WIDTH/2 - sensor_sensitivity * i) * cos(angle));
        yDir = round(robotCentreY - (ROBOT_HEIGHT/2) * cos(angle) - (-ROBOT_WIDTH/2 - sensor_sensitivity * i) * sin(angle));

        xTL = (int) xDir;
        yTL = (int) yDir;

        SDL_SetRenderDrawColor(renderer, 255, 60+(20*i), 60+(20*i), 255);
        if(i == 0){
            lastX = xTL;
            lastY = yTL;
            continue;
        }

        SDL_RenderDrawLine(renderer, xTL, yTL, lastX, lastY);
        lastX = xTL;
        lastY = yTL;
    }
}


// Move based on robot->direction
void robotMotorMove(struct Robot * robot) {
    double x_offset, y_offset;
    switch(robot->direction){
        case UP :
            robot->currentSpeed += DEFAULT_SPEED_CHANGE;
            if (robot->currentSpeed > MAX_ROBOT_SPEED)
                robot->currentSpeed = MAX_ROBOT_SPEED;
            break;
        case DOWN :
            robot->currentSpeed -= DEFAULT_SPEED_CHANGE;
            if (robot->currentSpeed < -MAX_ROBOT_SPEED)
                robot->currentSpeed = -MAX_ROBOT_SPEED;
            break;
        case LEFT :
            robot->angle = (robot->angle+360-DEFAULT_ANGLE_CHANGE)%360;
            break;
        case RIGHT :
            robot->angle = (robot->angle+DEFAULT_ANGLE_CHANGE)%360;
            break;
    }
    robot->direction = 0;
    x_offset = (-robot->currentSpeed * sin(-robot->angle*PI/180));
    y_offset = (-robot->currentSpeed * cos(-robot->angle*PI/180));

    robot->true_x += x_offset;
    robot->true_y += y_offset;

    x_offset = round(robot->true_x);
    y_offset = round(robot->true_y);

    robot->x = (int) x_offset;
    robot->y = (int) y_offset;
}


// MOVE AI
void robotAutoMotorMove(struct Robot * robot, int front_left_sensor, int front_right_sensor, int right_front_sensor, int right_back_sensor) {
    // DEFAULT MAP BEST 34 778

    if(robot->crashed == 1){
        return;
    }

    // Sensor empty
    bool frontLeftEmpty = front_left_sensor == 0;
    bool frontRightEmpty = front_right_sensor == 0;
    bool frontEmpty = frontLeftEmpty && frontRightEmpty;
    bool rightFrontEmpty = right_front_sensor == 0;
    bool rightBackEmpty = right_back_sensor == 0;

    // Distance
    int front_left_dist = SENSOR_VISION - (front_left_sensor * 5);
    int front_right_dist = SENSOR_VISION - (front_right_sensor * 5);
    int right_front_dist = SENSOR_VISION - (right_front_sensor * 5);
    int right_back_dist = SENSOR_VISION - (right_back_sensor * 5);

    // Static
    int halt_threshold = 10;
    int sweetSpotThreshold = 15;
    int maxSpeed = 5;
    int minSpeed = 0;

    bool stationary = robot->currentSpeed <= minSpeed;

    if(right_front_dist <= sweetSpotThreshold){
        robot->found = true;
        robot->foundRight = true;

        robot->initRot = 90;
    }

    if(robot->initRot < 90){
        robot->direction = RIGHT;
        robot->initRot += DEFAULT_ANGLE_CHANGE;
        return;
    }

    // Side is close
    bool front_left_close = front_left_dist <= halt_threshold;
    bool front_right_close = front_right_dist <= halt_threshold;
    bool right_front_close = right_front_dist <= halt_threshold;
    bool right_back_close = right_back_dist <= halt_threshold;

    printf("______\n");
    printf("Front left: %d, Front right: %d, Right front: %d, Right back: %d, Sweetspot: %d, halt_threshold: %d, speed: %d\n", front_left_dist, front_right_dist, right_front_dist, right_back_dist, sweetSpotThreshold, halt_threshold, robot->currentSpeed);

    if(!robot->foundRight){
        printf("-> Not found right wall\n");
        if(front_right_dist > sweetSpotThreshold && front_left_dist > sweetSpotThreshold && !robot->found){
            if(robot->currentSpeed < maxSpeed / 2){
                printf("-> Up: no sensors\n");
                robot->direction = UP;
            }
        }
        else{
            robot->found = true;
            printf("-> Front encountered\n");
            if(!stationary){
                printf("-> Down: stopping for right wall search\n");
                robot->direction = DOWN;
            }
            else{
                printf("-> Left: finding right wall\n");
                robot->direction = LEFT;
            }
        }
        return;
    }

    bool deadFrame = false;
    if (robot->currentSpeed > maxSpeed){
        // Going faster than max speed
        printf("-> Down: Going faster than max speed!\n");
        robot->direction = DOWN;
    }
    else if (!stationary && frontRightEmpty && right_front_dist > sweetSpotThreshold && !rightFrontEmpty) {
        // Sweetspot adjustment
        printf("-> Right: right wall detected but outside sweetspot\n");
        robot->direction = RIGHT;
    }
    else if(robot->currentSpeed > minSpeed + 1 && !frontEmpty && !(right_front_close && front_right_dist > sweetSpotThreshold)){
        // Slow down when moving faster than half of max speed and front sensor isn't empty
        printf("-> Down: Front detected while too fast\n");
        robot->direction = DOWN;
    }
    else if (!stationary && ((right_front_close && right_front_dist < right_back_dist) || (right_front_close && right_back_close))){ // OLD: robot->currentSpeed > minSpeed && right_front_close
        // Steer away from right wall when moving and right front too close
        printf("-> Left: running into right wall\n");
        robot->direction = LEFT;
    }
    else if (robot->currentSpeed < maxSpeed && frontEmpty && !rightFrontEmpty) {
        // Follow right wall upwards if no front wall
        printf("-> Up: no obstacles and no right branches\n");
        robot->direction = UP; // Speed up
    }
    else if (!stationary && ((front_left_close && robot->last_direction != RIGHT) || front_right_close)) {
        // Try to stop when front is too close and going fast
        printf("-> Down: Front too close\n");
        robot->direction = DOWN;
    }
    else if (rightFrontEmpty && !front_right_close && !front_left_close){
        // Right turn detected
        if(robot->dead_frame > 0){
            printf("-> DEAD FRAME\n");
            deadFrame = true;
            robot->direction = RIGHT;
            robot->last_direction = RIGHT;
            robot->dead_frame -= 1;
        }
        else if(robot->last_direction != RIGHT){
            if(robot->currentSpeed > 0){
                robot->dead_frame = 1;
                printf("Dead frame incremented: %d\n", robot->dead_frame);
            }

            if(robot->currentSpeed <= minSpeed + 3){
                printf("-> Up: speed up for right turn\n");
                robot->direction = UP;
            }
        }
        else if(robot->dead_frame == 0){
            printf("-> Right: right branch\n");
            robot->direction = RIGHT;
        }
    }
    else if(!frontRightEmpty && !rightFrontEmpty){
        // Approaching front wall where right walls are also present
        if(robot->currentSpeed <= maxSpeed / 2){ // Allow for small movement so robot doesn't get stuck
            printf("-> Left: Front wall and side walls (dead end or left turn)\n");
            robot->direction = LEFT;
        }
        else{
            printf("-> Down: Dead end too fast\n");
            robot->direction = DOWN;
        }
    }
    else if (stationary && (front_right_close || front_left_close)){
        // Stationary and any front sensor is close
        if(front_left_dist < front_right_dist){
            printf("-> Right: front left closer than front right\n");
            robot->direction = RIGHT;
        }
        else{
            printf("-> Left: front right closer than front left\n");
            robot->direction = LEFT;
        }
    }
    else if (!stationary && !rightFrontEmpty && !rightBackEmpty && right_front_dist > right_back_dist && right_front_dist > sweetSpotThreshold){
        // Moving and right sensors non-empty but not equal (Try to make robot parallel to right wall)
        printf("-> Right: running away from right wall\n");
        robot->direction = RIGHT;
    }
    else if(!stationary && right_front_dist > sweetSpotThreshold){
        // Moving away from right wall
        printf("-> Right: losing sweetspot\n");
        robot->direction = RIGHT;
    }
    else if(robot->currentSpeed <= minSpeed && !frontLeftEmpty){
        // Stationary and front left wall present (start moving to change states)
        printf("-> UP: Stationary and front left close\n");
        robot->direction = UP;
    }

    if(robot->dead_frame == 0 && !deadFrame){
        robot->last_direction = robot->direction;
    }
}
