#include "stdio.h"
#include "stdlib.h"
#include "sdl.h"
#include "SDL2_gfxPrimitives.h"
#include "time.h"

#include "formulas.h"
#include "wall.h"
#include "robot.h"

int done = 0;


int main(int argc, char *argv[]) {
    SDL_Window *window;
    SDL_Renderer *renderer;

    if(SDL_Init(SDL_INIT_VIDEO) < 0){
        return 1;
    }

    window = SDL_CreateWindow("Robot Maze", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, OVERALL_WINDOW_WIDTH, OVERALL_WINDOW_HEIGHT, SDL_WINDOW_OPENGL);
    window = SDL_CreateWindow("Robot Maze", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, OVERALL_WINDOW_WIDTH, OVERALL_WINDOW_HEIGHT, SDL_WINDOW_OPENGL);
    renderer = SDL_CreateRenderer(window, -1, 0);

    struct Robot robot;
    struct Wall_collection *head = NULL;
    int front_left_sensor, front_right_sensor, right_front_sensor, right_back_sensor=0;
    clock_t start_time, end_time;
    int msec;

    // SETUP MAZE

    /*insertAndSetFirstWall(&head, 1,  OVERALL_WINDOW_WIDTH/2, OVERALL_WINDOW_HEIGHT/2, 10, OVERALL_WINDOW_HEIGHT/2);
    insertAndSetFirstWall(&head, 2,  OVERALL_WINDOW_WIDTH/2-100, OVERALL_WINDOW_HEIGHT/2+100, 10, OVERALL_WINDOW_HEIGHT/2-100);
    insertAndSetFirstWall(&head, 3,  OVERALL_WINDOW_WIDTH/2-250, OVERALL_WINDOW_HEIGHT/2+100, 150, 10);
    insertAndSetFirstWall(&head, 4,  OVERALL_WINDOW_WIDTH/2-150, OVERALL_WINDOW_HEIGHT/2, 150, 10);
    insertAndSetFirstWall(&head, 5,  OVERALL_WINDOW_WIDTH/2-250, OVERALL_WINDOW_HEIGHT/2-200, 10, 300);
    insertAndSetFirstWall(&head, 6,  OVERALL_WINDOW_WIDTH/2-150, OVERALL_WINDOW_HEIGHT/2-100, 10, 100);
    insertAndSetFirstWall(&head, 7,  OVERALL_WINDOW_WIDTH/2-250, OVERALL_WINDOW_HEIGHT/2-200, 450, 10);
    insertAndSetFirstWall(&head, 8,  OVERALL_WINDOW_WIDTH/2-150, OVERALL_WINDOW_HEIGHT/2-100, 250, 10);
    insertAndSetFirstWall(&head, 9,  OVERALL_WINDOW_WIDTH/2+200, OVERALL_WINDOW_HEIGHT/2-200, 10, 300);
    insertAndSetFirstWall(&head, 10,  OVERALL_WINDOW_WIDTH/2+100, OVERALL_WINDOW_HEIGHT/2-100, 10, 300);
    insertAndSetFirstWall(&head, 11,  OVERALL_WINDOW_WIDTH/2+100, OVERALL_WINDOW_HEIGHT/2+200, OVERALL_WINDOW_WIDTH/2-100, 10);
    insertAndSetFirstWall(&head, 12,  OVERALL_WINDOW_WIDTH/2+200, OVERALL_WINDOW_HEIGHT/2+100, OVERALL_WINDOW_WIDTH/2-100, 10);*/

    // maze construction
    /*
    insertAndSetFirstWall(&head, 1, 0, OVERALL_WINDOW_HEIGHT-10, OVERALL_WINDOW_WIDTH, 10);
    insertAndSetFirstWall(&head, 2, 0, OVERALL_WINDOW_HEIGHT-90, OVERALL_WINDOW_WIDTH/2-80, 10);
    insertAndSetFirstWall(&head, 3, OVERALL_WINDOW_WIDTH/2-90, 160, 10, 240);
    insertAndSetFirstWall(&head, 4, OVERALL_WINDOW_WIDTH/2-10, 80, 10, OVERALL_WINDOW_HEIGHT-90);
    insertAndSetFirstWall(&head, 5, 0, 0, OVERALL_WINDOW_WIDTH, 10);
    insertAndSetFirstWall(&head, 6, 0, 0, 10, 160);
    insertAndSetFirstWall(&head, 7, 0, 160, OVERALL_WINDOW_WIDTH/2-80, 10);
    insertAndSetFirstWall(&head, 8, 80, 80, OVERALL_WINDOW_WIDTH-160, 10);
    insertAndSetFirstWall(&head, 9, 80, 80, OVERALL_WINDOW_WIDTH-160, 10);
    insertAndSetFirstWall(&head, 10, OVERALL_WINDOW_WIDTH-10, 0, 10, OVERALL_WINDOW_HEIGHT-80);
    insertAndSetFirstWall(&head, 11, 80, 80, OVERALL_WINDOW_WIDTH-160, 10);
    insertAndSetFirstWall(&head, 12, OVERALL_WINDOW_WIDTH-90, 80, 10, 240);
    insertAndSetFirstWall(&head, 13, OVERALL_WINDOW_WIDTH-170, 80, 10, 240);
    insertAndSetFirstWall(&head, 14, OVERALL_WINDOW_WIDTH-170, 320, 90, 10);
    insertAndSetFirstWall(&head, 15, OVERALL_WINDOW_WIDTH-250, 160, 10, 240);
    insertAndSetFirstWall(&head, 16, OVERALL_WINDOW_WIDTH-250, OVERALL_WINDOW_HEIGHT-90, 250, 10);*/

    // maze construction
    insertAndSetFirstWall(&head, 1,  OVERALL_WINDOW_WIDTH/2-300, OVERALL_WINDOW_HEIGHT/2-200, 5, OVERALL_WINDOW_HEIGHT/2+165);
    insertAndSetFirstWall(&head, 2,  OVERALL_WINDOW_WIDTH/2-300, OVERALL_WINDOW_HEIGHT/2-200, 600, 5);
    insertAndSetFirstWall(&head, 3,  OVERALL_WINDOW_WIDTH/2+300, OVERALL_WINDOW_HEIGHT/2-200, 5, OVERALL_WINDOW_HEIGHT/2-100);
    insertAndSetFirstWall(&head, 4,  OVERALL_WINDOW_WIDTH/2-300, OVERALL_WINDOW_HEIGHT/2+200, 605, 5);
    insertAndSetFirstWall(&head, 5,  OVERALL_WINDOW_WIDTH/3-140, OVERALL_WINDOW_HEIGHT/2-150, 5, OVERALL_WINDOW_HEIGHT/2-50);
    insertAndSetFirstWall(&head, 6,  OVERALL_WINDOW_WIDTH/3-140, OVERALL_WINDOW_HEIGHT/2, 50, 5);
    insertAndSetFirstWall(&head, 7,  OVERALL_WINDOW_WIDTH/3-90, OVERALL_WINDOW_HEIGHT/2, 5, OVERALL_WINDOW_HEIGHT/2-150);
    insertAndSetFirstWall(&head, 8,  OVERALL_WINDOW_WIDTH/3-135, OVERALL_WINDOW_HEIGHT/2+90, 50, 5);
    insertAndSetFirstWall(&head, 9,  OVERALL_WINDOW_WIDTH/3-140, OVERALL_WINDOW_HEIGHT/2+90, 5, OVERALL_WINDOW_HEIGHT/2-125);
    insertAndSetFirstWall(&head, 10,  OVERALL_WINDOW_WIDTH/3-140, OVERALL_WINDOW_HEIGHT/2-100, 190, 5);
    insertAndSetFirstWall(&head, 11,  OVERALL_WINDOW_WIDTH/3-80, OVERALL_WINDOW_HEIGHT/2-200, 5, OVERALL_WINDOW_HEIGHT/2-185);
    insertAndSetFirstWall(&head, 12,  OVERALL_WINDOW_WIDTH/3-25, OVERALL_WINDOW_HEIGHT/2-155, 5, OVERALL_WINDOW_HEIGHT/2-185);
    insertAndSetFirstWall(&head, 13,  OVERALL_WINDOW_WIDTH/3+40, OVERALL_WINDOW_HEIGHT/2-150, 320, 5);
    insertAndSetFirstWall(&head, 14,  OVERALL_WINDOW_WIDTH/3+355, OVERALL_WINDOW_HEIGHT/2-150, 5, OVERALL_WINDOW_HEIGHT/2-185);
    insertAndSetFirstWall(&head, 15,  OVERALL_WINDOW_WIDTH/3+360, OVERALL_WINDOW_HEIGHT/2-60, 52, 5);
    insertAndSetFirstWall(&head, 16,  OVERALL_WINDOW_WIDTH/3+355, OVERALL_WINDOW_HEIGHT/2-60, 5, OVERALL_WINDOW_HEIGHT/2-140);
    insertAndSetFirstWall(&head, 17,  OVERALL_WINDOW_WIDTH/3+100, OVERALL_WINDOW_HEIGHT/2-150, 5, OVERALL_WINDOW_HEIGHT/2-140);
    insertAndSetFirstWall(&head, 18,  OVERALL_WINDOW_WIDTH/3-80, OVERALL_WINDOW_HEIGHT/2-50, 185, 5);
    insertAndSetFirstWall(&head, 19,  OVERALL_WINDOW_WIDTH/3-25, OVERALL_WINDOW_HEIGHT/2-50, 5, OVERALL_WINDOW_HEIGHT/2-50);
    insertAndSetFirstWall(&head, 20,  OVERALL_WINDOW_WIDTH/3-80, OVERALL_WINDOW_HEIGHT/2+140, 60, 5);
    insertAndSetFirstWall(&head, 21,  OVERALL_WINDOW_WIDTH/3-20, OVERALL_WINDOW_HEIGHT/2+90, 50, 5);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/3+30, OVERALL_WINDOW_HEIGHT/2+40, 5, OVERALL_WINDOW_HEIGHT/2-185);
    insertAndSetFirstWall(&head, 23,  OVERALL_WINDOW_WIDTH/3+30, OVERALL_WINDOW_HEIGHT/2+40, 330, 5);
    insertAndSetFirstWall(&head, 24,  OVERALL_WINDOW_WIDTH/3+355, OVERALL_WINDOW_HEIGHT/2-60, 5, OVERALL_WINDOW_HEIGHT/2-140);
    insertAndSetFirstWall(&head, 25,  OVERALL_WINDOW_WIDTH/3, OVERALL_WINDOW_HEIGHT/2-50, 310, 5);
    insertAndSetFirstWall(&head, 26,  OVERALL_WINDOW_WIDTH/3+310, OVERALL_WINDOW_HEIGHT/2-100, 5, OVERALL_WINDOW_HEIGHT/2-140);
    insertAndSetFirstWall(&head, 27,  OVERALL_WINDOW_WIDTH/3+150, OVERALL_WINDOW_HEIGHT/2-110, 5, OVERALL_WINDOW_HEIGHT/2-215);
    insertAndSetFirstWall(&head, 28,  OVERALL_WINDOW_WIDTH/3+150, OVERALL_WINDOW_HEIGHT/2-110, 120, 5);
    insertAndSetFirstWall(&head, 29,  OVERALL_WINDOW_WIDTH/3+270, OVERALL_WINDOW_HEIGHT/2-110, 5, OVERALL_WINDOW_HEIGHT/2-215);
    insertAndSetFirstWall(&head, 30,  OVERALL_WINDOW_WIDTH/3+150, OVERALL_WINDOW_HEIGHT/2-90, 125, 5);
    insertAndSetFirstWall(&head, 31,  OVERALL_WINDOW_WIDTH/3+40, OVERALL_WINDOW_HEIGHT/2, 275, 5);
    insertAndSetFirstWall(&head, 32,  OVERALL_WINDOW_WIDTH/2+300, OVERALL_WINDOW_HEIGHT/2-200, 5, OVERALL_WINDOW_HEIGHT/2-100);



    setup_robot(&robot);
    updateAllWalls(head, renderer);

    SDL_Event event;
    while(!done){
        SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
        SDL_RenderClear(renderer);

        //Move robot based on user input commands/auto commands
        if (robot.auto_mode == 1)
            robotAutoMotorMove(&robot, front_left_sensor, front_right_sensor, right_front_sensor, right_back_sensor);
        robotMotorMove(&robot);

        //Check if robot reaches endpoint. and check sensor values
        if ((checkRobotReachedEnd(&robot, OVERALL_WINDOW_WIDTH, OVERALL_WINDOW_HEIGHT/2+100, 10, 100))){
            end_time = clock();
            msec = (end_time-start_time) * 1000 / CLOCKS_PER_SEC;
            robotSuccess(&robot, msec);
        }
        else if(checkRobotHitWalls(&robot, head))
            robotCrash(&robot);
        //Otherwise compute sensor information
        else {
            // SENSORS
            front_left_sensor = checkRobotSensorFrontLeft(&robot, head);
            front_right_sensor = checkRobotSensorFrontRight(&robot, head);
            right_front_sensor = checkRobotSensorRightFront(&robot, head);
            right_back_sensor = checkRobotSensorRightBack(&robot, head);
        }
        robotUpdate(renderer, &robot);
        updateAllWalls(head, renderer);

        // Check for user input
        SDL_RenderPresent(renderer);
        while(SDL_PollEvent(&event)){
            if(event.type == SDL_QUIT){
                done = 1;
            }
            const Uint8 *state = SDL_GetKeyboardState(NULL);
            if(state[SDL_SCANCODE_UP] && robot.direction != DOWN){
                robot.direction = UP;
            }
            if(state[SDL_SCANCODE_DOWN] && robot.direction != UP){
                robot.direction = DOWN;
            }
            if(state[SDL_SCANCODE_LEFT] && robot.direction != RIGHT){
                robot.direction = LEFT;
            }
            if(state[SDL_SCANCODE_RIGHT] && robot.direction != LEFT){
                robot.direction = RIGHT;
            }
            if(state[SDL_SCANCODE_SPACE]){
                setup_robot(&robot);
            }
            if(state[SDL_SCANCODE_RETURN]){
                // Set auto mode to true
                robot.auto_mode = 1;
                start_time = clock();
            }
        }

        SDL_Delay(120);
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    printf("DEAD\n");
}