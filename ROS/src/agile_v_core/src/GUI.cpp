#include "GUI.h"

SDL_Window *window;
SDL_Renderer *renderer;
SDL_Point wheel[9];

int GUI_Init(void){
    /* Initialise SDL */
    if( SDL_Init( SDL_INIT_VIDEO ) < 0){
        fprintf( stderr, "Could not initialise SDL: %s\n", SDL_GetError() );
        return -1;
    }
    
    wheel[0] = {-10,30};
    wheel[1] = {10,30};
    wheel[2] = {10,-30};
    wheel[3] = {-10,-30};
    wheel[4] = {-10,30};
    wheel[5] = {-10,-10};
    wheel[6] = {-30,-10};
    wheel[7] = {-30,10};
    wheel[8] = {-10,10};
    
    window = SDL_CreateWindow("SDL2 Vehicle Moniitor",
                              SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 640, 480, 0);
    renderer = SDL_CreateRenderer(window, -1, 0);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    return 0;
}

void GUIUpdateInput(const agile_v_core::joyinfoex& joyinfo){
    SDL_Rect frame1 = { 10, 10, 65535/150 , 10 };
	SDL_Rect frame2 = { 10, 25, 65535/150 , 10 };
	SDL_Rect frame3 = { 10, 40, 65535/150 , 10 };
    SDL_Rect bar1 = { 10, 10, joyinfo.dwXpos/150 , 10 };
	SDL_Rect bar2 = { 10, 25, joyinfo.dwZpos/150 , 10 };
	SDL_Rect bar3 = { 10, 40, joyinfo.dwRpos/150 , 10 };
	SDL_RenderClear(renderer);
	SDL_RenderDrawRect(renderer, &frame1);
	SDL_RenderDrawRect(renderer, &frame2);
	SDL_RenderDrawRect(renderer, &frame3);
	SDL_RenderFillRect(renderer, &bar1);
	SDL_RenderFillRect(renderer, &bar2);
	SDL_RenderFillRect(renderer, &bar3);
	SDL_RenderPresent(renderer);
	return;
}


SDL_Point* pointsRotTrans(int n, SDL_Point *actual, float angle, bool isRAD, int *Trans) {
  const double DEG2RAD = 2.0*M_PI/360.0;
  float radians;
  if (!isRAD){
    radians = angle * DEG2RAD;
  }
  else{
    radians = angle;
  }
  SDL_Point rotated[n];
  for  (int i=0; i<n; i++){
      rotated[i].x = actual[i].x * cos(radians) - actual[i].y * sin(radians) + Trans[0];
      rotated[i].y = actual[i].x * sin(radians) + actual[i].y * cos(radians) + Trans[1];
  }
  return rotated;
}


void DrawWheel0(const std_msgs::UInt16MultiArray& WheelStatus0){
    float angle = Enc[0][0].extractAngle()/2.0;
    static int position[2] = {50, 100};
    const SDL_Point *draw = pointsRotTrans(9, wheel, angle, 1, position);
    SDL_RenderDrawLines(renderer, draw, 9);
}
void DrawWheel1(const std_msgs::UInt16MultiArray& WheelStatus1){
    float angle = Enc[0][1].extractAngle()/2.0;
    static int position[2] = {182, 100};
    const SDL_Point *draw = pointsRotTrans(9, wheel, angle, 1, position);
    SDL_RenderDrawLines(renderer, draw, 9);
}
void DrawWheel2(const std_msgs::UInt16MultiArray& WheelStatus2){
    float angle = Enc[0][2].extractAngle()/2.0;
    static int position[2] = {50, 252};
    const SDL_Point *draw = pointsRotTrans(9, wheel, angle, 1, position);
    SDL_RenderDrawLines(renderer, draw, 9);
}
void DrawWheel3(const std_msgs::UInt16MultiArray& WheelStatus3){
    float angle = Enc[0][3].extractAngle()/2.0;
    static int position[2] = {182, 252};
    const SDL_Point *draw = pointsRotTrans(9, wheel, angle, 1, position);
    SDL_RenderDrawLines(renderer, draw, 9);
}


void SDL_Cleanup(void){
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return;
}
