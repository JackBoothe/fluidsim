#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <SDL_main.h>
#include <iostream>
#include <sstream>
#include <math.h>
#include <random>
#include <tuple>
#include <vector>

using namespace std;

typedef float f32;
typedef double d64;
typedef Uint32 u32;

const int SCREEN_WIDTH = 1400;
const int SCREEN_HEIGHT = 900;
const d64 PI = 3.141592653;
const int DEPTH = 50;
const f32 RADIUS = 10;
const int NUM_PARTICLES = 300;
const pair<f32, f32> GRAVITY = {0, 0.01};
const f32 COLLISION_DAMPENING = 0.65;
const f32 MIN_VELOCITY_THRESHOLD = 1;
const f32 MASS = 1;
const int SMOOTHINGRADIUS = 100;
const f32 targetDensity = 1;
const f32 pressureMultiplier = 10;
const f32 epsilon = 1e-6;

inline f32 min(f32 a, f32 b) { return (a < b) ? a : b; }
inline f32 max(f32 a, f32 b) { return (a > b) ? a : b; }
inline f32 clamp(f32 x, f32 mi, f32 ma) { return min(max(x, mi), ma); }
inline f32 magnitude(f32 x1, f32 y1, f32 x2, f32 y2){ return (sqrt((x2 - x1)*(x2 - x1) + (y2 - y1) * (y2 - y1)));}
inline f32 normalizeDensity(f32 density, f32 minDensity, f32 maxDensity) {return (density - minDensity) / (maxDensity - minDensity);}

struct SDLApp{ 
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Event event;
};

struct Particle{
    f32 x;
    f32 y;
    f32 velocityX;
    f32 velocityY;
    f32 density;
};

struct Colour {
    u32 r, g, b;
};

Colour interpolateColor(f32 t, Colour colorLow, Colour colorHigh) {
    Colour result;
    result.r = (u32)((1 - t) * colorLow.r + t * colorHigh.r);
    result.g = (u32)((1 - t) * colorLow.g + t * colorHigh.g);
    result.b = (u32)((1 - t) * colorLow.b + t * colorHigh.b);
    return result;
}

void drawFilledCircle(SDL_Renderer* renderer, int centerX, int centerY, int radius) {
    int offsetX, offsetY, d;
    offsetX = 0;
    offsetY = radius;
    d = 3 - 2 * radius;

    while (offsetY >= offsetX) {

        SDL_RenderDrawLine(renderer, centerX - offsetX, centerY - offsetY, centerX + offsetX, centerY - offsetY);
        SDL_RenderDrawLine(renderer, centerX - offsetY, centerY - offsetX, centerX + offsetY, centerY - offsetX);
        SDL_RenderDrawLine(renderer, centerX - offsetX, centerY + offsetY, centerX + offsetX, centerY + offsetY);
        SDL_RenderDrawLine(renderer, centerX - offsetY, centerY + offsetX, centerX + offsetY, centerY + offsetX);

        offsetX++;

        if (d < 0) {
            d += 4 * offsetX + 6;
        } else {
            d += 4 * (offsetX - offsetY) + 10;
            offsetY--;
        }
    }
}

void circleRender(SDL_Renderer* renderer, f32 CenterX, f32 CenterY, f32 radius, int depth){
    f32 angle = PI* 2.f / depth;
    f32 prevX = CenterX + radius * 1, prevY = CenterY - radius * 0;

    for (int i = 1; i <= depth; i++) {
        f32 currAngle = angle * i;
        f32 currX = CenterX + radius * cos(currAngle), currY = CenterY - radius * sin(currAngle);
        SDL_RenderDrawLine(renderer, prevX, prevY, currX, currY);

        prevX = currX, prevY = currY;
    }
}

float SmoothingF(f32 distance){
    f32 volume = PI * pow(SMOOTHINGRADIUS, 4) / 6;
    f32 influence = max(0, SMOOTHINGRADIUS - distance);
    return influence * influence * influence/ volume;
    }

float SmoothingFunctionDerivative(f32 distance){
    if (distance >= SMOOTHINGRADIUS){return 0;}

    f32 scale = 12/ (pow(SMOOTHINGRADIUS, 4) * PI);
    return (distance - SMOOTHINGRADIUS) * scale;
}


float calculateDensity(Particle& sampleP, vector<Particle>& particles){
    f32 density = 0;
    for (int i  = 0; i < NUM_PARTICLES;i++){
        f32 distance = magnitude(sampleP.x, sampleP.y, particles[i].x, particles[i].y);
        f32 influence = SmoothingF(distance);
        density += MASS * influence;
    }
    return density;
}

f32 densityToPresssure(f32& density){
    f32 densityError = density - targetDensity;
    f32 pressureStrength = densityError * pressureMultiplier;
    return pressureStrength;
}

vector<pair<f32, f32>> CalculatePressure(int particleIndex, vector<Particle>& particles){
    vector<pair<f32, f32>> pressureForce;
    pressureForce.resize(NUM_PARTICLES);
    for (int i = 0; i < NUM_PARTICLES; i++){
        if (particleIndex != i){
            
            f32 distance = magnitude(particles[particleIndex].x, particles[particleIndex].y, particles[i].x, particles[i].y);
            if (distance != 0){
                pair<f32, f32>  unitV = make_pair((particles[i].x  - particles[particleIndex].x) / distance, (particles[i].y  - particles[particleIndex].y) / distance);

                f32 slope = SmoothingFunctionDerivative(distance);
                f32 pressureStrength  = densityToPresssure(particles[i].density);
                    
                if (particles[i].density > epsilon){
                    pressureForce[i].first = pressureStrength * unitV.first * slope * MASS / particles[i].density;
                    pressureForce[i].second = pressureStrength* unitV.second * slope * MASS / particles[i].density;

                }else{
                    pressureForce[i].first = 0;
                    pressureForce[i].second = 0; 
                }
            }
        }
    }
   return pressureForce;
}

void handleEvents(bool* quit, SDL_Event* event){
    while (SDL_PollEvent(event) != 0) {
        
            if (event->type == SDL_QUIT) {
                *quit = true;
            }

            if(event->type == SDL_KEYDOWN){
                if (event->key.keysym.sym == SDLK_ESCAPE){
                    *quit = true;
                }
            }    
        }
}


void Render(SDLApp& app, vector<Particle>& particles, u32* startTick, int& frameC,f32& highestD,f32& lowestD){
    int fps = 0;

    Colour colorLow = {0, 0, 255};  
    Colour colorHigh = {255, 0, 0};

    SDL_SetRenderDrawColor(app.renderer, 0, 0, 0, 0);
    SDL_RenderClear(app.renderer);
    frameC ++;

    //blue SDL_SetRenderDrawColor(app.renderer, 32, 81, 255, 255);
    
    //circleRender(app.renderer, particles[0].x, particles[0].y, SMOOTHINGRADIUS, DEPTH);

    for (int i = 0; i < NUM_PARTICLES; i++){

        f32 normalizedDensity = normalizeDensity(particles[i].density, lowestD, highestD);
        Colour particleColor = interpolateColor(normalizedDensity, colorLow, colorHigh);
        SDL_SetRenderDrawColor(app.renderer, particleColor.r, particleColor.g, particleColor.b, 255);
        drawFilledCircle(app.renderer, particles[i].x, particles[i].y, RADIUS);
    }
  
    u32 currentTick = SDL_GetTicks();
   
    if (currentTick - *startTick >= 1000){
        
        fps = frameC;
        frameC = 0;
        *startTick = currentTick;

        ostringstream screenTitle; 
        screenTitle << "Fluid Simulation\t " << fps;
        SDL_SetWindowTitle(app.window,screenTitle.str().c_str());

    }

    SDL_RenderPresent(app.renderer);
    SDL_Delay(16);
}

void Update(SDLApp* app, vector<Particle>& particles, f32& highestD, f32& lowestD) {

    for (int i = 0; i < NUM_PARTICLES; i++) {
        f32 density = calculateDensity(particles[i], particles);

        if (density < lowestD) {
            lowestD = density;
        } else if (density > highestD) {
            highestD = density;
        }

        particles[i].density = density;
    }

    for (int i = 0; i < NUM_PARTICLES; i++) {
        vector<pair<f32, f32>> pressureForce = CalculatePressure(i, particles);
        for (int j = 0; j < NUM_PARTICLES; j++) {
        if (particles[j].density > epsilon) {
            particles[j].velocityX += pressureForce[j].first / particles[j].density;
            particles[j].velocityY += pressureForce[j].second / particles[j].density;
        }
      }
    }
    for (int i = 0; i < NUM_PARTICLES; i++) {

        //particles[i].velocityY += GRAVITY.second;
        //particles[i].velocityX += GRAVITY.first;

        if (particles[i].y >= SCREEN_HEIGHT - RADIUS || particles[i].y <= RADIUS) {
            particles[i].y = clamp(particles[i].y, RADIUS, SCREEN_HEIGHT - RADIUS);
            particles[i].velocityY *= -1 * COLLISION_DAMPENING;

            if (abs(particles[i].velocityY) < MIN_VELOCITY_THRESHOLD) {
                particles[i].velocityY = 0.0f;
            }
        }

        if (particles[i].x >= SCREEN_WIDTH - RADIUS || particles[i].x <= RADIUS) {
            particles[i].x = clamp(particles[i].x, RADIUS, SCREEN_WIDTH - RADIUS);
            particles[i].velocityX *= -1 * COLLISION_DAMPENING;

            if (abs(particles[i].velocityX) < MIN_VELOCITY_THRESHOLD) {
                particles[i].velocityX = 0.0f;
            }
        }

        particles[i].x += particles[i].velocityX;
        particles[i].y += particles[i].velocityY;
    }
}



int main(int argc, char* args[]) {

    f32 lowestD = 0;
    f32 highestD = 0;

    int frameC = 0;
    u32 startTick = SDL_GetTicks();
    SDLApp app;
    bool quit = false;

    app.window = SDL_CreateWindow("Fluid Simulation", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                           SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

    app.renderer= SDL_CreateRenderer(app.window, -1, SDL_RENDERER_ACCELERATED);

    random_device rd; 

    mt19937 gen(rd()); 
    uniform_int_distribution<> distrX(RADIUS, SCREEN_WIDTH-RADIUS);

    mt19937 gen2(rd());
    uniform_int_distribution<> distrY(RADIUS, SCREEN_HEIGHT-RADIUS);

    vector<Particle> particles;
    particles.resize(NUM_PARTICLES);

    for (int i = 0; i < NUM_PARTICLES; i++){
        particles[i].x = distrX(gen);
        particles[i].y = distrY(gen2);
    }

    SDL_SetRenderDrawColor(app.renderer, 255, 255, 255, 255);

    while (!quit) {
        handleEvents(&quit, &app.event);
        Render(app, particles, &startTick, frameC, highestD, lowestD);
        Update(&app, particles, highestD, lowestD);
    }

    SDL_DestroyRenderer(app.renderer);
    SDL_DestroyWindow(app.window);
    SDL_Quit();

    return 0;
}
