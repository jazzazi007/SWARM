#include <SDL2/SDL.h>
#include "../include/swarm.h"

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600

void visualize_coordinates(sts *sts, int id) {
    (void)id; // Unused parameter
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        return;
    }

    SDL_Window *window = SDL_CreateWindow("UAV Coordinates Visualization",
                                          SDL_WINDOWPOS_CENTERED,
                                          SDL_WINDOWPOS_CENTERED,
                                          SCREEN_WIDTH,
                                          SCREEN_HEIGHT,
                                          SDL_WINDOW_SHOWN);
    if (!window) {
        printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
        SDL_Quit();
        return;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        printf("Renderer could not be created! SDL_Error: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return;
    }

    int running = 1;
    SDL_Event event;

    while (running) {
        // Handle events
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = 0;
            }
        }

        // Clear screen
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Black background
        SDL_RenderClear(renderer);

        // Draw UAV coordinates
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // Red for UAVs
        for (int i = 0; i < UAV_COUNT; i++) {
            int x = (int)((sts->req_lon[i] - sts->gps_lon[0]) * 1e6) + SCREEN_WIDTH / 2;
            int y = (int)((sts->req_lat[i] - sts->gps_lat[0]) * -1e6) + SCREEN_HEIGHT / 2;
            SDL_RenderDrawPoint(renderer, x, y);
        }

        // Update screen
        SDL_RenderPresent(renderer);

        SDL_Delay(100); // Delay to control frame rate
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

static void meter2gps(sts *sts, double hype, int deg, int id)
{
    double lat0_rad = sts->gps_lat[0] * DEG_TO_RAD;
    //double lon0_rad = sts->t_lon * DEG_TO_RAD;
    //double t2m_heading = atan2(sts->t_lat - sts->heading_lat_uav, sts->t_lon - sts->heading_lon_uav);
    //printf("t2m heading: %f\n", t2m_heading);
    //double heading = sts->t2m_heading * DEG_TO_RAD;
    double heading = deg * DEG_TO_RAD;
    //printf("heading: %f\n", sts->t2m_heading);

    double dl_x = hype * cos(heading) - 0 * sin(heading);
    double dl_y = hype * sin(heading) + 0 * cos(heading);

    sts->req_lat[id] = sts->gps_lat[0] + dl_x / EARTH_RADIUS * RAD_TO_DEG;
    sts->req_lon[id] = sts->gps_lon[0] + dl_y / (EARTH_RADIUS * cos(lat0_rad)) * RAD_TO_DEG;
    sts->req_alt[id] = sts->gps_alt[0];
    printf("ID GPS: %d\n", id);
    printf("main lat: %f, main lon: %f\n", sts->gps_lat[0], sts->gps_lon[0]);
    printf("req lat: %f, req lon: %f req Alt: %f\n", sts->req_lat[id], sts->req_lon[id], sts->req_alt[id]);
}

void coverage_area_triangle(sts *sts, int id) 
{
    int base = 100;
    int length = 100;
    float hype;
    int angle = 60;

    if (id == 0)
        return ;

    hype = sqrt(pow(base / 2, 2) + pow(length, 2));
    if (id%2 == 0)
        angle = 60;
    else
        angle = 120;
    meter2gps(sts, hype, angle, id);
}