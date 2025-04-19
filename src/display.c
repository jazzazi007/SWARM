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