#include "../include/swarm.h"
#include <stdio.h>
#include <math.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#define GRID_SIZE 45          // 25x25 grid
#define GRID_SPACING 40       // 10 meters per grid cell
#define SCREEN_WIDTH 800      // Keep screen size the same
#define SCREEN_HEIGHT 800     // Make it square
#define PIXELS_PER_GRID (SCREEN_WIDTH / GRID_SIZE) // Pixels per grid cell
#define CIRCLE_RADIUS 50.0  // 50 meters radius
#define MOVEMENT_SPEED 0.0 // Movement speed in radians per frame

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static int display_initialized = 0;
static TTF_Font* font = NULL;
//static const int LABEL_SPACING = 40; // Match GRID_SPACING

// Add these helper functions at the top of the file
static double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    
    double a = sin(dlat/2) * sin(dlat/2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EARTH_RADIUS * c;
}

static void gps_to_meters(double lat, double lon, double ref_lat, double ref_lon, 
                         double *x, double *y) {
    // Convert longitude difference to meters (x-axis)
    *x = haversine_distance(ref_lat, ref_lon, ref_lat, lon);
    if (lon < ref_lon) *x = -*x;
    
    // Convert latitude difference to meters (y-axis)
    *y = haversine_distance(ref_lat, ref_lon, lat, ref_lon);
    if (lat < ref_lat) *y = -*y;
}

void init_display(void) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        return;
    }

    window = SDL_CreateWindow("UAV Swarm Visualization",
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

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        printf("Renderer could not be created! SDL_Error: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return;
    }

    if (TTF_Init() < 0) {
        printf("SDL_ttf could not initialize! SDL_ttf Error: %s\n", TTF_GetError());
        return;
    }

    font = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 10);
    if (!font) {
        printf("Failed to load font! SDL_ttf Error: %s\n", TTF_GetError());
        return;
    }

    display_initialized = 1;
}

static void render_text(const char* text, int x, int y) {
    SDL_Color textColor = {200, 200, 200, 255}; // Light gray text
    SDL_Surface* surface = TTF_RenderText_Solid(font, text, textColor);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
        if (texture) {
            SDL_Rect dstRect = {
                x - surface->w/2,
                y - surface->h/2,
                surface->w,
                surface->h
            };
            SDL_RenderCopy(renderer, texture, NULL, &dstRect);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    }
}

void update_display(sts *sts) {
    if (!display_initialized) return;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            close_display();
            exit(0);
        }
    }

    // Clear screen
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // Draw grid
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
    for (int i = 0; i <= GRID_SIZE; i++) {
        int pos = i * PIXELS_PER_GRID;
        SDL_RenderDrawLine(renderer, pos, 0, pos, SCREEN_HEIGHT);
        SDL_RenderDrawLine(renderer, 0, pos, SCREEN_WIDTH, pos);
    }

    // Draw axes
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderDrawLine(renderer, SCREEN_WIDTH/2, 0, SCREEN_WIDTH/2, SCREEN_HEIGHT);
    SDL_RenderDrawLine(renderer, 0, SCREEN_HEIGHT/2, SCREEN_WIDTH, SCREEN_HEIGHT/2);

    // After drawing grid lines, add meter labels
    char label[16];
    int center_x = SCREEN_WIDTH / 2;
    int center_y = SCREEN_HEIGHT / 2;
    
    // Draw X-axis labels (every 40m)
    for (int i = -GRID_SIZE/2; i <= GRID_SIZE/2; i++) {
        int meters = i * GRID_SPACING;
        int x = center_x + i * PIXELS_PER_GRID;
        if (i % 2 == 0 && i != 0) { // Draw every other label
            snprintf(label, sizeof(label), "%d", meters);
            render_text(label, x, center_y + 15);
        }
    }
    
    // Draw Y-axis labels (every 40m)
    for (int i = -GRID_SIZE/2; i <= GRID_SIZE/2; i++) {
        int meters = i * GRID_SPACING;
        int y = center_y - i * PIXELS_PER_GRID;
        if (i % 2 == 0 && i != 0) { // Draw every other label
            snprintf(label, sizeof(label), "%d", meters);
            render_text(label, center_x + 15, y);
        }
    }

    // Draw axis labels
    render_text("meters", center_x + SCREEN_WIDTH/2 - 30, center_y + 15);
    render_text("meters", center_x + 15, 20);

    // Draw UAVs
    double scale = (double)PIXELS_PER_GRID / GRID_SPACING;
    for (int i = 0; i < UAV_COUNT; i++) {
        double x_meters, y_meters;
        
        // Convert GPS to meters relative to leader
        gps_to_meters(sts->gps_lat[i], sts->gps_lon[i],
                     sts->gps_lat[0], sts->gps_lon[0],
                     &x_meters, &y_meters);

        // Convert to screen coordinates
        int screen_x = SCREEN_WIDTH/2 + (int)(x_meters * scale);
        int screen_y = SCREEN_HEIGHT/2 - (int)(y_meters * scale);
        
        // Draw UAV
        if (i == 0) {
            // Leader (yellow)
            SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
            SDL_Rect leader = {screen_x-5, screen_y-5, 10, 10};
            SDL_RenderFillRect(renderer, &leader);
        } else {
            // Followers (red)
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
            SDL_Rect follower = {screen_x-3, screen_y-3, 6, 6};
            SDL_RenderFillRect(renderer, &follower);
        }
    }

    SDL_RenderPresent(renderer);
    //SDL_Delay(1);
}

void close_display(void) {
    if (font) TTF_CloseFont(font);
    TTF_Quit();
    if (renderer) SDL_DestroyRenderer(renderer);
    if (window) SDL_DestroyWindow(window);
    SDL_Quit();
    display_initialized = 0;
}

/*int main() {
    // Create and initialize sts structure
    sts test_sts;
    
    // Initialize GPS coordinates for testing
    // Leader (UAV 0) at origin
    test_sts.gps_lat[0] = -35.3627010;  // Leader latitude
    test_sts.gps_lon[0] = 149.1652270;  // Leader longitude
    test_sts.gps_alt[0] = 100.0;        // Leader altitude
    
    // Follower 1 - 100m East, 100m North of leader
    test_sts.gps_lat[1] = -35.3618300;  // ~100m North
    test_sts.gps_lon[1] = 149.1663270;  // ~100m East
    test_sts.gps_alt[1] = 100.0;
    
    // Follower 2 - -100m East, 100m North of leader
    test_sts.gps_lat[2] = -35.3720010;  // ~100m North
    test_sts.gps_lon[2] = 149.1641270;  // ~100m West
    test_sts.gps_alt[2] = 100.0;

    // Copy gpsuired positions to gps_ fields
    for (int i = 0; i < 3; i++) {
        test_sts.gps_lat[i] = test_sts.gps_lat[i];
        test_sts.gps_lon[i] = test_sts.gps_lon[i];
        test_sts.gps_alt[i] = test_sts.gps_alt[i];
    }

    // Initialize display
    init_display();

    // Main loop
    while (1) {
        // Update follower positions
        follower1_angle += MOVEMENT_SPEED;
        follower2_angle += MOVEMENT_SPEED;

        // Calculate new positions for followers (reduce radius to fit grid)
        double x1 = (CIRCLE_RADIUS/2) * cos(follower1_angle);
        double y1 = (CIRCLE_RADIUS/2) * sin(follower1_angle);
        double x2 = (CIRCLE_RADIUS/2) * cos(follower2_angle);
        double y2 = (CIRCLE_RADIUS/2) * sin(follower2_angle);

        // Convert meters to GPS coordinates for followers
        double lat_per_meter = 1.0 / (EARTH_RADIUS * DEG_TO_RAD);
        double lon_per_meter = 1.0 / (EARTH_RADIUS * cos(test_sts.gps_lat[0] * DEG_TO_RAD) * DEG_TO_RAD);

        test_sts.gps_lat[1] = test_sts.gps_lat[0] + y1 * lat_per_meter;
        test_sts.gps_lon[1] = test_sts.gps_lon[0] + x1 * lon_per_meter;
        test_sts.gps_lat[2] = test_sts.gps_lat[0] + y2 * lat_per_meter;
        test_sts.gps_lon[2] = test_sts.gps_lon[0] + x2 * lon_per_meter;

        // Update display
        update_display(&test_sts);
    }

    // Close display
    close_display();

    return 0;
}*/