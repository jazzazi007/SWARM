#include "../include/swarm.h"

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
    //printf("req lat: %f, req lon: %f req Alt: %f\n", sts->req_lat, sts->req_lon, sts->req_alt);
}

void coverage_area_triangle(sts *sts, int id) 
{
    int base = 20;
    int length = 20;
    float hype;
    int angle = 60;

    hype = sqrt(pow(base/2, 2) + pow(length, 2));
    meter2gps(sts, hype, angle, id);

}