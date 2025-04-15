#include "../include/swarm.h"

static void gps2meter(sts *sts, int id)
{
    double dlat = sts->gps_lat[id] - sts->req_lat[id];  
    double dlon = sts->gps_lon[id] - sts->req_lon[id];  
    double lat1 = sts->t_lat[id] * DEG_TO_RAD;
    double lat2 = sts->gps_lat[id] * DEG_TO_RAD;
    double dlat_rad = dlat * DEG_TO_RAD;
    double dlon_rad = dlon * DEG_TO_RAD;

    double a = sin(dlat_rad/2) * sin(dlat_rad/2) +
               cos(lat1) * cos(lat2) * 
               sin(dlon_rad/2) * sin(dlon_rad/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    sts->t2m_distance[id] = EARTH_RADIUS * c;
    
    //printf("distance: %.2f meters\n", sts->t2m_distance);
}

static void calculate_bearing_alt(sts *sts, int id)
{
    // Convert to radians
    double lat1 = sts->gps_lat[id] * DEG_TO_RAD;
    double lon1 = sts->gps_lon[id] * DEG_TO_RAD;
    double lat2 = sts->req_lat[id] * DEG_TO_RAD;
    double lon2 = sts->req_lon[id] * DEG_TO_RAD;

    // Calculate bearing
    double dlon = lon2 - lon1;
    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    sts->bearing[id] = atan2(y, x);

    // Convert to degrees
    sts->bearing[id] = sts->bearing[id] * RAD_TO_DEG;
    // Normalize to 0-360
    sts->bearing[id] = fmod((sts->bearing[id] + 360.0), 360.0);

    // Calculate heading error (difference between current heading and required bearing)
    sts->bearing_error[id] = sts->bearing[id] - sts->heading[id];
    // Normalize to -180 to +180
    if (sts->bearing_error[id] > 180) sts->bearing_error[id] -= 360;
    if (sts->bearing_error[id] < -180) sts->bearing_error[id] += 360;

    sts->t2m_altitude[id] = sts->gps_alt[id] - sts->req_alt[id];

    //printf("ID: %d >> Current heading: %.2f°, Bearing to home: %.2f°, Error: %.2f°, different alt: %.2f\n", 
      //     id, sts->heading[id], sts->bearing[id], sts->bearing_error[id], sts->t2m_altitude[id]);
}

static void rc_drive(sts *sts, joy_s *joy, gains *gains, int id)
{
    // calculate the rate alt and distance
    gains->heading_rate = sts->bearing_error[id]/180; // roll
    gains->alt_rate = -sts->t2m_altitude[id]/100; // pitch
    gains->distance_rate = sts->t2m_distance[id]/500; // throttle

    gains->proportional_heading = gains->p_gain_heading * gains->heading_rate;
    gains->proportional_alt = gains->p_gain_alt * gains->alt_rate;
    gains->proportional_distance = gains->p_gain_distance * gains->distance_rate;

    gains->integral_heading =+ gains->i_gain_heading * gains->heading_rate_last;
    gains->integral_alt =+ gains->i_gain_alt * gains->alt_rate_last;
    gains->integral_distance =+ gains->i_gain_distance * gains->distance_rate_last;

    gains->derivative_heading = gains->d_gain_heading * (gains->heading_rate - gains->heading_rate_last);
    gains->derivative_alt = gains->d_gain_alt * (gains->alt_rate - gains->alt_rate_last);
    gains->derivative_distance = gains->d_gain_distance * (gains->distance_rate - gains->distance_rate_last);

    joy->roll[id] = 1500 + gains->proportional_heading + gains->integral_heading + gains->derivative_heading;
    joy->pitch[id] = 1500 + gains->proportional_alt + gains->integral_alt + gains->derivative_alt;
    joy->throttle[id] = 1500 + gains->proportional_distance + gains->integral_distance + gains->derivative_distance;

    gains->heading_rate_last = gains->heading_rate;
    gains->alt_rate_last = gains->alt_rate;
    gains->distance_rate_last = gains->distance_rate;

    double decay = 0.95;
    gains->integral_heading = gains->integral_heading*decay;
    gains->integral_distance = gains->integral_distance*decay;
    gains->i_gain_alt = gains->integral_alt*decay;
}
static void init_gains(gains *gains)
{
    gains->heading_rate = 0;
    gains->alt_rate = 0;
    gains->distance_rate = 0;

    gains->p_gain_heading = 600;
    gains->p_gain_alt = 500;
    gains->p_gain_distance = 50;

    gains->i_gain_heading = 20;
    gains->i_gain_alt = 20;
    gains->i_gain_distance = 0.1;

    gains->d_gain_heading = 0.1;
    gains->d_gain_alt = 0.1;
    gains->d_gain_distance = 0.1;

    gains->heading_rate_last = 0;
    gains->alt_rate_last = 0;
    gains->distance_rate_last = 0;
}

void rc_init(joy_s *joy, sts *sts, gains *gains)
{
    init_gains(gains);
    for (int id = 0; id < UAV_COUNT; id++)
    {
        gps2meter(sts, id);
        calculate_bearing_alt(sts, id);
        rc_drive(sts, joy, gains, id);
    }
}