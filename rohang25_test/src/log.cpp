#include "log.h"

int create_file(const char *path)
{
   FILE *fp = fopen(path, "w");
   fclose(fp);
}

// void save_timestamp(const char *path, int wpt, unsigned long long time)
// {
//    FILE *fp = fopen(path, "a+");
//    fprintf(fp, "%d,%lld\n", wpt, time);
//    fclose(fp);
// }

// void save_output(const char *path, int wpt, double data, double altitude, unsigned long long time)
// {
//    FILE *fp = fopen(path, "a+");
//    fprintf(fp, "%lld,<%d>,Remain(m),%f,Height(m),%f\n", time, wpt, data, altitude);
//    fclose(fp);
// }

void save_setpoint_local(const char *path, unsigned long long time, int wpt, std::vector<double> setpoint, std::vector<double> local)
{
   FILE *fp = fopen(path, "a+");
   fprintf(fp, "%lld,<%d>,Setpoint,%lf,%lf,%lf,Local(ENU),%lf,%lf,%lf\n", time, wpt, setpoint[0], setpoint[1], setpoint[2], local[0], local[1], local[2]);
   fclose(fp);
}

void save_object_pixel(const char *path, unsigned long long time, float alt, float score, int center_x, int center_y)
{
   FILE *fp = fopen(path, "a+");
   fprintf(fp, "%lld,Altitude(m),%.4f,Score,%f,Pixel,%d,%d\n", time, alt, score, center_x, center_y);
   fclose(fp);
}

void save_altitude(const char *path, unsigned long long time, float sensor_gps, float global_pos, float home_pos, float local_pos)
{
   FILE *fp = fopen(path, "a+");
   fprintf(fp, "%lld,Altitude(GPS-global-home-local),%.5f,%.5f,%.5f,%.5f\n", time, sensor_gps, global_pos, home_pos, local_pos);
   fclose(fp);
}

// void save_log(const char *path, int autopilot, double *llh, int *datetime, int mission_index)
// {
//     int i = 0;

//     FILE *fp = fopen(path, "a+");
//     fprintf(fp, "%d,", autopilot);
//     for(i=0; i<3; i++){
//         fprintf(fp, "%f,", llh[i]);
//     }
//     for(i=0; i<7; i++){
//         fprintf(fp, "%d,", datetime[i]);
//     }
//     fprintf(fp, "%d\n", mission_index);
//     fclose(fp);
// }
