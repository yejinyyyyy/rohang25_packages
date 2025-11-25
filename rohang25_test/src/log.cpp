#include "log.h"

int create_file(const char *path)
{
   FILE *fp = fopen(path, "w");
   fprintf(fp, "Flight mode,WPT,GPS Time,Latitude(deg),Longitude(deg),Altitude(m),Ax(m/s^2),Ay(m/s^2),Az(m/s^2),Roll,Pitch,Yaw\n");  // 파일 종류마다 바꿔줘야함
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

void save_submit_data(const char *path, int mode, int wpt, uint64_t gps_time, double lat, double lon, float alt, float ax, float ay, float az, float roll, float pitch, float yaw)
{
   // ======================== Time Conversion ========================
   // int year;
   // int month;
   // int day;
   // int hour;
   // int minute;
   // int second;
   // int millisecond;
   // int64_t utc_start;
   // double utcSeconds; // seconds since Unix epoch (can be fractional)
   // int64_t timezone_seconds = 32400;
   // bool first_flag = false;

   // if(!first_flag && time_utc == 0){
   //    return;
   // }
   // if(!first_flag && time_utc != 0){
   //    utc_start = time_utc - timestamp_gps; // microseconds
   //    first_flag = true;
   // }

   // if(first_flag){
   //    // 차량 timestamp + 보정값 => 실제 UTC (microseconds)
   //    int64_t time_real_utc = timestamp_v + utc_start;

   //    // MATLAB 코드에서 Korea 시간으로 변환하기 위해 9시간(32400s) 더함 (microseconds)
   //    int64_t time_real_utc_korea = time_real_utc + timezone_seconds * 1000000LL;

   //    // 분리: 초 부분과 마이크로초 나머지
   //    int64_t secs = time_real_utc_korea / 1000000LL;
   //    int64_t micros = time_real_utc_korea % 1000000LL;
   //    if (micros < 0) { // 음수 시 보정
   //       micros += 1000000LL;
   //       secs -= 1;
   //    }
   //    millisecond = static_cast<int>(micros / 1000LL);

   //    // time_t로 변환 (UTC 기준) — 지금은 이미 KST 오프셋을 더한 상태라 gmtime() 결과가 한국 시간
   //    std::time_t tt = static_cast<std::time_t>(secs);
   //    std::tm tm{};
   //    gmtime_r(&tt, &tm); // POSIX (Ubuntu) 전용, thread-safe

   //    year = tm.tm_year + 1900;
   //    month = tm.tm_mon + 1;
   //    day = tm.tm_mday;
   //    hour = tm.tm_hour;
   //    minute = tm.tm_min;
   //    second = tm.tm_sec;

   //    // 원래 UTC seconds (fractional) - 사용처가 있으면 참고용으로 넣음
   //    utcSeconds = static_cast<double>(time_real_utc) / 1e6;
   // }

   // ======================== Flight mode ========================
   int flight_mode;

   if(mode == 1 || mode == 2)  // Altitude & Manual
      flight_mode = 0; // Manual
   else flight_mode = 1; // Auto

   // ======================== Accel ========================
   float g = 9.8; // m/s^2
   float Ax = ax + g*sin(pitch);
   float Ay = ay - g*cos(pitch)*sin(roll);
   float Az = az - g*cos(pitch)*cos(roll);
   
   // ======================== Save to File ========================
   FILE *fp = fopen(path, "a+");
   fprintf(fp, "%d,%d,%f,%.6f,%.6f,%f,%f,%f,%f,%f,%f,%f\n", flight_mode, wpt, gps_time/1e6, lat, lon, alt, Ax, Ay, Az, roll, pitch, yaw);
   fclose(fp);
}

// void save_object_pixel(const char *path, unsigned long long time, float alt, float score, int center_x, int center_y)
// {
//    FILE *fp = fopen(path, "a+");
//    fprintf(fp, "%lld,Altitude(m),%.4f,Score,%f,Pixel,%d,%d\n", time, alt, score, center_x, center_y);
//    fclose(fp);
// }

// void save_altitude(const char *path, unsigned long long time, float sensor_gps, float global_pos, float home_pos, float local_pos)
// {
//    FILE *fp = fopen(path, "a+");
//    fprintf(fp, "%lld,Altitude(GPS-global-home-local),%.5f,%.5f,%.5f,%.5f\n", time, sensor_gps, global_pos, home_pos, local_pos);
//    fclose(fp);
// }

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
