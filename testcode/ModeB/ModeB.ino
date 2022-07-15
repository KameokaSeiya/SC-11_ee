// variables___GPS
//緯度
double GPSlat_array[5];
double GPSlat_sum = 0;
double GPSlat_data;
//経度
double GPSlng_array[5];
double GPSlng_sum = 0;
double GPSlng_data;
double delta_lng;
//距離
double distance; //直進前後でゴールに近づいているかどうかを表す
double Pre_distance;
// variables___GY-271
double heading_data;
double heading_array[5];
double heading_sum = 0;
double omega;
double azimuth;
double heading;

double desiredDistance = 10.0; //Mode-Fに移行できる距離


//緯度経度から距離を返す関数
double CalculateDis(double GOAL_lng, double GOAL_lat, double gps_longitude, double gps_latitude)
{

  GOAL_lng = deg2rad(GOAL_lng);
  GOAL_lat = deg2rad(GOAL_lat);

  gps_longitude = deg2rad(gps_longitude);
  gps_latitude = deg2rad(gps_latitude);

  double EarthRadius = 6378.137; //

  //目標地点までの距離を導出
  delta_lng = GOAL_lng - gps_longitude;

  distance = EarthRadius * acos(sin(gps_latitude) * sin(GOAL_lat) + cos(gps_latitude) * cos(GOAL_lat) * cos(delta_lng)) * 1000;

  return distance;
}


//角度計算用の関数
double CalculateAngle(double GOAL_lng, double GOAL_lat, double gps_longitude, double gps_latitude)
{

  GOAL_lng = deg2rad(GOAL_lng);
  GOAL_lat = deg2rad(GOAL_lat);

  gps_longitude = deg2rad(gps_longitude);
  gps_latitude = deg2rad(gps_latitude);

  //目標地点までの角度を導出
  delta_lng = GOAL_lng - gps_longitude;
  azimuth = rad2deg(atan2(sin(delta_lng), cos(gps_latitude) * tan(GOAL_lat) - sin(gps_latitude) * cos(delta_lng)));

  if (azimuth < 0)
  {
    azimuth += 360;
  }
  else if (azimuth > 360)
  {
    azimuth -= 360;
  }
  return azimuth;
}

//機体の向き(heading)の計算
double CalculateHeading(heading)
{

  double Sum_headingDegrees=0;
  declinationAngle = (-7.0 + (46.0 / 60.0)) / (180 / PI);
  heading+=declinationAngle;

                if (heading < 0){
                  heading += 2 * PI;
                }
                if (heading > 2 * PI){
                  heading -= 2 * PI;
                }

                // Convert to degrees
                headingDegrees = heading * 180 / M_PI;

                if (headingDegrees < 0){
                  headingDegrees += 360;
                }

                if (headingDegrees > 360){
                  headingDegrees -= 360;
                }

                for(i=0;i<15;i++){
                  Sum_headingDegrees += headingDegrees;
                }

                return (Sum_headingDegrees/15);
             
}


switch(phase){

  case B://Mode-B
  {
     CurrentDistance = CalculateDis(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
          Serial.print("CurrentDistance=");
          Serial.println(CurrentDistance);

    while(desireDistance < CurrentDistance){
     
        // Goalまでの偏角を計算する
       Angle_Goal = CalculateAngle(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);

       for (i = 0; i < 15; i++){
            delay(10);
            Vector norm = compass.readNormalize();
            heading = atan2(norm.YAxis, norm.XAxis);
         }
        Angle_gy270 = CalculateHeading(heading);


         // どちらに回ればいいか計算
              rrAngle = -Angle_gy270 + Angle_Goal;
              if (rrAngle < 0){
                rrAngle += 360;
              }
              if (rrAngle > 360){
                rrAngle -= 360;
              }
              llAngle = Angle_gy270 - Angle_Goal;
              if (llAngle < 0){
                llAngle += 360;
              }
              if (llAngle > 360){
                llAngle -= 360;
              }

            if (rrAngle > llAngle){
              //反時計回り
              if (llAngle > 20){
                 ;//右側のモータの回転数を上げる
              }
            }else{
              //時計回り
              if (rrAngle > 20){
                ;//左側のモータの回転数を上げる
              }
            }

            if(err>0.35)//0.35[rad](約20°以上ロー角傾いたとき
            {
              phase = A;//機体が安定していない場合Mode-Aに移行する
            }
          }
        
        phase = F;//距離が理想値よりも小さくなったらMode-Fに移行する

        break;

       }
