// #include <Arduino.h>
// #include <math.h>
// #include <PS4Controller.h>
// #include <M5Stack.h>
// #include <mcp_can.h>
// #include "cybergear_driver.hh"
// #include "cybergear_can_interface_mcp.hh"
                                                                                                          
// float inc_kp = 0.01;
// float inc_ki = 0.001;
// float inc_kd = 0.01;

// //canid
// uint8_t MASTER_CAN_ID = 0x00;
// uint8_t Rleg_CAN_ID = 0x7F;
// uint8_t Lleg_CAN_ID = 0x7E;

// //cybergeardriver初期化
// CybergearDriver driver1 = CybergearDriver(MASTER_CAN_ID, Rleg_CAN_ID);
// MotorStatus motor_status1;
// CybergearDriver driver2 = CybergearDriver(MASTER_CAN_ID, Lleg_CAN_ID);
// MotorStatus motor_status2;
// CybergearCanInterfaceMcp interface;

// float leg_pos[2] = {0.0f, 0.0f}; //target_pos[0]=Rleg, target_pos[1]=Lleg 
// float leg_origin_pos[2];
// uint8_t mode = MODE_CURRENT;
// float default_kp = 50.0f;      
// float default_kd = 1.0f;     
// float init_speed = 5.0f;     
// float slow_speed = 1.0f;     

// // canのデータ配列の作成
// unsigned char data[8] = {0};

// //can setup
// long unsigned int rxId;
// unsigned char len = 0;
// unsigned char rxBuf[8];
// #define CAN0_INT 15  // Set INT to pin 2
// MCP_CAN CAN0(12);    // Set CS to pin 10

// //RobomasterMotorの変数
// int motor_out[2] = {0, 0}; //motor_out[0]=r_motor,motor_out[0]=l_motor
// const int tire_current_max = 8000;
// unsigned long elapsed_time;
// unsigned long pre_time = 0;
// unsigned long dt;

// float kp = 0.0064;
// float ki = 0.0;
// float kd = 0.0022;

// float integral = 0.0; 
// float pre_error = 0.0; 
// float target_angle = -14.0;

// float tire_angle;
// float tire_pre_angle;
// int encoder_max = 8191;
// int encoder_min = 0;
// bool encoder_calc_flag = true;
// float motor_gear_ratio = 19.2;

// //dualshock
// const int joystick_max = 128;
// bool prev_Touchpad = false;
// bool prev_Triangle = false;
// bool prev_Cross = false;
// bool prev_Up = false;
// bool prev_Down = false;
// bool prev_R1 = false;
// bool prev_L1 = false;
// bool flag = false;
// bool pre_flag = false;

// //　加速度センサ
// float roll, pitch, yaw;

// //関数宣言
// //初期化
// void init_can();
// void init_cybergear();
// //モータコントロール
// void tire_controll();
// void leg_controll();
// void leg_caliblation();
// void motor_value_zero();
// void target_angle_calc();
// void self_balance_controll();
// void gain_adjust();
// //can_send
// void DJI_CAN_send();
// void Cybergear_CAN_send();
// //can_read
// void DJI_CAN_read();
// //m5stackのdisplay
// void display_update();


// void setup(){
//   M5.begin();
//   M5.Power.begin();
//   Serial.begin(115200);

//   PS4.begin("3C:61:05:0D:E5:DE");     

//   init_cybergear();
//   init_can();

//   M5.MPU6886.Init();

//   Serial.println("M5begin!!");
//   M5.Lcd.setTextSize(2); 
//   display_update(); 
// }

// void loop(){
//   M5.update();

//   //センサの読み取り
//   M5.MPU6886.getAhrsData(&pitch, &roll, &yaw);
//   if ( driver1.process_packet() ) {
//     motor_status1 = driver1.get_motor_status();
//   }
//   if ( driver2.process_packet() ) {
//     motor_status2 = driver2.get_motor_status();
//   }

//   DJI_CAN_read();

//   //コントローラ
//   if (PS4.isConnected()) {
//     if(PS4.Share()){
//       init_cybergear();
//     }
//     if(PS4.R3()){
//       motor_value_zero();
//     }
//     leg_caliblation();
//     // tire_controll();
//     leg_controll();
//     gain_adjust(); 
//   }else{
//     motor_value_zero();
//   }

//   //10ms周期で行うもの:シリアルモニタ,cansend
//   auto now = millis();
//   static auto pre = now;
//   if(now - pre > 10) {
//     Serial.printf("roll:%f",roll);
//     Serial.printf("\tr_leg:%f",leg_pos[0]);
//     Serial.printf("\tl_leg:%f",leg_pos[1]);
//     Serial.printf("\toutput:%d",motor_out[0]);
//     // Serial.printf("\tkp:%f",kp);
//     // Serial.printf("\tki:%f",ki);
//     // Serial.printf("\tkd:%f",kd);  
//     Serial.printf("\ttire_angle:%f",tire_angle);  
//     // Serial.printf("\tr_rad:%f",motor_status1.position);  
//     // Serial.printf("\tl_rad:%f",motor_status2.position);  
//     Serial.println();
//     //姿勢制御
//     self_balance_controll();
//     DJI_CAN_send();
//     Cybergear_CAN_send();
//     pre = now;
//   }

//   //100ms周期で行うもの:display
//   static auto last_display = now;
//   if(now - last_display > 100){
//     display_update();
//     last_display = now;
//   }
// }

// void init_can() {
//   if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
//     Serial.println("MCP2515 Initialized Successfully!");
//   } else {
//     Serial.println("Error Initializing MCP2515...");
//   }
//   CAN0.setMode(MCP_NORMAL); 
// }

// void init_cybergear(){
//   interface.init();

//   mode = MODE_CURRENT;

//   driver1.init(&interface);
//   driver1.init_motor(mode);
//   driver1.set_limit_speed(init_speed);
//   driver1.enable_motor();
//   driver2.init(&interface);
//   driver2.init_motor(mode);
//   driver2.set_limit_speed(init_speed);
//   driver2.enable_motor();
// }

// void leg_caliblation() {
//   if(PS4.Options()){
//     leg_origin_pos[0] = motor_status1.position;
//     leg_origin_pos[1] = -0.81;

//     mode = MODE_POSITION;

//     driver1.init_motor(mode);
//     driver1.enable_motor();
//     driver2.init_motor(mode);
//     driver2.enable_motor();
//     leg_pos[0] = leg_origin_pos[0];
//     leg_pos[1] = leg_origin_pos[1];
//   }
// }

// void tire_controll() {
//   motor_out[0] = (-2*PS4.LStickY() + 1*PS4.RStickX()) * tire_current_max / (3 * joystick_max);
//   motor_out[1] = (2*PS4.LStickY() + 1*PS4.RStickX()) * tire_current_max / (3 * joystick_max);

//   if(-2000 < motor_out[0] && motor_out[0] < 2000){
//     motor_out[0] = 0;
//   }
//     if(-2000 < motor_out[1] && motor_out[1] < 2000){
//     motor_out[1] = 0;
//   }
// }

// void leg_controll(){
//   bool current_button = PS4.Touchpad();

// //両足の原点と直立
//   if (current_button && !prev_Touchpad) {
//     driver1.set_limit_speed(slow_speed);
//     driver2.set_limit_speed(slow_speed);
//     flag = !flag;
//     if (flag) {
//       leg_pos[0] =  leg_origin_pos[0] - 2.25; //もともと2.25
//       leg_pos[1] =  leg_origin_pos[1] + 2.25;
//     } else {
//       leg_pos[0] =  leg_origin_pos[0];
//       leg_pos[1] =  leg_origin_pos[1];
//     }
//     driver1.set_limit_speed(init_speed);
//     driver2.set_limit_speed(init_speed);
//   }
//   prev_Touchpad = current_button;
// }

// void motor_value_zero(){
//   motor_out[0] = 0;
//   motor_out[1] = 0;
// }

// void DJI_CAN_send(){
//   int16_t r_currentValue = (int16_t)motor_out[0]; // 制御したい電流値 (-16384 〜 16384 の範囲)
//   data[0] = (r_currentValue >> 8) & 0xFF; // 上位バイト
//   data[1] = r_currentValue & 0xFF;        // 下位バイト
//   int16_t l_currentValue = (int16_t)motor_out[1]; // 制御したい電流値 (-16384 〜 16384 の範囲)
//   data[2] = (l_currentValue >> 8) & 0xFF; // 上位バイト
//   data[3] = l_currentValue & 0xFF;        // 下位バイト

//   byte sndStat = CAN0.sendMsgBuf(0x200, 0, 8, data);
// }

// void self_balance_controll(){
//   if(flag){
//     elapsed_time = millis();
//     dt = elapsed_time - pre_time;

//     float error = target_angle - roll;
//     integral += error * dt;
//     float deriv = (error - pre_error);
//     float output = kp * error + ki * integral + kd * deriv;

//     pre_error = error;
//     pre_time = elapsed_time;

//     if(output >= 1.0){
//       output = 1.0;
//     }else if(output <= -1.0){
//       output = -1.0;
//     }

//     motor_out[0] = - output * 16000;
//     motor_out[1] =   output * 16000;
//   }
// }

// void gain_adjust(){
//   bool current_button[6] = {
//     PS4.Triangle(),
//     PS4.Cross(),
//     PS4.Up(),      
//     PS4.Down(),
//     PS4.R1(),
//     PS4.L1(),
//   };

// //kpの微調整
//   if(current_button[0] && !prev_Triangle){
//     kp += inc_kp; //右足下降
//   }else if(current_button[1] && !prev_Cross){
//     kp -= inc_kp; //右足上昇
//   }
//   prev_Triangle = current_button[0];
//   prev_Cross = current_button[1];

// //kiの微調整
//   if(current_button[2] && !prev_Up){
//     ki += inc_ki; //左足下降
//   }else if(current_button[3] && !prev_Down){
//     ki -= inc_ki; //左足上昇
//   }
//   prev_Up = current_button[2];
//   prev_Down = current_button[3];

//   //kdの微調整
//   if(current_button[4] && !prev_R1){
//     kd += inc_kd; //左足下降
//   }else if(current_button[5] && !prev_L1){
//     kd -= inc_kd; //左足上昇
//   }
//   prev_R1 = current_button[4];
//   prev_L1 = current_button[5];

//   //ゲイン
//   if(PS4.R2()){
//     inc_kp = 0.0001;
//     inc_ki = 0.000001;
//     inc_kd = 0.0001;
//   }else{
//     inc_kp = 0.001;
//     inc_ki = 0.00001;
//     inc_kd = 0.001;
//   }
// }

// void Cybergear_CAN_send(){
//   if(mode == MODE_CURRENT){
//     driver1.set_current_ref(0.1f);
//     driver1.set_current_ref(0.0f);
//     driver2.set_current_ref(0.1f);
//     driver2.set_current_ref(0.0f);
//   }else if(mode == MODE_POSITION){
//     driver1.set_position_ref(leg_pos[0]);
//     driver2.set_position_ref(leg_pos[1]);
//   }
// }

// void DJI_CAN_read(){
//   if (CAN0.checkReceive() == CAN_MSGAVAIL) {
//     CAN0.readMsgBuf(&rxId, &len, rxBuf);
//     if (rxId == 0x201) {
//       int16_t angle_raw = ((rxBuf[0] << 8) | rxBuf[1]); // 上位と下位バイトを結合

//       //エンコーダの値から回転総量の計算
//       if(flag){
//         tire_pre_angle = angle_raw;
//         flag = !flag;
//       }

//       if(angle_raw - tire_pre_angle < -4000){
//         tire_angle += ((encoder_max - tire_pre_angle) + angle_raw) / motor_gear_ratio;
//       }else if(angle_raw - tire_pre_angle > 4000){
//         tire_angle -= ((tire_pre_angle - encoder_min) + (encoder_max - angle_raw)) / motor_gear_ratio;
//       }else{
//         tire_angle += (angle_raw - tire_pre_angle) / motor_gear_ratio;
//       }

//       tire_pre_angle = angle_raw;

//       Serial.println(angle_raw);
//     }
//   }
// }

// void display_update(){
//   M5.Lcd.setCursor(0, 3);

//   M5.Lcd.printf("\tkp:%14f\n",kp);
//   M5.Lcd.printf("\tkd:%14f\n",kd);
//   M5.Lcd.printf("\tki:%14f\n",ki);

//   M5.Lcd.printf("\troll:%12.3f\n",roll);
//   M5.Lcd.printf("\tR_output:%6d\n",motor_out[0]);
//   M5.Lcd.printf("\tL_output:%6d\n",motor_out[1]);
// }