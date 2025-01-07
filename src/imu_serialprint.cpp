// #include <M5Stack.h>
// #include <PS4Controller.h>
// #include <mcp_can.h>
// #include "cybergear_driver.hh"
// #include "cybergear_can_interface_mcp.hh"

// #define SAMPLE_PERIOD 0  // サンプリング間隔(micro秒)
// #define SAMPLE_SIZE 1500 //]

// // canid
// uint8_t MASTER_CAN_ID = 0x00;
// uint8_t Rleg_CAN_ID = 0x7F;
// uint8_t Lleg_CAN_ID = 0x7E;

// // cybergeardriver初期化
// CybergearDriver driver1 = CybergearDriver(MASTER_CAN_ID, Rleg_CAN_ID);
// MotorStatus motor_status1;
// CybergearDriver driver2 = CybergearDriver(MASTER_CAN_ID, Lleg_CAN_ID);
// MotorStatus motor_status2;
// CybergearCanInterfaceMcp interface;

// float leg_pos[2] = {0.0f, 0.0f}; // target_pos[0]=Rleg, target_pos[1]=Lleg
// float leg_origin_pos[2];
// uint8_t mode = MODE_CURRENT;
// float default_kp = 50.0f;
// float default_kd = 1.0f;
// float init_speed = 5.0f;
// float slow_speed = 1.0f;

// bool prev_Touchpad = false;
// bool flag = false;

// void init_cybergear();
// void leg_controll();
// void leg_caliblation();
// void Cybergear_CAN_send();

// // レジスタ書き込み用関数　ヘッダファイル変更できる方は mpu6886.hの同じ関数をpublic化して使いましょう。
// void I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer)
// {
//     Wire1.beginTransmission(driver_Addr);
//     Wire1.write(start_Addr);
//     Wire1.write(*write_Buffer);
//     Wire1.endTransmission();
// }

// // レジスタ読み込み用関数　ヘッダファイル変更できる方は mpu6886.hの同じ関数をpublic化して使いましょう。
// void I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer)
// {
//     Wire1.beginTransmission(driver_Addr);
//     Wire1.write(start_Addr);
//     Wire1.endTransmission(false);
//     uint8_t i = 0;
//     Wire1.requestFrom(driver_Addr, number_Bytes);

//     //! Put read results in the Rx buffer
//     while (Wire1.available())
//     {
//         read_Buffer[i++] = Wire1.read();
//     }
// }

// void setup()
// {
//     M5.begin();
//     M5.Lcd.setRotation(3);

//     PS4.begin("3C:61:05:0D:E5:DE");
//     init_cybergear();

//     M5.MPU6886.Init();      // MPU6886を初期設定する
//     Wire1.setClock(500000); // オンボードデバイスのI2C通信速度 デフォルト100k、Fast mode 400kまでが規定値。

//     unsigned char buf;
//     unsigned char regdata;

//     regdata = 0x00;                                       // 次行で設定するレジストリ設定値
//     I2C_Write_NBytes(MPU6886_ADDRESS, 0x19, 1, &regdata); // SAMPLE LATE DIVIDER,デフォルトはregdata = 0x05

//     regdata = 0x07;                                       // 次行で設定するレジストリ設定値
//     I2C_Write_NBytes(MPU6886_ADDRESS, 0x1A, 1, &regdata); // CONFIG,デフォルトは 0x01

//     regdata = 0x18; // 次行で設定するレジストリ設定値
//     //    I2C_Write_NBytes(MPU6886_ADDRESS,0x1B,1,&regdata);      //GYRO_CONFIG,デフォルトは 0x18

//     regdata = 0x10; // 次行で設定するレジストリ設定値
//     //    I2C_Write_NBytes(MPU6886_ADDRESS,0x1C,1,&regdata);      //ACCEL_CONFIG,デフォルトは 0x10

//     regdata = 0x08;                                       // 次行で設定するレジストリ設定値
//     I2C_Write_NBytes(MPU6886_ADDRESS, 0x1D, 1, &regdata); // ACCEL_CONFIG_2,デフォルトは0x00

//     regdata = 0x07; // 次行で設定するレジストリ設定値
//     //    I2C_Write_NBytes(MPU6886_ADDRESS,0x6C,1,&regdata);      //PWR_MANEGEMENT_2,デフォルトは 0x00

//     I2C_Read_NBytes(MPU6886_ADDRESS, 0x19, 1, &buf);
//     Serial.print("0x19 value = ");
//     Serial.println(buf);

//     I2C_Read_NBytes(MPU6886_ADDRESS, 0x1A, 1, &buf);
//     Serial.print("0x1A value = ");
//     Serial.println(buf);

//     I2C_Read_NBytes(MPU6886_ADDRESS, 0x1B, 1, &buf);
//     Serial.print("0x1B value = ");
//     Serial.println(buf);

//     I2C_Read_NBytes(MPU6886_ADDRESS, 0x1C, 1, &buf);
//     Serial.print("0x1C value = ");
//     Serial.println(buf);

//     I2C_Read_NBytes(MPU6886_ADDRESS, 0x1D, 1, &buf);
//     Serial.print("0x1D value = ");
//     Serial.println(buf);

//     I2C_Read_NBytes(MPU6886_ADDRESS, 0x6C, 1, &buf);
//     Serial.print("0x6C value = ");
//     Serial.println(buf);
// }

// float ax[SAMPLE_SIZE], ay[SAMPLE_SIZE], az[SAMPLE_SIZE]; // 加速度データを読み出す変数

// #define X0 5              // 横軸の描画開始座標
// #define MINZ 0            // 縦軸の最小値
// #define MAXZ 2000         // 縦軸の最大値
// long t;                   // サンプリング周期測定用
// long deltaT[SAMPLE_SIZE]; // サンプリング周期記録用

// void loop()
// {
//     M5.update();

//     if (PS4.isConnected())
//     {
//         if (PS4.Share())
//         {
//             init_cybergear();
//         }
//         leg_caliblation();
//         leg_controll();
//     }

//     if(PS4.R1()){
//         M5.Lcd.fillScreen(BLACK);
//     }

//     if (PS4.Circle())
//     {
//         Serial.println("start");
//         M5.Lcd.fillScreen(BLACK); // 画面をクリア
//         // delay(3000);
//         for (int i = 0; i < SAMPLE_SIZE; i++)
//         {
//             M5.MPU6886.getAccelData(&ax[i], &ay[i], &az[i]); // MPU6886から加速度を取得
//             ay[i] *= 1000;                                   // mGに変換
//             if (i > 1)
//             {
//                 int y0 = map((int)(ay[i - 1]), MINZ, MAXZ, M5.Lcd.height(), 0);
//                 int y1 = map((int)(ay[i]), MINZ, MAXZ, M5.Lcd.height(), 0);
//                 M5.Lcd.drawLine(i - 1 + X0, y0, i + X0, y1, GREEN);
//             }

//             deltaT[i] = micros() - t;
//             if (i == 0)
//                 deltaT[i] = 0;
//             t = micros();
//             delayMicroseconds(SAMPLE_PERIOD);
//         }

//         // Serial.printでのデータ確認をしたいが、通信に時間が食われてサンプルリングレートが落ちるので
//         // SANPLE_SIZE個のデータを採集後に、まとめてシリアル出力してサンプリングレートとデータを確認
//         // 不要な場合は以下のforループをコメントアウトで測定速度アップ

//         for (int i = 0; i < SAMPLE_SIZE; i++)
//         {
//             // Serial.print("測定周期(μsec)：");

//             // if(ay[i] > 0 || ay[i] < -1000){
//             //     Serial.println("get");
//             // }

//             Serial.print(deltaT[i]);
//             Serial.print(" , ");
//             Serial.print(ay[i]);
//             Serial.print(" , ");
//             Serial.println(i);
//         }
//         Serial.println("finish");
//     }else{
//         if ( driver1.process_packet() ) {
//         motor_status1 = driver1.get_motor_status();
//          }
//         auto now = millis();
//         static auto pre = now;
//         if(now - pre > 3){
//             Cybergear_CAN_send();
//             pre = now;
//         }
//     }
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

// void leg_controll(){
//   bool current_button = PS4.Touchpad();

// //両足の原点と直立
//   if (current_button && !prev_Touchpad) {
//     driver1.set_limit_speed(slow_speed);
//     driver2.set_limit_speed(slow_speed);
//     flag = !flag;
//     if (flag) {
//       leg_pos[0] =  leg_origin_pos[0] - 2.25;
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