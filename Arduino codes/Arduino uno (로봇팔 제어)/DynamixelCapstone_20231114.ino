#include <stdio.h>
#include <stdlib.h>
#define TX_Enable_pin (12)  // 송신 활성화 핀
#define RX_Enable_pin (13)  // 수신 활성화 핀
#define DX_ID1        (101)   // 서보모터 아이디
#define DX_ID2        (102)   // 서보모터 아이디
#define DX_ID3        (103)

  unsigned char ref_1[2];
  unsigned char ref_2[2];
  unsigned char wheel_speed[2];
  unsigned char wheel_speed_stop[2];
  unsigned char wheel_reverse_speed[2];
  unsigned char wheel_reverse_stop[2];
  char cmd;
  
typedef enum _com_mode {
  RX_MODE,
  TX_MODE
} com_mode;


// 통신 모드(송/수신)에 따른 버퍼칩 설정
void set_com_mode(com_mode mode) {

  if (mode == RX_MODE)
  {
    // 비활성화 먼저 수행하여 동시에 활성화 되는 순간을 방지
    digitalWrite(TX_Enable_pin, LOW); // TX disable
    digitalWrite(RX_Enable_pin, HIGH); // RX Enable
  }
  else
  {
    // 비활성화 먼저 수행하여 동시에 활성화 되는 순간을 방지
    digitalWrite(RX_Enable_pin, LOW); // RX disable
    digitalWrite(TX_Enable_pin, HIGH); // TX Enable
  }
}


// 통신 프로토콜에 체크섬 삽입
void dx_insert_checksum_byte(unsigned char *packet) {

  unsigned char i;
  unsigned char checksum_pt;
  unsigned char checksum;
  unsigned char packet_length;

  packet_length = packet[3];  // 3번 바이트에 패킷 길이가 저장되어 있음
  checksum_pt = packet_length + 3;

  checksum = 0x00;
  for (i = 2; i < checksum_pt; i++) {
    checksum += packet[i];
  }
  packet[checksum_pt] = ~checksum;

}


// id 설정을 위한 통신 패킷 조립
void dx_set_id(unsigned char id) {

  unsigned char packet[8];
  unsigned char i;

  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = 0xFE;
  packet[3] = 0x04;
  packet[4] = 0x03;
  packet[5] = 0x03;
  packet[6] = id;

  dx_insert_checksum_byte(packet);

  set_com_mode(TX_MODE); // 송신 모드로 설정
  for (i = 0; i < 8; i++) {
    Serial.write(packet[i]);
  }

  // 송신완료시까지 블록
  while ( !(UCSR0A & (1 << TXC0)) ) {
    __asm__("nop\n\t"); // no operation
  }

  set_com_mode(RX_MODE); // 수신 모드로 설정

}

// 바퀴모드: 모두 0으로 설정
// 관절모드: 0~360(0xfff)로 설정
void dx_set_control_mode(unsigned char id,
                         unsigned char cw_angle_limit[2],
                         unsigned char ccw_angle_limit[2]) {

  unsigned char packet[11];
  unsigned char i;

  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = 0x07;
  packet[4] = 0x03;
  packet[5] = 0x06;
  packet[6] = cw_angle_limit[0];
  packet[7] = cw_angle_limit[1];
  packet[8] = ccw_angle_limit[0];
  packet[9] = ccw_angle_limit[1];
  dx_insert_checksum_byte(packet);

  set_com_mode(TX_MODE); // 송신 모드로 설정
  for (i = 0; i < 11; i++) {
    Serial.write(packet[i]);
  }

  // 송신완료시까지 블록
  while ( !(UCSR0A & (1 << TXC0)) ) {
    __asm__("nop\n\t"); // no operation
  }

  set_com_mode(RX_MODE); // 수신 모드로 설정

}


//모터 속도 조절
void dx_set_speed(unsigned char id, unsigned char moving_speed[2]) {

  unsigned char packet[9];
  unsigned char i;

  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = 0x05;
  packet[4] = 0x03;
  packet[5] = 0x20;
  packet[6] = moving_speed[0];
  packet[7] = moving_speed[1];
  dx_insert_checksum_byte(packet);

  set_com_mode(TX_MODE); // 송신 모드로 설정
  for (i = 0; i < 9; i++) {
    Serial.write(packet[i]);
  }

  // 송신완료시까지 블록
  while ( !(UCSR0A & (1 << TXC0)) ) {
    __asm__("nop\n\t"); // no operation
  }

  set_com_mode(RX_MODE); // 수신 모드로 설정

}

// 위치제어용 패킷 조립
void dx_tx_packet_for_position_control(unsigned char id, unsigned char goal_pos[2]) {
  
  unsigned char packet[9];
  unsigned char i;

  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = 0x05;
  packet[4] = 0x03;
  packet[5] = 0x1E;
  packet[6] = goal_pos[0];
  packet[7] = goal_pos[1];
  dx_insert_checksum_byte(packet);

  set_com_mode(TX_MODE); // 송신 모드로 설정
  for (i = 0; i < 9; i++) {
    Serial.write(packet[i]);
  }

  // 송신완료시까지 블록
  while ( !(UCSR0A & (1 << TXC0)) ) {
    __asm__("nop\n\t"); // no operation
  }
  
  set_com_mode(RX_MODE); // 수신 모드로 설정
}


void setup() {
  
  Serial.begin(1000000);  // 통신 속도
  pinMode(TX_Enable_pin, OUTPUT); //TX Enable
  pinMode(RX_Enable_pin, OUTPUT); //RX Enable

  /**********필요할 때 ID 설정*******************/
  //dx_set_id(DX_ID1);
  //delay(1);

  /****************바퀴모드**********************/
  unsigned char cw_angle_limit_3[2] = {0x00, 0x00};
  unsigned char ccw_angle_limit_3[2] = {0x00, 0x00};
  dx_set_control_mode(DX_ID3, cw_angle_limit_3, ccw_angle_limit_3);

  wheel_speed[0] = 0xFF;
  wheel_speed[1] = 0x07;
  
  wheel_speed_stop[0] = 0x00;
  wheel_speed_stop[1] = 0x00;

  wheel_reverse_speed[0]= 0xFF;
  wheel_reverse_speed[1]= 0x03;

  wheel_reverse_stop[0]= 0x00;
  wheel_reverse_stop[1]= 0x04;
  
  /****************관절모드**********************/
  unsigned char cw_angle_limit_1[2] = {0x00, 0x00};
  unsigned char ccw_angle_limit_1[2] = {0xFF, 0x0F};
  dx_set_control_mode(DX_ID1, cw_angle_limit_1, ccw_angle_limit_1);
  
  unsigned char cw_angle_limit_2[2] = {0x00, 0x00};
  unsigned char ccw_angle_limit_2[2] = {0xFF, 0x03};
  dx_set_control_mode(DX_ID2, cw_angle_limit_2, ccw_angle_limit_2);

  delay(1);
  
  //모터 속도 set//
  unsigned char speed_id_1[2] = {0x60, 0x00};
  unsigned char speed_id_2[2] = {0x60, 0x00};
  
  dx_set_speed(101,speed_id_1);
  delay(1);
  dx_set_speed(102,speed_id_2);
  
  //모터 초기 각도 set//
  ref_1[0] = 0x00; // ID 101의 초기값 각도  180도
  ref_1[1] = 0x08;
  dx_tx_packet_for_position_control(101, ref_1);

  ref_2[0] = 0x30; // ID 102의 목표 각도 240도 
  ref_2[1] = 0x03;
  dx_tx_packet_for_position_control(102, ref_2);
  
 
}


void loop() {

if(Serial.available()){
  cmd = Serial.read();

  if(cmd == 'r'){
   Serial.println("rr");
   delay(2000);
   
//////////////쓰레기 수거//////////////////
ref_1[0] = 0x9D; // ID 101의 목표 각도 263
ref_1[1] = 0x0B;
dx_tx_packet_for_position_control(101, ref_1);

//ref_2[0] = 0x30; // ID 102의 목표 각도 
//ref_2[1] = 0x03;
//dx_tx_packet_for_position_control(102, ref_2);
delay(4000);

//ref_1[0] = 0xEC; // ID 101의 목표 각도 20 step:1
//ref_1[1] = 0x02;
//dx_tx_packet_for_position_control(101, ref_1);

/**바퀴모드로 쓰레기 잡기**/
dx_set_speed(DX_ID3, wheel_reverse_speed);
delay(6200);
dx_set_speed(DX_ID3, wheel_speed_stop);

//////////쓰레기 버리는 작업//////////////
ref_1[0] = 0x00; // ID 101의 목표 각도 180
ref_1[1] = 0x08;
dx_tx_packet_for_position_control(101, ref_1);
delay(2000);

ref_2[0] = 0xAA; // ID 102의 목표 각도 50
ref_2[1] = 0x00;
dx_tx_packet_for_position_control(102, ref_2);
delay(5000);

///////////쓰레기 투하////////////////////
dx_set_speed(DX_ID3, wheel_speed);
delay(6200);
dx_set_speed(DX_ID3, wheel_speed_stop);
delay(2000);

///////////복귀 (setup 동일)////////////////
ref_1[0] = 0x00; // ID 101의 초기값 각도
ref_1[1] = 0x08;
//dx_tx_packet_for_position_control(101, ref_1);
//delay(3000);

ref_2[0] = 0x30; // ID 102의 목표 각도
ref_2[1] = 0x03;
dx_tx_packet_for_position_control(102, ref_2);
delay(5000);

dx_set_speed(DX_ID3, wheel_reverse_speed);
delay(400);
dx_set_speed(DX_ID3, wheel_speed_stop);
  }
}
}
