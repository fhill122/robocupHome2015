/* Serial2_Echo

 */

/*
 Serial1 : Dynamixel_Poart
 Serial2 : Serial_Poart
 TxD(Cm9) <--(Connect)--> RxD(PC)
 RxD(Cm9) <--(Connect)--> TxD(PC)
*/

#define ID_NUM 1
#define MAX_TORQUE 300

#define POSITION_OPEN 550
#define POSITION_CLOSE 800

#define OPEN 0
#define CLOSE 128 //FIX THIS

Dynamixel Dxl(1); //Dynamixel on Serial1(USART1)

void setup(){
  //Serial2 Serial initialize
  Dxl.begin(3);
  Dxl.maxTorque(ID_NUM, MAX_TORQUE);
  Serial2.begin(57600);  
  pinMode(BOARD_LED_PIN, OUTPUT);  //toggleLED_Pin_Out
  
  //disable alarm shutdown
  Dxl.writeWord(ID_NUM, 18, 0);
}
void loop(){
  // when you typed any character in terminal
  if(Serial2.available()){
    //print it out though USART2(RX2,TX2)
    delay(100);
    
    byte input = Serial2.read();
    //unsigned char input = (unsigned char)Serial2.read();
    SerialUSB.println(input);
    Serial2.print(input);
    
    if (input == OPEN) {
      digitalWrite(BOARD_LED_PIN, LOW);  // set to as LOW LED is turn-on
      
      /*ID 1 dynamixel moves to position 0 with velocity 100*/ 
      Dxl.setPosition(1,POSITION_OPEN,100); 
      delay(1000);// it has more delay time for slow movement
      
      
    } else if (input > OPEN) {
      digitalWrite(BOARD_LED_PIN, HIGH); // set to as HIGH LED is turn-off
      
      /*ID 1 dynamixel moves to position 500 with velocity 300*/ 
      Dxl.setPosition(1,POSITION_CLOSE,300); 
      delay(1000);
    }
    //toggleLED();
    
  }
}
