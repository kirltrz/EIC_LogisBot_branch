/*
RegWritePos示例通过了ST3215/ST3020/ST3025伺服系统的测试， 
如果测试其他型号的ST系列伺服系统
请更改适当的位置，速度和延迟参数。
*/

#include <SCServo.h>

SMS_STS st;

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19


void setup()
{
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);
}

void loop()
{
  st.RegWritePosEx(1, 4095, 3400, 50);//servo(ID1) speed=3400，acc=50，move to position=4095.
  st.RegWritePosEx(2, 4095, 3400, 50);//servo(ID2) speed=3400，acc=50，move to position=4095.
  st.RegWriteAction();
  delay(3000);

  st.RegWritePosEx(1, 0, 3400, 50);//servo(ID1) speed=3400，acc=50，move to position=0.
  st.RegWritePosEx(2, 0, 3400, 50);//servo(ID2) speed=3400，acc=50，move to position=0.
  st.RegWriteAction();
  delay(3000);
}
