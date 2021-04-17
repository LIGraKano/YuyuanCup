void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}
int m;
char z;
void loop() {
  // put your main code here, to run repeatedly:
//   c = 0;
//   while(Serial1.available())
//   {
//     delay(10);
//     z = Serial1.read();
//     c = c * 10 + z - '0'; 
//   }
//   Serial.println(c);
//   Serial.println("sev");
    if(Serial1.available())
    {
      m=0;
      while(Serial1.available())
      {
        delay(10);
        z = Serial1.read();
        m = m*10 + z - '0';
      }
      Serial.println(m);
      Serial.println("sev");
      delay(20);
      }
}
