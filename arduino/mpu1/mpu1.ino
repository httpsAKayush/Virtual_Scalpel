const int flexPin = A0; 
// const int ledPin = 8 ;

void setup() { 
  Serial.begin(115200);
  // pinMode(ledPin,OUTPUT);
} 

void loop(){ 
  int flexValue;
  flexValue = analogRead(flexPin);
  Serial.print("sensor: ");
  Serial.println(flexValue);
 
  // if(flexValue>400)
  //    digitalWrite(ledPin,HIGH);
  // else
  //   digitalWrite(ledPin,LOW);
 
  delay(2000);
}