int analogPin_0 = 0;     // potentiometer wiper (middle terminal) connected to analog pin 3
                       // outside leads to ground and +5V
int analogPin_1 = 1;     // potentiometer wiper (middle terminal) connected to analog pin 3

int val_0 = 0;           // variable to store the value read
int val_1 = 0;           // variable to store the value read

void setup()
{
  Serial.begin(9600);              //  setup serial
}

void loop()
{
  val_0 = analogRead(analogPin_0);     // read the input pin
  val_1 = analogRead(analogPin_1);     // read the input pin
  Serial.println(-1000);
  //Serial.print("Presion: ");
  Serial.println(val_1*5.0/1024.0);             // debug value
  //Serial.print("Luminosidad: ");
  Serial.println(val_0*5.0/1024.0);             // debug value

  delay(1000);
}

