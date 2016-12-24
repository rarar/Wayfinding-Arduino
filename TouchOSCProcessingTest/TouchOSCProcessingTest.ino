int currentValue = 0;
int values[] = {0, 0, 0, 0};

void setup() {  
  Serial.begin(9600);  //set serial to 9600 baud rate
}

void loop(){
   if (Serial.available()) { //  Check if there is a new message
    int incomingValue = Serial.read();
    values[currentValue] = incomingValue;
    currentValue++;
    if (currentValue > 3) {
      currentValue = 0;
    }
   analogWrite(11, values[0]);
   }   
}

