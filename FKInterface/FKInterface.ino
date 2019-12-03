
bool verPress, horPress;
int upState, downState;
int jointID;

void setup() {
  pinMode(10, INPUT); //Increase Joint Value
  pinMode(11, INPUT); //Decrease Joint Value
  pinMode(13, INPUT); //Change Joint ID (Upward)
  pinMode(12, INPUT); //Change Joint ID (Downward)
  verPress = false;
  horPress = false;
  jointID = 0;
  Serial.begin(9600);
}

void loop() {
  upState = digitalRead(13);
  downState = digitalRead(12);

  if(upState == HIGH && !verPress) {
    verPress = true;
    if (jointID < 4) {
      jointID += 1;
    }
  } else if (downState == HIGH && !verPress) {
    verPress = true;
    if (jointID > 0) {
      jointID -= 1;
    }
  } else if (upState == LOW && downState == LOW && verPress) {
    verPress = false;
  }

  switch(jointID) {
    case 0 :
      Serial.println("HEAD_TILT");
      break;
    case 1 :
      Serial.println("HEAD_PAN");
      break;
    case 2 :
      Serial.println("HIP_YAW");
      break;
    case 3 :
      Serial.println("HIP_ROLL");
      break;
    case 4 :
      Serial.println("HIP_PITCH");
      break;
  }
}
