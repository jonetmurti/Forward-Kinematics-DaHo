
bool verPress, horPress;
int upState, downState;
int rightState, leftState;
int jointID;
int jointOffset;

void setup() {
  pinMode(10, INPUT); //Increase Joint Value
  pinMode(11, INPUT); //Decrease Joint Value
  pinMode(13, INPUT); //Change Joint ID (Upward)
  pinMode(12, INPUT); //Change Joint ID (Downward)
  verPress = false;
  horPress = false;
  jointID = 0;
  jointOffset = 0;
  Serial.begin(9600);
}

void loop() {
  upState = digitalRead(13);
  downState = digitalRead(12);
  rightState = digitalRead(10);
  leftState = digitalRead(11);

  if(upState == HIGH && !verPress) {
    verPress = true;
    if (jointID < 4) {
      jointID += 1;
      jointOffset = 0;
    }
  } else if (downState == HIGH && !verPress) {
    verPress = true;
    if (jointID > 0) {
      jointID -= 1;
      jointOffset = 0;
    }
  } else if (upState == LOW && downState == LOW && verPress) {
    verPress = false;
  }

  if(rightState == HIGH && !horPress) {
    horPress = true;
    if(jointOffset < 72) {
      jointOffset += 1;
    }
  } else if (leftState == HIGH && !horPress) {
    horPress = true;
    if(jointOffset > 0) {
      jointOffset -= 1;
    }
  } else if (rightState == LOW && leftState == LOW && horPress) {
    horPress = false;
  }

  switch(jointID) {
    case 0 :
      Serial.print("HEAD_TILT : ");
      break;
    case 1 :
      Serial.print("HEAD_PAN : ");
      break;
    case 2 :
      Serial.print("HIP_YAW : ");
      break;
    case 3 :
      Serial.print("HIP_ROLL : ");
      break;
    case 4 :
      Serial.print("HIP_PITCH : ");
      break;
  }

  Serial.println(jointOffset);
}
