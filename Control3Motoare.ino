#include "AccelStepper.h"

// Motor X
#define enablePinX 38
#define dirPinX A1
#define stepPinX A0
AccelStepper motor_X(AccelStepper::DRIVER, stepPinX, dirPinX, enablePinX);

// Motor Y
#define enablePinY A2
#define dirPinY A7
#define stepPinY A4
AccelStepper motor_Y(AccelStepper::DRIVER, stepPinY, dirPinY, enablePinY);

// Motor Z
#define enablePinZ A8
#define dirPinZ 48
#define stepPinZ 46
AccelStepper motor_Z(AccelStepper::DRIVER, stepPinZ, dirPinZ, enablePinZ);

// Motor R1
#define enablePinR1 24
#define dirPinR1 28
#define stepPinR1 26
AccelStepper motor_R1(AccelStepper::DRIVER, stepPinR1, dirPinR1, enablePinR1);

// Motor R2
#define enablePinR2 30
#define dirPinR2 34
#define stepPinR2 36
AccelStepper motor_R2(AccelStepper::DRIVER, stepPinR2, dirPinR2, enablePinR2);

String command = "";

void setup() {
  pinMode(enablePinX, OUTPUT);
  digitalWrite(enablePinX, HIGH);

  pinMode(enablePinY, OUTPUT);
  digitalWrite(enablePinY, HIGH);

  pinMode(enablePinZ, OUTPUT);
  digitalWrite(enablePinZ, HIGH);
  
  pinMode(enablePinR1, OUTPUT);
  digitalWrite(enablePinR1, HIGH);

  pinMode(enablePinR2, OUTPUT);
  digitalWrite(enablePinR2, HIGH);

  motor_X.setAcceleration(200);
  motor_X.setMaxSpeed(500);

  motor_Y.setAcceleration(200);
  motor_Y.setMaxSpeed(500);

  motor_Z.setAcceleration(200);
  motor_Z.setMaxSpeed(500);

  motor_R1.setAcceleration(200);
  motor_R1.setMaxSpeed(500);

  motor_R2.setAcceleration(200);
  motor_R2.setMaxSpeed(500);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');

    // Motor X commands
    if (command.startsWith("readParamsX")) {
      Serial.print("MotorX_Status=");
      Serial.println(digitalRead(enablePinX) == HIGH ? "DISABLED" : "ENABLE");
      Serial.print("AccelerationX=");
      Serial.println(motor_X.acceleration());
      Serial.print("MaxSpeedX=");
      Serial.println(motor_X.maxSpeed());
      Serial.print("CurrentPositionX=");
      Serial.println(motor_X.currentPosition());
    } else if (command.startsWith("setAccelerationX")) {
      motor_X.setAcceleration(parseValue(command));
      Serial.print("AccelerationValueSetToX=");
      Serial.println(motor_X.acceleration());
    } else if (command.startsWith("setMaxSpeedX")) {
      motor_X.setMaxSpeed(parseValue(command));
      Serial.print("MaxSpeedValueSetToX=");
      Serial.println(motor_X.maxSpeed());
    } else if (command.startsWith("MoveToX")) {
      motor_X.moveTo(parseValue(command));
      motor_X.runToPosition();
    } else if (command.startsWith("motorEnableX")) {
      digitalWrite(enablePinX, LOW);
      Serial.println("MotorX_ENABLED");
    } else if (command.startsWith("motorDisableX")) {
      digitalWrite(enablePinX, HIGH);
      Serial.println("MotorX_DISABLED");
    } else if (command.startsWith("MoveX")) {
      motor_X.move(parseValue(command));
      motor_X.runToPosition();
    } else if (command.startsWith("stopX")) {
      motor_X.stop();
    }

    // Motor Y commands
    else if (command.startsWith("readParamsY")) {
      Serial.print("MotorY_Status=");
      Serial.println(digitalRead(enablePinY) == HIGH ? "DISABLED" : "ENABLE");
      Serial.print("AccelerationY=");
      Serial.println(motor_Y.acceleration());
      Serial.print("MaxSpeedY=");
      Serial.println(motor_Y.maxSpeed());
      Serial.print("CurrentPositionY=");
      Serial.println(motor_Y.currentPosition());
    } else if (command.startsWith("setAccelerationY")) {
      motor_Y.setAcceleration(parseValue(command));
      Serial.print("AccelerationValueSetToY=");
      Serial.println(motor_Y.acceleration());
    } else if (command.startsWith("setMaxSpeedY")) {
      motor_Y.setMaxSpeed(parseValue(command));
      Serial.print("MaxSpeedValueSetToY=");
      Serial.println(motor_Y.maxSpeed());
    } else if (command.startsWith("MoveToY")) {
      motor_Y.moveTo(parseValue(command));
      motor_Y.runToPosition();
    } else if (command.startsWith("motorEnableY")) {
      digitalWrite(enablePinY, LOW);
      Serial.println("MotorY_ENABLED");
    } else if (command.startsWith("motorDisableY")) {
      digitalWrite(enablePinY, HIGH);
      Serial.println("MotorY_DISABLED");
    } else if (command.startsWith("MoveY")) {
      motor_Y.move(parseValue(command));
      motor_Y.runToPosition();
    } else if (command.startsWith("stopY")) {
      motor_Y.stop();
    }

    // Motor Z commands
    else if (command.startsWith("readParamsZ")) {
      Serial.print("MotorZ_Status=");
      Serial.println(digitalRead(enablePinZ) == HIGH ? "DISABLED" : "ENABLE");
      Serial.print("AccelerationZ=");
      Serial.println(motor_Z.acceleration());
      Serial.print("MaxSpeedZ=");
      Serial.println(motor_Z.maxSpeed());
      Serial.print("CurrentPositionZ=");
      Serial.println(motor_Z.currentPosition());
    } else if (command.startsWith("setAccelerationZ")) {
      motor_Z.setAcceleration(parseValue(command));
      Serial.print("AccelerationValueSetToZ=");
      Serial.println(motor_Z.acceleration());
    } else if (command.startsWith("setMaxSpeedZ")) {
      motor_Z.setMaxSpeed(parseValue(command));
      Serial.print("MaxSpeedValueSetToZ=");
      Serial.println(motor_Z.maxSpeed());
    } else if (command.startsWith("MoveToZ")) {
      motor_Z.moveTo(parseValue(command));
      motor_Z.runToPosition();
    } else if (command.startsWith("motorEnableZ")) {
      digitalWrite(enablePinZ, LOW);
      Serial.println("MotorZ_ENABLED");
    } else if (command.startsWith("motorDisableZ")) {
      digitalWrite(enablePinZ, HIGH);
      Serial.println("MotorZ_DISABLED");
    } else if (command.startsWith("MoveZ")) {
      motor_Z.move(parseValue(command));
      motor_Z.runToPosition();
    } else if (command.startsWith("stopZ")) {
      motor_Z.stop();
    }
 // }
//}

    // Motor R1 commands
      else if (command.startsWith("readParamsR1")) {
      Serial.print("MotorR1_Status=");
      Serial.println(digitalRead(enablePinR1) == HIGH ? "DISABLED" : "ENABLE");
      Serial.print("AccelerationR1=");
      Serial.println(motor_R1.acceleration());
      Serial.print("MaxSpeedR1=");
      Serial.println(motor_R1.maxSpeed());
      Serial.print("CurrentPositionR1=");
      Serial.println(motor_R1.currentPosition());
    } else if (command.startsWith("setAccelerationR1")) {
      motor_R1.setAcceleration(parseValue(command));
      Serial.print("AccelerationValueSetToR1=");
      Serial.println(motor_R1.acceleration());
    } else if (command.startsWith("setMaxSpeedR1")) {
      motor_R1.setMaxSpeed(parseValue(command));
      Serial.print("MaxSpeedValueSetToR1=");
      Serial.println(motor_R1.maxSpeed());
    } else if (command.startsWith("MoveToR1")) {
      motor_R1.moveTo(parseValue(command));
      motor_R1.runToPosition();
    } else if (command.startsWith("motorEnableR1")) {
      digitalWrite(enablePinR1, LOW);
      Serial.println("MotorR1_ENABLED");
    } else if (command.startsWith("motorDisableR1")) {
      digitalWrite(enablePinR1, HIGH);
      Serial.println("MotorR1_DISABLED");
    } else if (command.startsWith("MoveR1")) {
      motor_R1.move(parseValue(command));
      motor_R1.runToPosition();
    } else if (command.startsWith("stopR1")) {
      motor_R1.stop();
    }

    // Motor R2 commands
    else if (command.startsWith("readParamsR2")) {
      Serial.print("MotorR2_Status=");
      Serial.println(digitalRead(enablePinR2) == HIGH ? "DISABLED" : "ENABLE");
      Serial.print("AccelerationR2=");
      Serial.println(motor_R2.acceleration());
      Serial.print("MaxSpeedR2=");
      Serial.println(motor_R2.maxSpeed());
      Serial.print("CurrentPositionR2=");
      Serial.println(motor_R2.currentPosition());
    } else if (command.startsWith("setAccelerationR2")) {
      motor_R2.setAcceleration(parseValue(command));
      Serial.print("AccelerationValueSetToR2=");
      Serial.println(motor_R2.acceleration());
    } else if (command.startsWith("setMaxSpeedR2")) {
      motor_R2.setMaxSpeed(parseValue(command));
      Serial.print("MaxSpeedValueSetToR2=");
      Serial.println(motor_R2.maxSpeed());
    } else if (command.startsWith("MoveToR2")) {
      motor_R2.moveTo(parseValue(command));
      motor_R2.runToPosition();
    } else if (command.startsWith("motorEnableR2")) {
      digitalWrite(enablePinR2, LOW);
      Serial.println("MotorR2_ENABLED");
    } else if (command.startsWith("motorDisableR2")) {
      digitalWrite(enablePinR2, HIGH);
      Serial.println("MotorR2_DISABLED");
    } else if (command.startsWith("MoveR2")) {
      motor_R2.move(parseValue(command));
      motor_R2.runToPosition();
    } else if (command.startsWith("stopR2")) {
      motor_R2.stop();
    }
  }
}

// Serial STRING value parsing
int parseValue(String input) {
  int spaceIndex = input.indexOf(' ');
  if (spaceIndex != -1) {
    String valueStr = input.substring(spaceIndex + 1);
    return valueStr.toInt();
  }
  return 0;
}
