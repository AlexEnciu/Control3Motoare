#include "AccelStepper.h"

// Motor X
#define enablePinX !PD7
#define dirPinX PF1
#define stepPinX PF0
AccelStepper motor_X(AccelStepper::DRIVER, stepPinX, dirPinX, enablePinX);

// Motor Y
#define enablePinY !PF2
#define dirPinY PF7
#define stepPinY PF6
AccelStepper motor_Y(AccelStepper::DRIVER, stepPinY, dirPinY, enablePinY);

// Motor Z
#define enablePinZ !PK0
#define dirPinZ PL1
#define stepPinZ PL3
AccelStepper motor_Z(AccelStepper::DRIVER, stepPinZ, dirPinZ, enablePinZ);

String command = "";

void setup() {
  pinMode(enablePinX, OUTPUT);
  digitalWrite(enablePinX, LOW);

  pinMode(enablePinY, OUTPUT);
  digitalWrite(enablePinY, LOW);

  pinMode(enablePinZ, OUTPUT);
  digitalWrite(enablePinZ, LOW);

  motor_X.setAcceleration(200);
  motor_X.setMaxSpeed(500);

  motor_Y.setAcceleration(200);
  motor_Y.setMaxSpeed(500);

  motor_Z.setAcceleration(200);
  motor_Z.setMaxSpeed(500);

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
