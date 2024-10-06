void encoder_right_func() {
  if (right_count == true) {
    if (digitalRead(right_encoderpinA) > digitalRead(right_encoderpinB))
      right_encodervalue--;
    else
      right_encodervalue++;
  } else {
    if (digitalRead(right_encoderpinA) < digitalRead(right_encoderpinB))
      right_encodervalue--;
    else
      right_encodervalue++;
  }
}

void encoder_left_func() {
  if (left_count == true) {
    if (digitalRead(left_encoderpinA) < digitalRead(left_encoderpinB))
      left_encodervalue--;
    else
      left_encodervalue++;
  } else {
    if (digitalRead(left_encoderpinA) > digitalRead(left_encoderpinB))
      left_encodervalue++;
    else
      left_encodervalue--;
  }
}

void forward() {

  int count = 0;
  last_pos = 0;
  right_count = false;
  left_count = false;
  left_encodervalue = 0;
  right_encodervalue = 0;
  bool left_over = false;
  bool right_over = false;
  Setpoint_right = 813;
  Setpoint_left = 830;
  pid_right.SetMode(AUTOMATIC);
  pid_left.SetMode(AUTOMATIC);
  while (true) {
    if (last_pos == left_encodervalue) {
      count++;
    }
    if (count > 700) {
      break;
    }

    if (left_encodervalue >= Setpoint_left) {
      digitalWrite(left_motor1, LOW);
      digitalWrite(left_motor2, LOW);
      right_over = true;
    }

    if (right_encodervalue >= Setpoint_right) {
      digitalWrite(right_motor1, LOW);
      digitalWrite(right_motor2, LOW);
      left_over = true;
    }
    if (right_over == true && left_over == true) {
      break;
    }
    pid_left.Compute();
    pid_right.Compute();
    Serial.print(" leftencoder--->");
    Serial.print(left_encodervalue);
    Serial.print(" ");
    Serial.print("rightencoder--->");
    Serial.println(right_encodervalue);
    analogWrite(right_motor_pwm, base_speed + right_motor_pwm_pid-10);
    digitalWrite(right_motor1, HIGH);
    digitalWrite(right_motor2, LOW);
    analogWrite(left_motor_pwm, base_speed + left_motor_pwm_pid-10);
    digitalWrite(left_motor1, HIGH);
    digitalWrite(left_motor2, LOW);
    last_pos = left_encodervalue;
  }

  Stop();

  if (current_direction == 0) {
    currentX--;
  } else if (current_direction == 90) {
    currentY++;
  } else if (current_direction == 180) {
    currentX++;
  } else if (current_direction == 270) {
    currentY--;
  }
}

void turn_right() {

  int count = 0;
  last_pos = 0;
  right_count = true;
  left_encodervalue = 0;
  right_encodervalue = 0;
  bool left_over = false;
  bool right_over = false;
  Setpoint_right = 275;
  Setpoint_left = 275;
  pid_right_turn.SetMode(AUTOMATIC);
  pid_left_turn.SetMode(AUTOMATIC);
  while (true) {

    if (last_pos == left_encodervalue) {
      count++;
    }
    if (count > 700) {
      right_count = false;
      break;
    }
    pid_left_turn.Compute();
    pid_right_turn.Compute();
    Serial.print(" ");


    if (left_encodervalue >= Setpoint_left) {
      digitalWrite(left_motor1, LOW);
      digitalWrite(left_motor2, LOW);
      right_over = true;
    }

    if (right_encodervalue >= Setpoint_right) {
      digitalWrite(right_motor1, LOW);
      digitalWrite(right_motor2, LOW);
      left_over = true;
    }
    if (right_over == true && left_over == true) {
      break;
    }
    //Serial.println(count);
    pid_left.Compute();
    pid_right.Compute();
    Serial.print("leftencoder--->");
    Serial.print(left_encodervalue);
    Serial.print("\t");
    Serial.print("rightencoder--->");
    Serial.println(right_encodervalue);
    analogWrite(right_motor_pwm, base_speed + right_motor_pwm_pid - 15);
    digitalWrite(right_motor1, LOW);
    digitalWrite(right_motor2, HIGH);
    analogWrite(left_motor_pwm, base_speed + left_motor_pwm_pid - 15);
    digitalWrite(left_motor1, HIGH);
    digitalWrite(left_motor2, LOW);

    last_pos = left_encodervalue;
  }

  Stop();

  if (current_direction != 270) {
    current_direction += 90;
  }

  else {
    current_direction = 0;
  }
}

void turn_left() {
  int count = 0;
  last_pos = 0;
  left_count = true;
  left_encodervalue = 0;
  right_encodervalue = 0;
  bool left_over = false;
  bool right_over = false;
  Setpoint_right = 252;//252
  Setpoint_left = 252;
  pid_right_turn.SetMode(AUTOMATIC);
  pid_left_turn.SetMode(AUTOMATIC);
  while (true) {

    if (last_pos == left_encodervalue) {
      count++;
    }
    if (count > 700) {
      left_count = false;
      break;
    }
    
    pid_left_turn.Compute();
    pid_right_turn.Compute();
    if (left_encodervalue >= Setpoint_left) {
      digitalWrite(left_motor1, LOW);
      digitalWrite(left_motor2, LOW);
      right_over = true;
    }

    if (right_encodervalue >= Setpoint_right) {
      digitalWrite(right_motor1, LOW);
      digitalWrite(right_motor2, LOW);
      left_over = true;
    }
    if (right_over == true && left_over == true) {
      left_count = false;
      break;
    }
    Serial.print("leftencoder--->");
    Serial.print(left_encodervalue);
    Serial.print("\t");
    Serial.print("rightencoder--->");
    Serial.println(right_encodervalue);
    analogWrite(right_motor_pwm, base_speed + right_motor_pwm_pid - 15);
    digitalWrite(right_motor1, HIGH);
    digitalWrite(right_motor2, LOW);
    analogWrite(left_motor_pwm, base_speed + left_motor_pwm_pid - 20);
    digitalWrite(left_motor1, LOW);
    digitalWrite(left_motor2, HIGH);

    last_pos = left_encodervalue;
    
  }
  Stop();

  if (current_direction != 0) {
    current_direction -= 90;
  }

  else {
    current_direction = 270;
  }
}

void Stop() {
  digitalWrite(right_motor1, LOW);
  digitalWrite(right_motor2, LOW);
  digitalWrite(left_motor1, LOW);
  digitalWrite(left_motor2, LOW);
}
