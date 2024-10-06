#include <ArduinoQueue.h>
#include <PID_v1.h>
#define maze_size 12
int blockno = 144;


int right_motor1 = 14;
int right_motor2 = 12;
int left_motor1 = 27;
int left_motor2 = 26;
int right_encoderpinA = 19;
int right_encoderpinB = 18;
int left_encoderpinA = 35;
int left_encoderpinB = 34;
int right_motor_pwm = 13;
int left_motor_pwm = 25;
int stby = 33;
int base_speed = 60;



int currentX = 11;
int currentY = 11;

int current_direction = 0;

bool right_count = false;
bool left_count = false;
double right_encodervalue;
double left_encodervalue;

int last_pos;

int count;


double Setpoint;
double Setpoint_right;
double Setpoint_left;

double left_motor_pwm_pid;
double right_motor_pwm_pid;


int IRF = 4;
int IRLF = 32;
int IRRF = 5;
//int IRLB = 23;
//int IRRB = 4;



ArduinoQueue<int> element(blockno);
ArduinoQueue<int> elementX(blockno);
ArduinoQueue<int> elementY(blockno);

int current_queue = 0;
int manhattan_distances[maze_size][maze_size] = {
  {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999},
  {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999},
  {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999},
  {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999},
  {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999},
  {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999},
  {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999},
  {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999},
  {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999},
  {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999},
  {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999},
  {999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999, 999}
};


int HorizontalWalls[maze_size + 1][maze_size] = {
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
};

int VerticalWalls[maze_size][maze_size + 1] = {
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
};

bool up, down, left, right;

bool bot_up, bot_down, bot_right, bot_left;


double kp = 0.87, ki = 0, kd = 0.05;
double kp_turn = 0.755359375, ki_turn = 0.3, kd_turn = 0;

PID pid_right(&right_encodervalue, &right_motor_pwm_pid, &Setpoint_right, kp, ki, kd, DIRECT);
PID pid_left(&left_encodervalue, &left_motor_pwm_pid, &Setpoint_left, kp, ki, kd, DIRECT);
PID pid_right_turn(&right_encodervalue, &right_motor_pwm_pid, &Setpoint_right, kp_turn, ki_turn, kd_turn, DIRECT);
PID pid_left_turn(&left_encodervalue, &left_motor_pwm_pid, &Setpoint_left, kp_turn, ki_turn, kd_turn, DIRECT);
 



void setup() {

  Serial.begin(9600);

  pinMode(right_motor1, OUTPUT);
  pinMode(right_motor2, OUTPUT);
  pinMode(left_motor1, OUTPUT);
  pinMode(left_motor2, OUTPUT);
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(right_motor_pwm, OUTPUT);
  pinMode(right_encoderpinA, INPUT);
  pinMode(right_encoderpinB, INPUT);
  pinMode(left_encoderpinA, INPUT);
  pinMode(left_encoderpinA, INPUT);
  pinMode(IRF, INPUT);
  pinMode(IRLF, INPUT);
  pinMode(IRRF, INPUT);

  pinMode(stby, OUTPUT);
  digitalWrite(stby, HIGH);
  attachInterrupt(digitalPinToInterrupt(left_encoderpinA), encoder_left_func, RISING);

  attachInterrupt(digitalPinToInterrupt(right_encoderpinA), encoder_right_func, RISING);

  pid_left.SetOutputLimits(-50, 50); // Motor PWM range
  pid_right.SetOutputLimits(-50,49); // Motor PWM range
  pid_left_turn.SetOutputLimits(-50, 50); // Motor PWM range
  pid_right_turn.SetOutputLimits(-50,50); // Motor PWM range
  
  Serial.begin(9600);
  flood_fill();
  
  delay(3000);
  
  

}

void loop() {
  wall_update(HorizontalWalls, VerticalWalls, currentX, currentY);
  move_next();
  if(manhattan_distances[currentX][currentY]==0){
    currentX=11;
    currentY=0;
    delay(10000);
  }
  delay(100);

}

void move_next() {
  int next_cell = find_direction();
  
  if (next_cell == 0) {
    Serial.println("forward");
    forward();
  }
  else if (next_cell == 2) {
    Serial.println("left");
    turn_left();
    delay(100);
    forward();
  }
  else if (next_cell == 3) {
    Serial.println("right");
    turn_right();
    delay(100);
    forward();
  }
  else if (next_cell == 1) {
    Serial.println("uturn");
    turn_right();
    delay(100);
    turn_right();
    delay(100);
    forward();
  }
  else if (next_cell == 5) {
    move_next();
  }
}

int find_direction() {
  int mindist = 999;
  int way_to_go=8;
  int uturn = 0;
  int call_floodfill = 0;
  Serial.println(uturn);
  if (current_direction == 0) {
    if (VerticalWalls[currentX][currentY + 1] == 0) {
      if (manhattan_distances[currentX][currentY + 1] < mindist) {
        mindist = manhattan_distances[currentX][currentY + 1];
        way_to_go = 3;
      }
      if (manhattan_distances[currentX][currentY + 1] > manhattan_distances[currentX][currentY]) {
        call_floodfill++;
      }
    }
    else {
      call_floodfill++;
      uturn++;
    }
    if (HorizontalWalls[currentX][currentY] == 0) {
      if (manhattan_distances[currentX - 1][currentY] < mindist) {
        mindist = manhattan_distances[currentX - 1][currentY];
        way_to_go = 0;

      }
      if (manhattan_distances[currentX - 1][currentY] > manhattan_distances[currentX][currentY]) {
        call_floodfill++;
      }
    }
    else {
      call_floodfill++;
      uturn++;

    }
    if (VerticalWalls[currentX][currentY] == 0) {
      if (manhattan_distances[currentX][currentY - 1] < mindist) {
        mindist = manhattan_distances[currentX][currentY - 1];
        way_to_go = 2;
      }
      if (manhattan_distances[currentX][currentY - 1] > manhattan_distances[currentX][currentY]) {
        call_floodfill++;

      }
    }
    else {
      call_floodfill++;
      uturn++;

    }
    if (uturn >= 3) {
      way_to_go = 1;
    }
  }

  if (current_direction == 90) {


    if (HorizontalWalls[currentX + 1][currentY] == 0) {
      if (manhattan_distances[currentX + 1][currentY] < mindist) {
        mindist = manhattan_distances[currentX + 1][currentY];
        way_to_go = 3;
      }
      if (manhattan_distances[currentX + 1][currentY] > manhattan_distances[currentX][currentY]) {
        call_floodfill++;

      }
    }
    else {
      call_floodfill++;
      uturn++;

    }
    if (VerticalWalls[currentX][currentY + 1] == 0) {
      if (manhattan_distances[currentX][currentY + 1] < mindist) {
        mindist = manhattan_distances[currentX][currentY + 1];
        way_to_go = 0;

      }
      if (manhattan_distances[currentX][currentY + 1] > manhattan_distances[currentX][currentY]) {
        call_floodfill++;

      }
    }
    else {
      call_floodfill++;
      uturn++;

    }
    if (HorizontalWalls[currentX][currentY] == 0) {
      if (manhattan_distances[currentX - 1][currentY] < mindist) {
        mindist = manhattan_distances[currentX - 1][currentY];
        way_to_go = 2;
      }
      if (manhattan_distances[currentX - 1][currentY] > manhattan_distances[currentX][currentY]) {
        call_floodfill++;
      }
    }
    else {
      call_floodfill++;
      uturn++;

    }
    if (uturn >= 3) {
      way_to_go = 1;
    }

  }

  if (current_direction == 180) {


    if (VerticalWalls[currentX][currentY] == 0) {
      if (manhattan_distances[currentX][currentY - 1] < mindist) {
        mindist = manhattan_distances[currentX][currentY - 1];
        way_to_go = 3;
      }
      if (manhattan_distances[currentX][currentY - 1] > manhattan_distances[currentX][currentY]) {
        call_floodfill++;
      }
    }
    else {
      call_floodfill++;
      uturn++;

    }
    if (HorizontalWalls[currentX + 1][currentY] == 0) {
      if (manhattan_distances[currentX + 1][currentY] < mindist) {
        mindist = manhattan_distances[currentX + 1][currentY];
        way_to_go = 0;
      }
      if (manhattan_distances[currentX + 1][currentY] > manhattan_distances[currentX][currentY]) {
        call_floodfill++;
      }

    }
    else {
      call_floodfill++;
      uturn++;

    }
    if (VerticalWalls[currentX][currentY + 1] == 0) {
      if (manhattan_distances[currentX][currentY + 1] < mindist) {
        mindist = manhattan_distances[currentX][currentY + 1];
        way_to_go = 2;
      }
      if (manhattan_distances[currentX][currentY + 1] > manhattan_distances[currentX][currentY]) {
        call_floodfill++;
      }
    }
    else {
      call_floodfill++;
      uturn++;

    }
    if (uturn >= 3) {
      way_to_go = 1;
    }

  }

  if (current_direction == 270) {


    if (HorizontalWalls[currentX][currentY] == 0) {
      if (manhattan_distances[currentX - 1][currentY] < mindist) {
        mindist = manhattan_distances[currentX - 1][currentY];
        way_to_go = 3;
      }
      if (manhattan_distances[currentX - 1][currentY] > manhattan_distances[currentX][currentY]) {
        call_floodfill++;
      }
    }
    else {
      call_floodfill++;
      uturn++;

    }
    if (VerticalWalls[currentX][currentY] == 0) {
      if (manhattan_distances[currentX][currentY - 1] < mindist) {
        mindist = manhattan_distances[currentX][currentY - 1];
        way_to_go = 0;
        uturn = false;

      }
      if (manhattan_distances[currentX][currentY - 1] > manhattan_distances[currentX][currentY]) {
        call_floodfill++;
      }
    }
    else {
      call_floodfill++;
      uturn++;

    }
    if (VerticalWalls[currentX + 1][currentY] == 0) {
      if (manhattan_distances[currentX + 1][currentY] < mindist) {
        mindist = manhattan_distances[currentX + 1][currentY];
        way_to_go = 2;
        uturn = false;
      }
      if (manhattan_distances[currentX + 1][currentY] > manhattan_distances[currentX][currentY]) {
        call_floodfill++;
      }
    }
    else {
      call_floodfill++;
      uturn++;

    }
    if (uturn >= 3) {
      way_to_go = 1;

    }

  }
  if (call_floodfill >= 3 && uturn >= 3) {
    flood_fill();
    /*for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        Serial.print(manhattan_distances[i][j]);
      }
      Serial.println();
      }*/
    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < 6; j++) {
        Serial.print(HorizontalWalls[i][j]);
      }
      Serial.println();
    }
    way_to_go = 1;
  }
  else if (call_floodfill >= 3) {
    flood_fill();
    for (int i = 0; i < maze_size+1; i++) {
      for (int j = 0; j < maze_size; j++) {
        Serial.print(HorizontalWalls[i][j]);
      }
      Serial.println();
    }
    /*for (int i = 0; i < 7; i++) {
      for (int j = 0; j < 6; j++) {
        Serial.print(HorizontalWalls[i][j]);
      }
      Serial.println();
      }*/
    way_to_go = 5;
  }
  return way_to_go;
}
