#define NUM_MOTORS 8  // 8 个步进电机

// 分配 8 个电机的脉冲信号：使用数字引脚 2~9
const int PUL[NUM_MOTORS] = {2, 3, 4, 5, 6, 7, 8, 9};
// 分配 8 个电机的方向信号：使用数字引脚 10,11,12,13 和模拟引脚 A0,A1,A2,A3（作为数字 I/O）
const int DIR[NUM_MOTORS] = {10, 11, 12, 13, A0, A1, A2, A3};

// 全局变量：目标步数和当前步数，用于跟踪每个电机的运动状态（开环控制）
volatile long targetSteps[NUM_MOTORS] = {0};
volatile long currentSteps[NUM_MOTORS] = {0};
volatile int stepDelay = 500; // 默认脉冲间隔（微秒）

void setup() {
  Serial.begin(115200);
  
  // 初始化脉冲和方向引脚为输出，并设置初始低电平
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(PUL[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
    digitalWrite(PUL[i], LOW);
    digitalWrite(DIR[i], LOW);
  }
}

void loop() {
  // 如果串口有数据，读取一行指令进行解析
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
  }
  
  moveMotors(); // 持续调用电机运动控制函数
}

// 解析 PC 发送的指令，支持 "TIGHTEN" 指令和普通目标步数指令
void parseCommand(String command) {
  if (command.startsWith("TIGHTEN")) {
    tightenCables();          // 执行预紧操作
    Serial.println("TIGHTEN_OK"); // 反馈给 PC
  } else {
    int T = 500; // 默认运动时间（毫秒）
    int motorValues[NUM_MOTORS] = {0};

    // 将 String 转换为 char 数组以便使用 strtok 分割
    char buf[command.length() + 1];
    command.toCharArray(buf, sizeof(buf));
    char *token = strtok(buf, ";");
    while (token != NULL) {
      if (token[0] == 'M' && isDigit(token[1])) {
        int motorIdx = token[1] - '1';  // 如 "M1" 对应索引 0
        long steps = atol(&token[3]);   // 解析步数值
        if (motorIdx >= 0 && motorIdx < NUM_MOTORS) {
          motorValues[motorIdx] = steps;
        }
      } else if (token[0] == 'T') {
        T = atoi(&token[2]);  // 解析运动时间（毫秒）
      }
      token = strtok(NULL, ";");
    }

    // 更新目标步数：目标步数 = 当前步数 + 指令步数变化量
    for (int i = 0; i < NUM_MOTORS; i++) {
      targetSteps[i] = currentSteps[i] + motorValues[i];
    }

    // 根据第一个电机的步数变化量计算脉冲间隔（单位：微秒），避免除以零
    if (abs(motorValues[0]) > 0) {
      stepDelay = T * 1000 / abs(motorValues[0]);
    }
  }
}

// 预紧绳索：对每个电机施加一定数量的脉冲，用于确保负载下保持绳索张力
void tightenCables() {
  int tightenForce = 50;  // 预紧脉冲数量（可调）
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    digitalWrite(DIR[i], HIGH);  // 设置方向为预紧方向（假设 HIGH 表示拉紧）
    for (int j = 0; j < tightenForce; j++) {
      digitalWrite(PUL[i], HIGH);
      delayMicroseconds(500);
      digitalWrite(PUL[i], LOW);
      delayMicroseconds(500);
    }
    // 更新当前步数：假设每个电机均向前增加 tightenForce 步
    currentSteps[i] += tightenForce;
    targetSteps[i] = currentSteps[i]; // 拉紧后目标位置与当前一致
  }
}

// 电机运动控制函数：逐个检查每个电机，产生脉冲直到当前步数达到目标步数
void moveMotors() {
  bool allDone = true;
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (currentSteps[i] != targetSteps[i]) {
      allDone = false;
      
      // 根据目标和当前步数决定电机转向
      if (targetSteps[i] > currentSteps[i]) {
        digitalWrite(DIR[i], HIGH);
      } else {
        digitalWrite(DIR[i], LOW);
      }
      
      // 产生一个脉冲：HIGH 延时，再 LOW 延时，构成一个完整脉冲
      digitalWrite(PUL[i], HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(PUL[i], LOW);
      delayMicroseconds(stepDelay);
      
      // 更新当前步数：向目标靠近
      if (targetSteps[i] > currentSteps[i]) {
        currentSteps[i]++;
      } else {
        currentSteps[i]--;
      }
    }
  }
  
  // 当所有电机都达到目标后，向 PC 发送 "DONE" 信号
  if (allDone) {
    Serial.println("DONE");
  }
}
