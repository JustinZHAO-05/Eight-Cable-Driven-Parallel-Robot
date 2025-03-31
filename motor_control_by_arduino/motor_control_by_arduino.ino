#define NUM_MOTORS 8  // 8 个步进电机

// 定义每个电机的控制引脚（NPN接口）
// 每个电机 3 个引脚：顺时针 (CW)、停止 (STOP)、逆时针 (CCW)
const int PIN_CW[NUM_MOTORS]   = {2, 3, 4, 5, 6, 7, 8, 9};
const int PIN_STOP[NUM_MOTORS] = {10, 11, 12, 13, A0, A1, A2, A3};
const int PIN_CCW[NUM_MOTORS]  = {22, 23, 24, 25, 26, 27, 28, 29};

// 全局变量：目标步数和当前步数，用于跟踪每个电机的运动状态（开环控制）
volatile long targetSteps[NUM_MOTORS] = {0};
volatile long currentSteps[NUM_MOTORS] = {0};

// 默认脉冲间隔（单位：微秒）
volatile int stepDelay = 500;

void setup() {
  Serial.begin(115200);
  
  // 初始化所有控制引脚为输出，并设置初始状态：
  // 对于 NPN 接口，STOP 信号默认为高电平（解除制动），
  // 而 CW 与 CCW 引脚默认设为 HIGH（不产生脉冲）。
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(PIN_CW[i], OUTPUT);
    pinMode(PIN_STOP[i], OUTPUT);
    pinMode(PIN_CCW[i], OUTPUT);
    digitalWrite(PIN_CW[i], HIGH);     // CW 引脚默认 HIGH（不激活）
    digitalWrite(PIN_CCW[i], HIGH);    // CCW 引脚默认 HIGH
    digitalWrite(PIN_STOP[i], HIGH);    // STOP 开机时默认 HIGH（解除制动状态）
  }
  
}

void loop() {
  // 如果串口有数据，则读取并解析指令
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
  }
  
  // 持续调用电机运动控制函数
  moveMotors();
}

// 解析 PC 发送的指令，支持 "TIGHTEN" 指令和普通目标步数指令。
// 指令格式示例："M 500 -300 200 0 100 0 -150 250;T:500"
// "M" 后面依次为每个电机的步数变化，";T:" 后面为运动时间（毫秒）
void parseCommand(String command) {
  if (command.startsWith("TIGHTEN")) {
    tightenCables();          // 执行预紧操作
    Serial.println("TIGHTEN_OK");
  } else {
    int T = 500; // 默认运动时间（毫秒）
    int motorValues[NUM_MOTORS] = {0};

    // 将 String 转换为 char 数组以便使用 strtok 分割
    char buf[command.length() + 1];
    command.toCharArray(buf, sizeof(buf));
    // 使用 ";" 分割指令字符串
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
    
    // 更新目标步数：目标步数 = 当前步数 + 指令变化量
    for (int i = 0; i < NUM_MOTORS; i++) {
      targetSteps[i] = currentSteps[i] + motorValues[i];
    }
    
    // 根据所有电机的步数变化量计算脉冲间隔（单位：微秒）
    int totalChange = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
      totalChange += abs(motorValues[i]);
    }
    if (totalChange > 0) {
      stepDelay = T * 1000 / totalChange;
    }
    
  }
}

// 预紧绳索：对每个电机施加一定数量的脉冲，以确保负载下保持绳索张力
void tightenCables() {
  int tightenForce = 50;  // 预紧脉冲数（可调）
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    // 预紧时，假设使用顺时针控制，即使用 PIN_CW 信号
    digitalWrite(PIN_STOP[i], HIGH); // 先解除 STOP（设为 HIGH）允许转动
    for (int j = 0; j < tightenForce; j++) {
      digitalWrite(PIN_CW[i], LOW);   // 激活顺时针脉冲（低电平有效）
      delayMicroseconds(500);
      digitalWrite(PIN_CW[i], HIGH);  // 结束脉冲
      delayMicroseconds(500);
    }
    currentSteps[i] += tightenForce;
    targetSteps[i] = currentSteps[i]; // 拉紧后，目标与当前步数相同
    digitalWrite(PIN_STOP[i], LOW);  // 恢复 STOP 状态（低电平有效）
  }
}

// 电机运动控制函数：遍历每个电机，产生脉冲直到当前步数达到目标步数
void moveMotors() {
  bool allDone = true;
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (currentSteps[i] != targetSteps[i]) {
      allDone = false;
      
      // 根据目标与当前步数决定转向：
      // 如果目标大于当前，表示需要顺时针转动；否则逆时针转动。
      if (targetSteps[i] > currentSteps[i]) {
        digitalWrite(PIN_CW[i], LOW);      // 激活顺时针
        digitalWrite(PIN_STOP[i], HIGH); // 解除 STOP
      } else {
        digitalWrite(PIN_CCW[i], LOW);     // 激活逆时针
        digitalWrite(PIN_STOP[i], HIGH); // 解除 STOP
      }
      
      // 产生脉冲，保持一定的脉冲宽度
      delayMicroseconds(stepDelay);
      digitalWrite(PIN_STOP[i], LOW);  // 重新使 STOP 生效
      digitalWrite(PIN_CW[i], HIGH);   // 结束脉冲
      digitalWrite(PIN_CCW[i], HIGH);
      
      // 更新当前步数向目标靠近
      if (targetSteps[i] > currentSteps[i])
        currentSteps[i]++;
      else
        currentSteps[i]--;
    }
  }
  
  // 当所有电机均达到目标步数后，向 PC 发送 "DONE" 信号
  if (allDone) {
    Serial.println("DONE");
  }
}
