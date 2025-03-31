#include <TimerOne.h>

#define NUM_MOTORS 8  // 8 个步进电机

// 定义每个电机的控制引脚（NPN接口）
// 每个电机3个引脚：顺时针 (CW)、停止 (STOP)、逆时针 (CCW)
const int PIN_CW[NUM_MOTORS]   = {2, 3, 4, 5, 6, 7, 8, 9};
const int PIN_STOP[NUM_MOTORS] = {10, 11, 12, 13, A0, A1, A2, A3};
const int PIN_CCW[NUM_MOTORS]  = {22, 23, 24, 25, 26, 27, 28, 29};

// 全局变量：目标步数和当前步数
volatile long targetSteps[NUM_MOTORS] = {0};
volatile long currentSteps[NUM_MOTORS] = {0};

// 每个电机的中断计数器，用于累积中断次数
volatile unsigned long pulseCounter[NUM_MOTORS] = {0};
// 每个电机所需的中断次数阈值（即每隔多少个中断产生一次脉冲），默认值
volatile unsigned long threshold[NUM_MOTORS] = {80,80,80,80,80,80,80,80};  // 比如 80 次中断 = 80*50us = 4000us = 4ms

// 脉冲宽度（单位：微秒），例如 10 微秒
volatile int PULSE_WIDTH = 10;

// 定时器中断周期（单位：微秒）
const int ISR_PERIOD = 50;  // 50 微秒

void setup() {
  Serial.begin(115200);
  
  // 初始化所有电机的引脚：
  // 注意：对于 NPN 接口，STOP 引脚默认为低电平（激活状态）
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(PIN_CW[i], OUTPUT);
    pinMode(PIN_STOP[i], OUTPUT);
    pinMode(PIN_CCW[i], OUTPUT);
    digitalWrite(PIN_CW[i], HIGH);    // 不激活
    digitalWrite(PIN_CCW[i], HIGH);   // 不激活
    digitalWrite(PIN_STOP[i], LOW);   // 激活 STOP（保持静止）
  }
  
  // 初始化 Timer1，每 50 微秒中断一次
  Timer1.initialize(ISR_PERIOD);
  Timer1.attachInterrupt(timerISR);
}

void loop() {
  // 主循环可以处理串口通信或其他任务。
  // 例如，根据接收到的命令更新 targetSteps 数组。
  // 这里简化为：如果串口有数据，则解析命令并更新 targetSteps。
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
  }
  // 其他任务...
}

// 定时器中断服务程序：每 50 微秒调用一次
void timerISR() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    // 如果该电机还未达到目标步数，则更新计数器
    if (currentSteps[i] != targetSteps[i]) {
      pulseCounter[i]++;
      // 当计数器达到阈值时，产生一个脉冲
      if (pulseCounter[i] >= threshold[i]) {
        // 先释放 STOP（设为 HIGH）使电机允许转动
        digitalWrite(PIN_STOP[i], HIGH);
        // 根据目标与当前步数决定转向：
        if (targetSteps[i] > currentSteps[i]) {
          digitalWrite(PIN_CW[i], LOW);  // 顺时针激活
        } else {
          digitalWrite(PIN_CCW[i], LOW); // 逆时针激活
        }
        
        // 等待脉冲宽度
        delayMicroseconds(PULSE_WIDTH);
        
        // 结束脉冲，将方向引脚设回 HIGH（非激活），并重新拉低 STOP（保持静止）
        digitalWrite(PIN_CW[i], HIGH);
        digitalWrite(PIN_CCW[i], HIGH);
        digitalWrite(PIN_STOP[i], LOW);
        
        // 更新当前步数向目标逼近
        if (targetSteps[i] > currentSteps[i]) {
          currentSteps[i]++;
        } else {
          currentSteps[i]--;
        }
        // 重置该电机的中断计数器
        pulseCounter[i] = 0;
      }
    }
  }
  
  // 可选：如果所有电机均达到目标，可在主循环中处理 DONE 信号
}

// 命令解析函数，类似原来的 parseCommand() 函数（不在中断中执行）
void parseCommand(String command) {
  // If the command is a "TIGHTEN" command, perform cable tightening.
  if (command.startsWith("TIGHTEN")) {
    tightenCables();
    Serial.println("TIGHTEN_OK");
  } else {
    int T = 500; // Default sample time (ms) if not provided.
    int motorValues[NUM_MOTORS] = {0};

    // Convert the command String into a char array for tokenization.
    char buf[command.length() + 1];
    command.toCharArray(buf, sizeof(buf));
    
    // Tokenize the command string using ";" as the delimiter.
    // This should split the string into two tokens:
    // 1. The motor steps token (e.g., "M 500 -300 200 0 100 0 -150 250")
    // 2. The sample time token (e.g., "T:80")
    char *token = strtok(buf, ";");
    while (token != NULL) {
      // Check if this token starts with 'M'
      if (token[0] == 'M') {
        // For motor steps, tokenize the token by spaces.
        // The first token "M" is skipped.
        char *subToken = strtok(token, " ");
        int motorIdx = 0;
        subToken = strtok(NULL, " "); // Get the first numeric value
        while (subToken != NULL && motorIdx < NUM_MOTORS) {
          long steps = atol(subToken); // Convert string to long
          motorValues[motorIdx] = steps;
          motorIdx++;
          subToken = strtok(NULL, " ");
        }
      } 
      // If token starts with 'T', it contains sample time information.
      else if (token[0] == 'T') {
        // Expect token in the format "T:80"
        T = atoi(token + 2);  // Skip the "T:" part and convert the rest to an integer.
      }
      // Get the next token.
      token = strtok(NULL, ";");
    }

    // Update targetSteps for each motor.
    for (int i = 0; i < NUM_MOTORS; i++) {
      targetSteps[i] = currentSteps[i] + motorValues[i];
    }

    // Calculate stepDelay (in microseconds) based on the sample time T and step change.
    // 根据第一个电机的步数变化量计算步进间隔（单位微秒），例如 T*1000/abs(motorValues[0])
    if (abs(motorValues[0]) > 0) {
      int newStepDelay = T * 1000 / abs(motorValues[0]);
      // 更新全局阈值（以中断计数为单位）：阈值 = stepDelay / ISR_PERIOD
      for (int i = 0; i < NUM_MOTORS; i++) {
        threshold[i] = newStepDelay / ISR_PERIOD;
      }
    }
  }
}

// 预紧绳索函数（与原来的 tightenCables 类似）
void tightenCables() {
  int tightenForce = 50;  // 脉冲数
  for (int i = 0; i < NUM_MOTORS; i++) {
    // 预紧采用顺时针控制
    digitalWrite(PIN_STOP[i], HIGH); // 先解除STOP
    for (int j = 0; j < tightenForce; j++) {
      digitalWrite(PIN_CW[i], LOW);
      delayMicroseconds(PULSE_WIDTH);
      digitalWrite(PIN_CW[i], HIGH);
      delayMicroseconds(PULSE_WIDTH);
    }
    currentSteps[i] += tightenForce;
    targetSteps[i] = currentSteps[i];
    digitalWrite(PIN_STOP[i], LOW); // 保持STOP状态
  }
}

