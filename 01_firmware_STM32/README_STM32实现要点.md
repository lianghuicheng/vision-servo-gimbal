# STM32 端实现要点（接收视觉坐标 → PID → 舵机/电机）

> 本文件是**给你自己看的实现指南**。协议规范见 `..\03_协议文档\串口协议_v1.md`。
> 目标：不依赖队友也能把这条链路跑通 —— 这是你"不可替代性"的来源。

---

## 一、分工定位（重要）

旧计划里你做视觉、队友 A 做 STM32。**本计划要求你也把这一段做一遍**，理由：
1. 只有自己写过接收解析，才知道协议哪里设计得不好；
2. 面试时被问"串口丢包怎么办"你能答出真实经验；
3. 一个人也能把项目跑通 = 组队失败时的保险。

**重复劳动是刻意的**：你写的版本是"学习版/备份版"，队友的版本是"比赛版"。

---

## 二、接收方案：IDLE 中断 + DMA（不要用轮询）

### 为什么不用轮询

```c
// ❌ 反面做法：HAL_UART_Receive 轮询
HAL_UART_Receive(&huart1, buf, 11, 100);   // 阻塞 100ms，主循环被卡死，PID 周期全乱
```

轮询会阻塞主循环 → PID 周期抖动 → **永远调不好控制**。这是新手最常犯的错。

### 正确做法：DMA 搬数据 + IDLE 中断判帧尾

原理：串口线空闲（IDLE）时产生中断，此时 DMA 已把这一批字节搬到缓冲区，
在中断里算出本次收到多少字节，交给解析器。

```c
/* ---- 1. 定义缓冲区（放在文件顶部） ---- */
#define RX_BUF_SIZE  128
uint8_t  rx_dma_buf[RX_BUF_SIZE];      /* DMA 直接写入 */
uint8_t  rx_proc_buf[RX_BUF_SIZE];     /* 给解析器用，避免 DMA 写一半被读 */
volatile uint16_t rx_len = 0;          /* 本次收到的字节数 */
volatile uint8_t  rx_ready = 0;        /* 标志：有一批新数据待处理 */

/* ---- 2. 启动（main 初始化里调用一次） ---- */
void UART_StartReceive(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);   /* 关掉半传输中断，否则会中断两次 */
}

/* ---- 3. 空闲中断回调（HAL 会调用它） ---- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        memcpy(rx_proc_buf, rx_dma_buf, Size);   /* 拷走，防止被下一批覆盖 */
        rx_len = Size;
        rx_ready = 1;                            /* 只置标志，重活留给主循环 */
        /* 立刻重启接收，否则只收到一次 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}
```

> 💡 CubeMX 里要打开：USART1 的 **DMA RX（Normal 模式）** 和 **NVIC 里的 USART1 全局中断**。

---

## 三、解析：状态机（必须能处理粘包/断包）

**核心思想**：不能假设"一次收到一个完整帧"。要维护跨调用的缓冲区。

```c
#define FRAME_HEAD0  0xAA
#define FRAME_HEAD1  0x55
#define FRAME_TAIL0  0x0D
#define FRAME_TAIL1  0x0A
#define MAX_PAYLOAD  64

typedef struct {
    uint16_t cx, cy, w, h;
    int16_t  angle;      /* 单位 0.1° */
    uint8_t  lost;       /* 1 = 目标丢失 */
    uint8_t  valid;      /* 1 = 数据是新的 */
} Target_t;

Target_t g_target;
volatile uint32_t g_stat_ok = 0, g_stat_crc_err = 0, g_stat_tail_err = 0;

/* ---- CRC16-CCITT，必须和 Python 端一致 ---- */
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ---- 环形缓冲区 + 状态机 ---- */
static uint8_t  s_buf[512];
static uint16_t s_head = 0, s_tail = 0;

static void buf_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_head + 1) % sizeof(s_buf));
    if (next != s_tail) { s_buf[s_head] = byte; s_head = next; }   /* 满了就丢新数据 */
}

static int buf_pop(uint8_t *out)
{
    if (s_head == s_tail) return 0;
    *out = s_buf[s_tail];
    s_tail = (uint16_t)((s_tail + 1) % sizeof(s_buf));
    return 1;
}

/* 在主循环里调用（不阻塞） */
void Protocol_Task(void)
{
    static uint8_t  frame[MAX_PAYLOAD + 8];
    static uint16_t idx = 0;
    static uint8_t  len_field = 0;
    uint8_t byte;

    if (rx_ready) {                       /* 把新到的数据推进环形缓冲区 */
        __disable_irq();
        for (uint16_t i = 0; i < rx_len; i++) buf_push(rx_proc_buf[i]);
        rx_ready = 0;
        __enable_irq();
    }

    while (buf_pop(&byte)) {
        /* --- 状态 0：找帧头 --- */
        if (idx == 0) { if (byte == FRAME_HEAD0) frame[idx++] = byte; continue; }
        if (idx == 1) {
            if (byte == FRAME_HEAD1)      frame[idx++] = byte;
            else if (byte == FRAME_HEAD0) frame[1] = byte;   /* 可能是 AA AA 55 */
            else                          idx = 0;
            continue;
        }
        /* --- 状态 2：收 LEN --- */
        if (idx == 2) {
            if (byte == 0 || byte > MAX_PAYLOAD) { idx = 0; continue; }   /* 非法长度，重新找头 */
            len_field = byte;
            frame[idx++] = byte;
            continue;
        }
        /* --- 状态 3：收 PAYLOAD + CRC --- */
        frame[idx++] = byte;
        uint16_t need = (uint16_t)(3 + len_field + 2);        /* 头2+len1+载荷+crc2 */
        if (idx < need) continue;

        /* --- 状态 4：收帧尾 --- */
        if (frame[0] != FRAME_HEAD0 || frame[1] != FRAME_HEAD1) { idx = 0; continue; }
        frame[idx++] = 0;  /* 占位，实际下一字节判断 */
        idx--;             /* 简化：先判断已有 need 字节，再等 2 字节帧尾 */
        /* 收帧尾 */
        frame[idx++] = byte;   /* 注意：此处逻辑见下方说明 */
        break;
    }
}
```

> ⚠️ 上面状态机为了篇幅做了简化，**建议做法**：一次性等够 `need + 2` 字节再统一校验，
> 伪代码：
> ```
> 1. 在环形缓冲区里找 AA 55
> 2. 若剩余字节 < 3 → 退出等待
> 3. 取 LEN，若 LEN==0 或 LEN>64 → 跳过这个 AA，回到 1
> 4. 若剩余字节 < 3+LEN+2+2 → 退出等待
> 5. 校验尾部 == 0D 0A ? 否则跳过 AA，回到 1
> 6. 计算 CRC16(LEN+PAYLOAD) 与收到的比对 ? 不等则 crc_err++，跳过 AA，回到 1
> 7. 解析 PAYLOAD → 更新 g_target，ok++，把这一帧从缓冲区消费掉，回到 1
> ```
> 这和 `protocol.py` 里 `StreamDecoder.feed()` 的逻辑**必须完全一致**。
> **建议：先在 PC 上用 Python 把 StreamDecoder 测通（含粘包/断包/错帧），再照抄逻辑到 C。**

---

## 四、超时与兜底（安全动作）

```c
static uint32_t s_last_rx_tick = 0;

void Safety_Task(void)
{
    if (HAL_GetTick() - s_last_rx_tick > 1000) {     /* 1 秒没收到任何合法帧 */
        g_target.valid = 0;
        g_target.lost  = 1;
        Motor_Stop();                                 /* ← 兜底动作，防止失控 */
        /* 可选：舵机回中位、蜂鸣器报警 */
    }
}
```

**这条是电赛的拿分点**：视觉被遮挡时系统要"安全地失败"，而不是疯跑。

---

## 五、单轴 PID（位置式）模板（W5 用）

```c
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_err;
    float out_min, out_max;
    float integral_max;      /* 抗积分饱和 */
    float dead_zone;         /* 死区，防止抖动 */
    float dt;                /* 采样周期（秒），固定值！ */
} PID_t;

float PID_Update(PID_t *p, float target, float measure)
{
    float err = target - measure;

    if (fabsf(err) < p->dead_zone) err = 0;          /* 死区 */

    p->integral += err * p->dt;
    if (p->integral >  p->integral_max) p->integral =  p->integral_max;   /* 抗饱和 */
    if (p->integral < -p->integral_max) p->integral = -p->integral_max;

    float derivative = (err - p->prev_err) / p->dt;
    p->prev_err = err;

    float out = p->kp * err + p->ki * p->integral + p->kd * derivative;

    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;
    return out;
}
```

**调用方式（固定周期，千万别用 HAL_Delay 或不定长循环）**

```c
/* TIM6 定时器中断里，1 kHz 或 200 Hz 固定触发 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        g_flag_1ms = 1;
    }
}

/* 主循环 */
while (1) {
    Protocol_Task();                 /* 解析串口，非阻塞 */
    if (g_flag_1ms) {
        g_flag_1ms = 0;
        static uint16_t cnt = 0;
        if (++cnt >= 5) {            /* 每 5 ms 跑一次 PID（200 Hz） */
            cnt = 0;
            float ex = (float)g_target.cx - IMG_CX;       /* 图像中心 320 */
            float ey = (float)g_target.cy - IMG_CY;       /* 图像中心 240 */
            float out_x = PID_Update(&pid_yaw,  0.0f, ex);
            float out_y = PID_Update(&pid_pitch, 0.0f, ey);
            Servo_SetAngle(SERVO_YAW,   SERVO_YAW_MID   + out_x);
            Servo_SetAngle(SERVO_PITCH, SERVO_PITCH_MID + out_y);
        }
    }
    Safety_Task();
}
```

**注意**：这里 PID 的目标是 **0**（把误差调到 0），输入是**目标与画面中心的偏差**。
这是视觉伺服最经典的用法。

---

## 六、调参记录表（W5–W6 必须填）

| 序号 | Kp | Ki | Kd | 采样周期 | 现象（振荡/超调/静差/迟滞） | 结论 |
|---|---|---|---|---|---|---|
| 1 | 0.5 | 0 | 0 | 5 ms | | |
| 2 | 1.0 | 0 | 0 | 5 ms | | |
| 3 | | | | | | |

**调参顺序**：只用 P → 加到临界振荡再回退一半 → 加 D 压振荡 → 最后加少量 I 消静差。

---

## 七、常见坑速查

| 现象 | 原因 | 解决 |
|---|---|---|
| 收到的数总是错位/跳变 | 没处理粘包，把半个帧当整帧 | 上状态机 + 长度字段 |
| 偶尔出现完全离谱的数值 | 没校验 CRC | 加 CRC 并丢弃错误帧 |
| 只收到第一帧，后面没反应 | 忘了在回调里重启 DMA 接收 | `HAL_UARTEx_ReceiveToIdle_DMA` 重新调用 |
| 电机一转就乱码/复位 | 电源耦合干扰，共电源 | 独立供电 + 光耦隔离 + 去耦电容 + 共地 |
| PID 怎么调都抖 | 主循环被阻塞，周期不固定 | 用定时器固定周期，去掉 HAL_Delay |
| 舵机抖动、MCU 复位 | 舵机电流冲击 | 舵机独立 5V 供电，电源加大电容（≥1000 µF） |
| 舵机不动 | PWM 定时器没使能 / 占空比算错 | 确认 PSC/ARR 与 50 Hz（20 ms 周期）匹配 |

---

## 八、W1–W8 检查清单

- [ ] CubeMX 配好 USART1（115200, DMA RX, 全局中断）
- [ ] 心跳帧能正确解析（type=0x03）
- [ ] 固定 TARGET 帧解析出的 5 个数值与 Python 端打印完全一致
- [ ] CRC 自测向量 `0x29B1` 通过（对 ASCII `"123456789"`，init=0xFFFF）
- [ ] 故意插错字节，CRC 错误计数增加且程序不崩
- [ ] 连发 1000 帧，接收计数 ≥ 999
- [ ] 舵机 PWM 输出正常（示波器或直接看舵机转动）
- [ ] 单轴闭环跑起来，能跟踪扫动的目标
- [ ] 双轴闭环，静态误差 ±3° 以内
- [ ] 拔掉视觉端电源，1 s 内进入安全动作
