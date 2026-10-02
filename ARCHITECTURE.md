# ESP32-P4 MIPI-DSI RID 探测系统

> 项目仓库：<https://github.com/hepeng95/esp32P4_mipi_dsi>
> 构建框架：ESP-IDF 5.5.4 · LVGL v9 · FreeRTOS · ESP32-P4 RISC-V 双核 400MHz · 32MB PSRAM

---

## 目录

- [一、硬件总览](#一硬件总览)
- [二、软件架构](#二软件架构)
- [三、FreeRTOS 任务架构](#三freertos-任务架构)
- [四、PSRAM 内存使用](#四psram-内存使用-32mb)
- [五、各模块详解](#五各模块详解)
  - [5.1 rf_map — 瓦片地图](#51-rf_map--瓦片地图)
  - [5.2 nRF24L01 + rf_24g — 2.4G 频谱扫描](#52-nrf24l01--rf_24g--24g-频谱扫描)
  - [5.3 A5133 + rf_5g — 5.8G 频谱扫描](#53-a5133--rf_5g--58g-频谱扫描)
  - [5.4 rf_link — UART 上行链路 (RID JSON)](#54-rf_link--uart-上行链路-rid-json)
  - [5.5 rf_gps — NMEA GPS 解析](#55-rf_gps--nmea-gps-解析)
  - [5.6 rf_ui — LVGL UI 框架](#56-rf_ui--lvgl-ui-框架)
  - [5.7 rid_registry — 无人机注册表](#57-rid_registry--无人机注册表)
  - [5.8 rf_json / rf_proto / rf_ringbuf](#58-rf_json--rf_proto--rf_ringbuf)
- [六、关键设计决策与踩坑总结](#六关键设计决策与踩坑总结)
- [七、初始化流程总览](#七初始化流程总览)
- [八、sdkconfig 关键配置](#八sdkconfig-关键配置)

---

## 一、硬件总览

### 核心芯片

| 芯片 | 功能 | 接口/主频 |
|------|------|----------|
| ESP32-P4 | 主控 | RISC-V 双核 400MHz, 32MB PSRAM, MIPI-DSI, SPI2/3, UART×3 |
| ST7701S | LCD | MIPI-DSI 2-lane, 480×854, RGB565/RGB888 |
| nRF24L01 | 2.4G 扫描 | SPI, 1/2Mbps, RPD 能量阈值检测 |
| A5133 | 5.8G 扫描 | 内置 PLL + RSSI ADC, GPIO bitbang SPI |
| GNSS 模块 | GPS/北斗 | UART2 输出 NMEA |
| SD 卡 | 离线地图 | MicroSD, SPI3_HOST 4-bit / 标准 SPI 模式 |

### 引脚分配表

| 外设 | MOSI/SDA | MISO | SCLK | CS | 其他 | Host / 模式 |
|------|----------|------|------|-----|------|-------------|
| **NRF24L01** | 5 | 4 | 6 | 2 | CE=7 | SPI2_HOST (**独占**) |
| **SD 卡** | 51 (MOSI/D1) | 49 (D0) | 53 (CLK) | 52 (D3/CS) | CMD=50, D2=— | **SPI3_HOST** (避开 NRF!) |
| **A5133** | — | — | 39 (SCLK) | 41 (CS) | SDIO=34, PA_EN=27, LNA_EN=10 | GPIO bitbang |
| **GNSS** | TX→UART2 RXD | RX←UART2 TXD | — | — | — | UART2 |
| **LCD RST** | — | — | — | — | GPIO | 推挽输出 |

### ⚠️ 引脚冲突血泪史

1. **SD 卡 vs NRF24L01** 曾共用 SPI2_HOST 但引脚不同 (SD MOSI=51, NRF MOSI=5)。
   SPI 总线的物理规则：**所有从设备必须共享同一条 SCK / MOSI / MISO**（只有 CS 可以不同）。
   结果 NRF24L01 实际通过 SD 卡引脚通信，虽然自检偶然通过但不可靠。
   **修复**：SD 卡改用 **SPI3_HOST**，NRF24L01 独占 SPI2_HOST。

2. **A5133 SDIO=34** 与 SDSPI D0/D1 复用冲突。
   **修复**：A5133 不走 SPI Host，用 **GPIO bitbang**（手动翻转 39/34/41）。

3. **A5133 LNA_EN/PA_EN (10/27)** 初始没配置 → RX 链路完全不通 → PLL 校准超时。
   **修复**：在 `a5133_set_rx()` 和 `a5133_set_tx()` 里显式拉高/拉低。

---

## 二、软件架构

```
main/
├── mipi_dsi_lcd_example_main.c  (479) ← app_main 入口
├── rf_ui.c          (3741) ← LVGL UI + 页面路由
├── rf_map.c          (469) ← 瓦片地图 v6 (同步 + SPI3)
├── rf_map.h          (104)
├── rf_link.c         (1841) ← UART 上行链路 (RID JSON)
├── rf_link.h          (254)
├── rf_gps.c           (397) ← NMEA GPS 解析
├── rf_gps.h            (83)
├── rf_24g.c            (140) ← nRF24L01 2.4G 扫描
├── rf_24g.h             (41)
├── rf_5g.c             (203) ← A5133 5.8G 扫描
├── rf_5g.h              (43)
├── nrf24l01.c          (422) ← nRF24L01 SPI 驱动
├── nrf24l01.h           (81)
├── a5133.c             (528) ← A5133 bitbang SPI 驱动
├── a5133.h             (151)
├── rid_registry.c      (155) ← 无人机注册表 (LRU + 过期清理)
├── rf_proto.c          (205) ← UART 协议解析 (magic header)
├── rf_proto.h           (34)
├── rf_json.c           (359) ← 极简 JSON parser (无依赖)
├── rf_json.h            (60)
├── rf_ringbuf.c         (46) ← 环形缓冲区
├── rf_ringbuf.h         (55)
├── rf_types.h          (194) ← 全局类型定义
├── rf_ui.h              (28)
└── stb_image.h        (7481) ← 内联 PNG 解码 (STB, 公共域)
```

---

## 三、FreeRTOS 任务架构

| 任务 | CPU | 优先级 | 栈 | 功能 | 启停时机 |
|------|-----|--------|-----|------|----------|
| **LVGL UI** (idle) | 0 | 4 | 默认 | 主循环, 触摸, 渲染 | 始终运行 |
| rf_24g_task | 1 | 3 | 4KB | nRF24L01 2.4G 扫描 | 进入 Spectrum 页启动 |
| rf_5g_task | 1 | 3 | 4KB | A5133 5.8G 扫描 | 进入 Spectrum 页启动 |
| rf_gps_task | 1 | 3 | 4KB | UART2 GPS 解析 | app_main 启动一次 |
| rf_link_task | 1 | 3 | 8KB | UART RID JSON 解析 | app_main 启动一次 |

### 为什么 scan 任务放在 CPU1 prio 3？

- LVGL 在 CPU0 prio 4（更高优先级），永远不会被 scan 阻塞
- scan 是纯 CPU 忙等（SPI + settle delay），不能饿死 IDLE task
- **必须在 sdkconfig 里关掉 CPU1 IDLE WDT**：
  ```
  CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=n
  ```
- 之前 render_task CPU1 prio 2 decode tile 饿死 IDLE → 导致 WDT 死卡 → 改成同步方案

---

## 四、PSRAM 内存使用 (32MB)

| 分配 | 大小 | 位置 | 备注 |
|------|------|------|------|
| Tile cache (32 × 128KB) | 4 MB | `rf_map.c` `heap_caps_malloc(SPIRAM)` | TC_SIZE=32, 5×5 覆盖 + 7 冗余 |
| STB decode 临时 buffer | ≤ 512 KB | 同上 | 每个 tile decode 时分配, 用完释放 |
| LVGL Canvas (448×280×2 RGB565) | 246 KB | `lv_canvas_set_buffer` 静态 | |
| Spectrum/Waterfall canvases | ~400 KB | 同上 | 2.4G + 5.8G |
| LVGL heap (CONFIG_LV_MEM_SIZE) | ~10 MB | sdkconfig | |
| **剩余** | ~17 MB | — | 其他组件/堆栈使用 |

### ⚠️ PSRAM 注意事项

- tile cache 用 **TC_SIZE=32** 而非无限缓存。每个 tile ≈ 128KB, 32 个刚好 4MB, 5×5=25 个 tile 覆盖 canvas + 7 冗余
- **所有动态大 buffer 必须用 `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)`**，不能用普通 `malloc`（ESP32-P4 内部 RAM 只有 ~500KB）

---

## 五、各模块详解

### 5.1 rf_map — 瓦片地图

**文件**：`rf_map.c` (469 行), `rf_map.h`

#### 原理

Web Mercator XYZ 瓦片标准，TMS 命名约定：

```
/sdcard/tiles/{zoom}/{y}/{x}.PNG
```

坐标转换公式（`rf_map_lon_to_tile_x` / `rf_map_lat_to_tile_y`）：

```c
tile_x = (lon + 180.0) / 360.0 * (1 << zoom)
tile_y = (1.0 - log(tan(lat) + sec(lat)) / π) / 2.0 * (1 << zoom)
```

逆投影（`tile_y_to_lat`）：

```c
lat = atan(0.5 * (exp(n) - exp(-n))) × 180/π
where n = π - 2π × tile_y_frac / (1 << zoom)
```

#### 数据模型（核心，必须理解）

```
tile_x, tile_y              = Web Mercator 中心瓦片编号 (整数)
tile_slot_x, tile_slot_y    = 瓦片**左上角**在 canvas 上的像素坐标
canvas_center = (canvas_w/2, canvas_h/2)  ← canvas 中心像素
```

**初始 set_center**：

```c
tile_slot_x = canvas_center_x - frac_x * 256   // 瓦片中心对准 canvas 中心
tile_slot_y = canvas_center_y - frac_y * 256
```

**pan 方向**：手指向右拖 → `tile_slot_x += dx`（同方向，直觉友好）

#### 完整数据流

```
手指触摸 → LV_EVENT_PRESSED → LV_EVENT_PRESSING → LV_EVENT_RELEASED
     │                              │
     ▼                              ▼
ridmap_touch_cb          ridmap_touch_cb (每帧)
     │                              │
     │                        dx = p.x - last_x
     │                        dy = p.y - last_y
     │                              │
     │                        rf_map_pan(view, dx, dy)
     │                              │
     │                        ┌─────┴──────────────────────────┐
     │                        │ Y 方向:                          │
     │                        │   dy>0 → memmove(buf,buf+dy*stride, ...) │
     │                        │            像素向上搬, 底 dy 行 memset BG  │
     │                        │   dy<0 → memmove(buf+|dy|*stride, buf, ...) │
     │                        │            像素向下搬, 顶 |dy| 行 memset BG  │
     │                        ├────────────────────────────────┤
     │                        │ X 方向 (逐行):                   │
     │                        │   dx>0 → memmove(row, row+dx*2, ...)   │
     │                        │            像素向左搬, 右 dx 列 memset BG  │
     │                        │   dx<0 → memmove(row+|dx|*2, row, ...) │
     │                        │            像素向右搬, 左 |dx| 列 memset BG │
     │                        ├────────────────────────────────┤
     │                        │ ★ slot 同步 (同方向):             │
     │                        │   tile_slot_x += dx              │
     │                        │   tile_slot_y += dy              │
     │                        ├────────────────────────────────┤
     │                        │ lv_obj_invalidate(canvas)       │
     │                        │ ← 通知 LVGL 刷新, 否则拖影!      │
     │                        ├────────────────────────────────┤
     │                        │ shift 检查:                      │
     │                        │   tx_target = canvas_w/2 - 128 │
     │                        │   sx = round((slot_x - target)/256) │
     │                        │   if (sx||sy):                   │
     │                        │     tile_x -= sx; slot_x -= sx*256 │
     │                        │     rf_map_request_edges()       │
     │                        └─────────────────────────────────┘
     │
     ▼ (pan 回调末尾)
render_grid(view, 1)    ← 3×3, 同步, cache hit 秒开
     │
     ├─ tc_lookup(zoom, tile_x+dx, tile_y+dy)  × 9 tiles
     │     hit → paste (memcpy 级速度)
     │     miss → read_psram(PNG) → stbi_load → RGB888→RGB565 → tc_insert → paste
     │
     └─ lv_obj_invalidate(canvas)
```

#### 初始化流程

```c
rf_map_init()
  ├─ mount_sd(SPI3_HOST, SDSPI_DEVICE_CONFIG_DEFAULT, ...)
  │     host.slot = SPI3_HOST          // ★ 避开 NRF SPI2
  │     cs.gpio_cs = RF_MAP_SD_D3      // D3 = CS
  │     esp_vfs_fat_sdspi_mount()      // 3 次重试
  │     ESP_LOGI("SD mount OK card=%.1fGB")
  │
  ├─ tc_flush()  // 清空 tile cache
  │
  └─ /* ★ 不启动 render_task — 全同步无竞争 */
```

#### Zoom 锚点反推（关键！防漂移）

```c
void rf_map_zoom(view, delta) {
    /* ★ 关键: 反推 canvas 中心当前对应的地理坐标 ★
     * view->center_lat/lon 可能是 set_center 时的初始值, pan 后已过期! */
    canvas_center_to_geo(view, &cur_lat, &cur_lon);

    double fx = rf_map_lon_to_tile_x(cur_lon, nz);
    double fy = rf_map_lat_to_tile_y(cur_lat, nz);

    view->tile_slot_x = canvas_w/2 - (fx - floor(fx)) * 256;
    view->tile_slot_y = canvas_h/2 - (fy - floor(fy)) * 256;

    rf_map_request_redraw(view);  // fill_bg + 3×3
}
```

#### Tile Cache 结构

```c
#define TC_SIZE 32
typedef struct {
    int zoom, tx, ty;
    int w, h;          /* 真实 decode 尺寸 (cache hit paste 要用) */
    uint16_t *rgb;     /* heap_caps_malloc(SPIRAM) */
    uint32_t age;      /* LRU */
    bool used;
} tc_t;
static tc_t s_tc[TC_SIZE];
```

TC 管理：
- `tc_lookup(zoom, tx, ty)` — 全表匹配命中
- `tc_insert(zoom, tx, ty, rgb, w, h)` — 淘汰最老
- `tc_flush()` — `rf_map_exit_page()` 调, 释放所有 `rgb` 指针

#### ⚠️ rf_map 踩坑清单

| 坑 | 根因 | 修复 |
|----|------|------|
| **v4 死卡** | render_task + mutex 竞争, pan 每帧拿锁阻塞 decode | 删 task + mutex, 全同步 |
| **memmove 方向反** | 手指右拖 → 像素没向左搬 → 拖影 | `memmove(row, row+dx*2, (w-dx)*2)` |
| **tile_slot 方向反** | `tile_slot -= dx` (反直觉) | `tile_slot += dx` (同方向) |
| **v3 cache 只有 8** | LRU 频繁 miss | TC_SIZE=32 (4MB PSRAM 够) |
| **SD SPI2 vs NRF SPI2** | 物理总线冲突 | SD 改 SPI3_HOST |
| **zoom 跳回初始位置** | 用 stale center_lat | `canvas_center_to_geo()` 反推 |
| **竖条纹花屏** | `lv_canvas_set_buffer` 后没设对象大小 | `lv_obj_set_size(canvas, w, h)` |
| **shift 方向错** | `tile_x += sx` (应该 -=) | `tile_x -= sx` |

---

### 5.2 nRF24L01 + rf_24g — 2.4G 频谱扫描

**文件**：`nrf24l01.c` (422 行), `rf_24g.c` (140 行)

#### 原理

nRF24L01 的 RPD 寄存器 (`NRF_REG_RPD`, 0x09, bit0) 是**能量阈值检测**（PWR > -64dBm → 1），非真正 RSSI ADC。
为了让频谱有中间态（不是只有 0 或 180），多次采样 RPD hits 数，映射到多档灰度。

#### 关键时序

```c
uint8_t nrf_detect_channel(ch, settle_us=100) {
    ce_low();                    // CE 必须 Low 才能切通道
    nrf_set_channel(ch);         // SPI 写 RF_CH register
    nrf_clear_irq_flags();       // 清中断标志
    ce_high();                   // 进 RX 模式
    esp_rom_delay_us(100);       // ★ RPD settling (datasheet min 70µs)
    uint8_t rpd = nrf_read_reg(NRF_REG_RPD) & 0x01;
    ce_low();
    return rpd;                  // 0 或 1
}
```

#### 扫描任务参数（已优化到 26 FPS）

```c
RF2G_CHANNELS       = 126    // ch0~125 → 2400~2525MHz
RF2G_SAMPLES_PER_CH = 3      // hits 0~3 → ×60 = 0/60/120/180 四档灰度
RF2G_SETTLE_US      = 100    // RPD settling (datasheet 70µs, 留 30µs 余量)
```

总耗时：**126ch × 3 × 100µs ≈ 37ms → 26 FPS**（之前 151ms → 6 FPS）

#### SPI 驱动（手动 CS）

```c
/* ★ 必须手动 CS 控制 — nRF24L01 对 CSN 时序要求严格 */
static esp_err_t spi_xfer(tx, rx, len) {
    cs_low();                      // GPIO 手动拉低 CSN
    spi_device_polling_transmit(); // SPI host 传输
    cs_high();                     // GPIO 手动拉高 CSN
}
```

为什么不能用硬件 CS？ESP32 SPI host 的 CS 是 edge-triggered, nRF24L01 要求 CSN 在 SCLK 空闲期间稳定拉低/拉高。

#### 初始化流程

```c
nrf24l01_init(&pins)
  ├─ GPIO 自检 (ce/csn/mosi/sclk/miso 全部手动翻转 + 读回)
  ├─ spi_bus_initialize(SPI2_HOST, ...)
  ├─ spi_bus_add_device(SPI2_HOST, mode=0, 1MHz, spics_io_num=-1)
  │     spics_io_num = -1  ← ★ 禁用硬件 CS
  ├─ nrf_write_reg(CONFIG, PWR_UP | PRIM_RX | EN_CRC)
  ├─ nrf_set_data_rate(NRF_RATE_2M)
  ├─ nrf_self_test()    ← 多个寄存器写后回读验证
  └─ nrf_dump_registers()
```

#### ⚠️ nRF24L01 注意事项

| 坑 | 根因 | 修复 |
|----|------|------|
| **SPI2 vs SD 冲突** | SD 也想占 SPI2 | SD 改 SPI3, NRF 独占 SPI2 |
| **CS 时序不稳** | 用硬件 CS 模式 | spics_io_num=-1, 手动 cs_low/cs_high |
| **RPD 永远 0** | CE 没进 RX, settle 太短 | ce_high() 后至少等 70µs |
| **自检失败** | SPI 时钟太快/线太长 | 初始 1MHz, 稳定后可试 8MHz |

---

### 5.3 A5133 + rf_5g — 5.8G 频谱扫描

**文件**：`a5133.c` (528 行), `rf_5g.c` (203 行)

#### 原理

A5133 是真正的频谱芯片，内置 PLL + RSSI ADC（寄存器 0x1E 的 bit[7:0]）。
datasheet：RSSI 范围 -95~-20dBm，8-bit ADC，约 0.3 dB/LSB。

#### GPIO bitbang SPI

A5133 的 SCLK=39, SDIO=34 (双向), CS=41 与 SDSPI 引脚冲突，**必须不用 SPI Host**：

```c
static void bb_sclk_high() { gpio_set_level(39, 1); esp_rom_delay_us(1); }
static void bb_sclk_low()  { gpio_set_level(39, 0); esp_rom_delay_us(1); }
static void bb_sdio_out(b) { gpio_set_level(34, b); }
static uint8_t bb_sdio_in(){ return gpio_get_level(34); }
static uint8_t bb_spi_rreg(uint8_t reg) { /* bitbang 读写 */ }
static void    bb_spi_wreg(uint8_t reg, uint8_t val) { /* bitbang 读写 */ }
```

#### 初始化 + 校准

```c
a5133_init()
  ├─ GPIO config (SCLK=OUT, SDIO=OUT+IN, CS=OUT, PA_EN/LNA_EN=OUT)
  ├─ 写 RF_CONFIG (PLL_I / PLL_Q / LO_I / LO_Q / MIXER / BB / ADC)
  ├─ a5133_calibrate()     ← ★ 关键! 必做
  │     a5133_write_reg(0x02, 0x23)    // 启动 IF/RSSI/RC 校准
  │     while (!(a5133_read_reg(0x03) & BIT_CAL_DONE)) {
  │         vTaskDelay(1);              // ★ 不能死等, 会触发 WDT
  │     }
  └─ a5133_set_rx(pa_en=0, lna_en=1)  // ★ LNA/PA 必须显式使能
```

#### RSSI 读取

```c
void a5133_set_freq_mhz(int mhz) {
    bb_spi_wreg(PLL_I, mhz_div_I);
    bb_spi_wreg(PLL_Q, mhz_div_Q);
    /* Strobe PLL → Strobe RX → wait 60µs (PLL→RX settling datasheet) */
}
uint8_t a5133_read_rssi_raw() {
    bb_spi_wreg(STROBE, STROBE_RX);
    esp_rom_delay_us(60);        // RSSI settle
    return bb_spi_rreg(REG_RSSI_ADC);  // 0x1E
}
```

#### ⚠️ A5133 踩坑清单

| 坑 | 根因 | 修复 |
|----|------|------|
| **PLL 校准超时** | 对只读 FIFO 寄存器 0x04 写 | 跳过只读寄存器, 只写 0x02 启动校准 |
| **RSSI 一直 0** | PA_EN=27/LNA_EN=10 没使能 | `a5133_set_rx/tx` 里显式 gpio_set_level |
| **SDIO 方向错** | bitbang 写入时 sdio 设成输入 | 切换 GPIO direction: 写→OUT, 读→IN (上拉) |
| **WDT 触发** | `while(!cal_done)` 死等 | 循环里 `vTaskDelay(1)` 让其他 task 跑 |

---

### 5.4 rf_link — UART 上行链路 (RID JSON)

**文件**：`rf_link.c` (1841 行)

#### 原理

外部设备（无人机侦测器/电脑模拟器）通过 UART 发送 **UDP 封装的 JSON**（ASTM F3411 Remote ID 标准）。
UDP 有 magic header，方便同步流中的包边界。

#### 数据流

```
UART ISR (RX)
  └─ rf_ringbuf_push(&s_uart_rb, byte)    // Producer
                                            │
rf_link_task (CPU1, 8KB)                    │
  └─ rf_proto_recv(&s_uart_rb, &pkt_ctx)  // Consumer
       │  找 UDP 头 DEADBEEF magic
       │  长度 / 校验 / 提取 payload
       ▼
     rf_json_find_key(udp_payload, "uas_id", ...)
     rf_json_find_key(..., "ua_lat")
     rf_json_find_key(..., "ua_lon")
     rf_json_find_key(..., "op_lat")
     rf_json_find_key(..., "op_lon")
       │
       ▼
     rid_registry_upsert(&drone)    // 更新/新增
     rid_registry_purge_expired()   // last_seen > 60s → 删除
```

#### 无锁 atomic 模式

Producer / Consumer 无锁：

```c
/* 写侧 */
memcpy(s_rssi, new_data, RF2G_CHANNELS);
__atomic_fetch_add(&s_version, 1, __ATOMIC_RELEASE);

/* 读侧 */
do {
    v0 = __atomic_load_n(&s_version, __ATOMIC_ACQUIRE);
    memcpy(out, s_rssi, RF2G_CHANNELS);
    v1 = __atomic_load_n(&s_version, __ATOMIC_ACQUIRE);
} while (v0 != v1);  // 写侧在 memcpy 中途改了 → 重试
```

#### 启动/停止

```c
/* 只有进入 RID 页面才启动 (节省 CPU) */
rf_link_start()   → xTaskCreatePinnedToCore(rf_link_task, CPU1)
rf_link_stop()    → 设 s_running=false, 等 task 退出
```

---

### 5.5 rf_gps — NMEA GPS 解析

**文件**：`rf_gps.c` (397 行)

#### 原理

GNSS 模块 (如 G7020UB) 通过 UART2 输出 NMEA-0183 字符串：

```
$GNGGA,143559.000,2904.5000,N,11314.2500,E,1,08,0.95,100.0,M,-10.0,M,,*47
  ↑      ↑        ↑             ↑       ↑  ↑  ↑
  时间    纬度     N/S           经度    E/W 定位质量=fix, fix=1/2 有效
```

转换为 7 位整数 (e-7 度, 避免浮点 UI 开销)：

```c
lat_e7 = 29 + 04.5000/60 = 29.075 → 290750000
```

#### 初始化

```c
rf_gps_init(uart_port, tx_pin, rx_pin, baud)
  ├─ uart_driver_install()
  ├─ xTaskCreatePinnedToCore(rf_gps_task, CPU1, prio 3, 4KB)
  │     while (s_running) {
  │         uart_read_bytes(..., 100ms)
  │         if (buf contains '$G') rf_gps_parse_line(line)
  │         rf_gps_publish()    // atomic store version
  │     }
  └─ return ESP_OK;
```

---

### 5.6 rf_ui — LVGL UI 框架

**文件**：`rf_ui.c` (3741 行)

#### 页面路由

```c
typedef enum { PAGE_HOME=0, PAGE_SPECTRUM, PAGE_RID, PAGE_RIDMAP, PAGE_SETTINGS } page_idx_t;

void page_show(PAGE_IDX new) {
    // 1. 退出清理: 如果从 RIDMAP 退出 → rf_map_exit_page() 刷 cache
    // 2. 隐藏所有页面 header/home button/状态栏
    // 3. 根据页面启停 scan tasks
    // 4. 显示目标页面组件
}
```

#### LVGL Canvas（瓦片地图）

```c
s_ridmap_canvas = lv_canvas_create(parent);
lv_canvas_set_buffer(canvas, buf, 448, 280, LV_COLOR_FORMAT_RGB565);
lv_obj_set_size(canvas, 448, 280);      // ★ 对象大小 = buffer 大小!
                                        // 否则 LVGL 会缩放 → 竖条纹!
```

#### Spectrum 合并视图（复用已有页面）

```c
build_page_spectrum()
  → lv_obj_set_parent(s_page_24g, spectrum_parent)  // 不是重新创建!
  → lv_obj_set_parent(s_page_58g, spectrum_parent)
  → lv_obj_set_size(s_page_24g, LV_SIZE_CONTENT, LV_SIZE_CONTENT)
  → lv_obj_set_align(s_page_24g, LV_ALIGN_TOP_LEFT, 12, 60)
  → 加 24G/58G toggle btn
```

为什么用 `lv_obj_set_parent` 而不是 `lv_obj_create`？因为 canvases 已经初始化好，reparent 复用 buffer 避免二次分配（之前 bug：冗余 canvas 分配导致 LVGL 堆崩）。

#### 触摸回调 (RID MAP)

```c
static void ridmap_touch_cb(lv_event_t *e) {
    if (code == LV_EVENT_PRESSED) {
        last_x = p.x; last_y = p.y;
        pending_pan = 0;
    } else if (code == LV_EVENT_PRESSING) {
        dx = p.x - last_x; dy = p.y - last_y;
        s_track_enabled = false;   // 手动拖 → 自动取消跟踪
        rf_map_pan(view, dx, dy);
        last_x = p.x; last_y = p.y;
    } else if (code == LV_EVENT_RELEASED) {
        rf_map_request_edges(view);  // 松手补全边缘
        pending_pan = 0;
    }
}
```

#### ⚠️ LVGL v9 注意事项

- `lv_canvas_set_buffer` v9 签名变了（最后一个参数是 `lv_color_format_t` 枚举）
- `lv_obj_set_size(canvas, w, h)` **必须和 buffer 尺寸一致**，否则 LVGL 会缩放
- flush flags 必须是 `LV_COLOR_FORMAT_RGB565`（我们用 RGB565 消除 flush 时的颜色转换）
- 不要频繁 delete/create 页面 → LVGL 堆会被打碎 (`lv_malloc_zeroed` 失败 → LV_ASSERT_MALLOC)

---

### 5.7 rid_registry — 无人机注册表

**文件**：`rid_registry.c` (155 行)

#### 原理

最大 32 个无人机槽位 (LRU)，60 秒过期自动清理。

```c
typedef struct {
    char  uas_id[24];
    int32_t lat_e7, lon_e7;          // 无人机位置
    int32_t op_lat_e7, op_lon_e7;    // 遥控器位置
    int64_t last_seen_ms;            // (esp_timer_get_time()/1000)
    bool    active;
} rf_drone_t;

// upsert: 新 ID → 找空槽/最旧槽; 已存在 → 更新 + 刷新 last_seen
// purge:  last_seen > 60000 → slot.active = false
```

---

### 5.8 rf_json / rf_proto / rf_ringbuf

| 模块 | 功能 | 原理 |
|------|------|------|
| **rf_json** (359 行) | 极简 JSON parser | 手写递归下降, 支持 string / number / bool / null, **零依赖** |
| **rf_proto** (205 行) | UART UDP 封装解析 | 找 magic header `DEADBEEF`, 长度 + payload 提取 |
| **rf_ringbuf** (46 行) | 环形缓冲区 | `head / tail / size`, `(tail+1)%size == head` 表示满 |

为什么不用 cJSON？因为 cJSON 太大 (~3KB flash)，而且我们只需要从 JSON 对象里按 key 提取值，手写 parser 更快更省。

---

## 六、关键设计决策与踩坑总结

| 决策 | 原因 |
|------|------|
| **瓦片渲染全同步** | 异步 render_task + mutex 竞争 → pan 每帧拿锁阻塞 decode → 死卡 |
| **3×3 而非 5×5** | 瓦片 decode 慢, 但 cache hit 后 memcpy 秒开, 3×3 足够覆盖 canvas |
| **SPI3_HOST 给 SD** | SPI2_HOST 留给 NRF24L01, 避免物理总线冲突 |
| **GPIO bitbang 给 A5133** | SDIO 引脚复用冲突, 只能手动翻 GPIO |
| **tile_slot += dx (同方向)** | 用户直觉: 手指向右拖 → 地图向右移 |
| **zoom 反推 canvas 中心地理坐标** | pan 后 center_lat/lon 已过期, 必须从 slot 反推 |
| **RPD 阈值 × 3 samples** | nRF24L01 无 RSSI ADC, 阈值检测 + 多次采样模拟多档 |
| **IDLE WDT CPU1 关闭** | scan task CPU1 prio3 纯 CPU 忙等会饿死 IDLE |
| **LVGL Canvas 对象大小 = buffer 大小** | v9 中不一致会触发 LVGL 缩放 → 竖条纹花屏 |
| **LVGL SPIRAM 初始化** | `CONFIG_LV_MEM_CUSTOM=y` + `lv_port_mem.c` 路由到 heap_caps_malloc(SPIRAM) |
| **Spectrum 页 reparent 复用** | 避免冗余 canvas 分配 → LVGL 堆崩 |

---

## 七、初始化流程总览

```
app_main()
  │
  ├─ nvs_flash_init()
  │
  ├─ ESP-VFS-FAT mount SD (SPI3_HOST)
  │     → rf_map_init()
  │
  ├─ 初始化硬件
  │     ├─ nrf24l01_init(&spi2_pins)    // SPI2_HOST, 手动 CS
  │     ├─ a5133_init()                 // GPIO bitbang
  │     ├─ a5133_calibrate()            // ★ PLL/RSSI 校准
  │     ├─ rf_gps_init(UART2, ...)      // CPU1 task
  │     └─ rf_link_init(UART, ...)      // CPU1 task
  │
  ├─ 初始化 LVGL UI
  │     ├─ lvgl_port_init()
  │     ├─ build_page_home()
  │     ├─ build_page_spectrum()  (含 24G + 5.8G canvases)
  │     ├─ build_page_rid()
  │     ├─ build_page_ridmap()
  │     └─ build_page_settings()
  │
  ├─ page_show(PAGE_HOME)
  │
  ├─ lvgl_port_task_start()   // CPU0, prio 4
  │
  └─ return                    // FreeRTOS 调度器接管
```

---

## 八、sdkconfig 关键配置

```ini
# === 双核心 + PSRAM ===
CONFIG_ESP32P4_DEFAULT_CPU_FREQ_MHZ=400
CONFIG_ESP32P4_INSTRUCTION_SET_MARCH=100
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_QUAD=y
CONFIG_SPIRAM_SPEED_80M=y

# === Task WDT ===
CONFIG_ESP_TASK_WDT=y
CONFIG_ESP_TASK_WDT_TIMEOUT_S=15
CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y
CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=n   # ★ CPU1 scan 纯 CPU 忙等

# === LVGL ===
CONFIG_LV_USE_PPA=y                         # P4 PPA 硬件加速
CONFIG_LV_USE_PPA_IMG=y
CONFIG_LV_USE_GPU_DMA2D=n                   # P4 用 PPA 不用 DMA2D
CONFIG_LV_MEM_SIZE=65536                    # LVGL 堆大小
CONFIG_LV_MEM_CUSTOM=y                      # 用 SPIRAM

# === SDMMC ===
CONFIG_SD_SPI_PINS_CONTAINER_SIZE=16
CONFIG_SPI_FLASH_SPIRAM_GDMA_TRANSFER=y
```

---

*文档生成于 2026-10-02 · 对应项目 commit `2c92541`*
