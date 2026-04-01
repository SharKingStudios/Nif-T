#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>

// ==================== BRANDING / USER CONFIG ====================
static const char* PRODUCT_NAME = "Nif-T";
static const char* HOSTNAME     = "nif-t";

static const char* WIFI_SSID = "wifiname";
static const char* WIFI_PASS = "password";

static const bool RELAY_ACTIVE_LOW = false;

#define ENABLE_WIFI      1
#define ENABLE_ETHERNET  0
#define ENABLE_BUZZER_DEFAULT 1
#define ENABLE_LEDS_DEFAULT   1

// -------------------- PIN MAP --------------------
static const int PIN_ETH_POWER_OR_RESET = 5;
static const int PIN_ETH_MDC = 23;
static const int PIN_ETH_MDIO = 18;
static const int PIN_ETH_ADDR = 0;

static const int PIN_LED_DATA = 17;
static const int LED_COUNT    = 10;
static const int PIN_BUTTON   = 34;   // external pull-up: pressed = LOW
static const int PIN_BUZZER   = 16;

static const int PIN_RLY1 = 32;
static const int PIN_RLY2 = 33;
static const int PIN_RLY3 = 13;
static const int PIN_RLY4 = 14;
static const int PIN_RLY5 = 2;
static const int PIN_RLY6 = 4;
static const int PIN_RLY7 = 15;
static const int PIN_RLY8 = 16; // shared with buzzer

static const int PIN_AUX1 = 37;
static const int PIN_AUX2 = 36;
static const int PIN_AUX3 = 35;

#if ENABLE_ETHERNET
  #define ETH_PHY_TYPE  ETH_PHY_LAN8720
  #define ETH_PHY_ADDR  PIN_ETH_ADDR
  #define ETH_PHY_MDC   PIN_ETH_MDC
  #define ETH_PHY_MDIO  PIN_ETH_MDIO
  #define ETH_PHY_POWER -1
  #define ETH_CLK_MODE  ETH_CLOCK_GPIO0_IN
  #include <ETH.h>
#endif

#define PIXEL_TYPE (NEO_GRB + NEO_KHZ800)
Adafruit_NeoPixel strip(LED_COUNT, PIN_LED_DATA, PIXEL_TYPE);
WebServer server(80);

// ==================== SETTINGS ====================
static const uint8_t  LED_BRIGHTNESS        = 180;
static const uint32_t FRAME_MS              = 25;
static const uint32_t BUTTON_DEBOUNCE_MS    = 30;
static const uint32_t MULTICLICK_TIMEOUT_MS = 420;
static const uint8_t  RICKROLL_CLICK_COUNT  = 5;
static const uint32_t WIFI_RETRY_MS         = 15000;
static const uint32_t WIFI_FAILOVER_MS      = 10000;
static const bool     DEFAULT_SHARED_LINES_ON = true;

// Fallback hotspot
static const char* FALLBACK_AP_SSID = "Nif-T";
static const char* FALLBACK_AP_PASS = "17171717";

// ==================== EFFECTS ====================
enum EffectMode : uint8_t {
  MODE_RAINBOW_PHYSICAL = 0,
  MODE_PURPLE_FIRE      = 1,
  MODE_COUNT
};

EffectMode effectMode = MODE_RAINBOW_PHYSICAL;

// Physical LED layout
// Rows top -> bottom:
// y=0:  5, 2
// y=1:  6, 1
// y=2:  7, 3
// y=3:  8, 4
// y=4:  9,10
static const int8_t physToIndex[5][2] = {
  {4, 1},
  {5, 0},
  {6, 2},
  {7, 3},
  {8, 9}
};

static const uint8_t ledX[LED_COUNT] = {1,1,1,1,0,0,0,0,0,1};
static const uint8_t ledY[LED_COUNT] = {1,0,2,3,0,1,2,3,4,4};

static const uint8_t rowLevelPct[5] = {4, 8, 18, 42, 100};
uint8_t heat[5][2] = {0};

// ==================== BREAKOUT / DEVICE MODEL ====================
enum DeviceType : uint8_t {
  DEV_DISABLED    = 0,
  DEV_DIGITAL_OUT = 1,
  DEV_PWM         = 2,
  DEV_SERVO       = 3,
  DEV_INPUT       = 4
};

enum RelayMirrorState : uint8_t {
  RELAY_RESERVED  = 0,
  RELAY_UNMANAGED = 1,
  RELAY_OFF       = 2,
  RELAY_ON        = 3,
  RELAY_DRIVEN    = 4
};

struct PinMeta {
  uint8_t pin;
  const char* defaultLabel;
  bool inputOnly;
  int8_t relayIndex;
  bool ledShared;
  bool buzzerShared;
};

static const PinMeta PIN_META[] = {
  { 2,  "GPIO2",  false, 4, false, false },
  { 4,  "GPIO4",  false, 5, false, false },
  { 15, "GPIO15", false, 6, false, false },
  { 16, "GPIO16", false, 7, false, true  },
  { 17, "GPIO17", false, -1, true, false },
  { 13, "GPIO13", false, 2, false, false },
  { 14, "GPIO14", false, 3, false, false },
  { 32, "GPIO32", false, 0, false, false },
  { 33, "GPIO33", false, 1, false, false },
  { 37, "GPIO37", true,  -1, false, false },
  { 36, "GPIO36", true,  -1, false, false },
  { 35, "GPIO35", true,  -1, false, false }
};

static const uint8_t PIN_DEVICE_COUNT = sizeof(PIN_META) / sizeof(PIN_META[0]);

struct PinDevice {
  uint8_t pin;
  char label[24];
  DeviceType type;
  int value;
  int freq;
  bool inputOnly;
  int8_t relayIndex;
  bool ledShared;
  bool buzzerShared;
  bool pwmAttached;
};

PinDevice devices[PIN_DEVICE_COUNT];

static const uint8_t relayPins[8] = {
  PIN_RLY1, PIN_RLY2, PIN_RLY3, PIN_RLY4,
  PIN_RLY5, PIN_RLY6, PIN_RLY7, PIN_RLY8
};

// ==================== NETWORK STATE ====================
#if ENABLE_ETHERNET
volatile bool ethConnected = false;
#endif

uint32_t lastWiFiAttemptMs = 0;
uint32_t wifiConnectStartMs = 0;
bool apFallbackActive = false;

// ==================== SYSTEM FLAGS ====================
bool ledEngineEnabled = ENABLE_LEDS_DEFAULT;
bool buzzerEnabled    = ENABLE_BUZZER_DEFAULT;

// ==================== BUTTON STATE ====================
int btnLastReading = HIGH;
int btnStableState = HIGH;
uint32_t btnLastChangeMs = 0;
uint8_t pendingClicks = 0;
uint32_t lastClickReleaseMs = 0;

// ==================== TIMING ====================
uint32_t lastFrameMs = 0;
uint16_t tick = 0;

// ==================== NOTES ====================
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

struct TunePlayer {
  const int* data = nullptr;
  size_t length = 0;
  int tempo = 120;
  size_t index = 0;
  bool active = false;
  bool toneOn = false;
  uint32_t noteOffAtMs = 0;
  uint32_t nextStepAtMs = 0;
  const char* name = "idle";
} sound;

#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

const int bootTune[] = {
  NOTE_E5,16, NOTE_G5,16, NOTE_B5,16, NOTE_E6,10,
  REST,32,
  NOTE_D6,16, NOTE_E6,10
};

const int clickTune[] = {
  NOTE_G5,20, NOTE_C6,20, NOTE_E6,14
};

const int rickTune[] = {
  NOTE_D5,-4, NOTE_E5,-4, NOTE_A4,4,
  NOTE_E5,-4, NOTE_FS5,-4, NOTE_A5,16, NOTE_G5,16, NOTE_FS5,8,
  NOTE_D5,-4, NOTE_E5,-4, NOTE_A4,2,
  NOTE_A4,16, NOTE_A4,16, NOTE_B4,16, NOTE_D5,8, NOTE_D5,16,

  NOTE_D5,-4, NOTE_E5,-4, NOTE_A4,4,
  NOTE_E5,-4, NOTE_FS5,-4, NOTE_A5,16, NOTE_G5,16, NOTE_FS5,8,
  NOTE_D5,-4, NOTE_E5,-4, NOTE_A4,2,
  NOTE_A4,16, NOTE_A4,16, NOTE_B4,16, NOTE_D5,8, NOTE_D5,16,

  REST,4, NOTE_B4,8, NOTE_CS5,8, NOTE_D5,8, NOTE_D5,8, NOTE_E5,8, NOTE_CS5,-8,
  NOTE_B4,16, NOTE_A4,2, REST,4,

  REST,8, NOTE_B4,8, NOTE_B4,8, NOTE_CS5,8, NOTE_D5,8, NOTE_B4,4, NOTE_A4,8,
  NOTE_A5,8, REST,8, NOTE_A5,8, NOTE_E5,-4, REST,4,

  NOTE_B4,8, NOTE_B4,8, NOTE_CS5,8, NOTE_D5,8, NOTE_B4,8, NOTE_D5,8, NOTE_E5,8, REST,8,
  REST,8, NOTE_CS5,8, NOTE_B4,8, NOTE_A4,-4, REST,4,

  NOTE_B4,8, NOTE_B4,8, NOTE_CS5,8, NOTE_D5,8, NOTE_B4,8, NOTE_A4,4,
  NOTE_E5,8, NOTE_E5,8, NOTE_E5,8, NOTE_FS5,8, NOTE_E5,4, REST,4,

  NOTE_D5,2, NOTE_E5,8, NOTE_FS5,8, NOTE_D5,8,
  NOTE_E5,8, NOTE_E5,8, NOTE_E5,8, NOTE_FS5,8, NOTE_E5,4, NOTE_A4,4,

  REST,2, NOTE_B4,8, NOTE_CS5,8, NOTE_D5,8, NOTE_B4,8,
  REST,8, NOTE_E5,8, NOTE_FS5,8, NOTE_E5,-4, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,

  NOTE_FS5,-8, NOTE_FS5,-8, NOTE_E5,-4, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
  NOTE_E5,-8, NOTE_E5,-8, NOTE_D5,-8, NOTE_CS5,16, NOTE_B4,-8, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,

  NOTE_D5,4, NOTE_E5,8, NOTE_CS5,-8, NOTE_B4,16, NOTE_A4,8, NOTE_A4,8, NOTE_A4,8,
  NOTE_E5,4, NOTE_D5,2, REST,4
};

// ==================== HTML ====================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Nif-T Admin</title>
<style>
:root{
  --bg:#141416;
  --bg2:#19191c;
  --surface:#202126;
  --surface2:#26272d;
  --surface3:#2b2d34;
  --line:rgba(255,255,255,.08);
  --lineStrong:rgba(255,255,255,.15);
  --text:#f5f5f5;
  --muted:#a6a6af;
  --primary:#e6d3ab;
  --primaryStrong:#f3e4c2;
  --primaryInk:#1f1b14;
  --good:#69d08e;
  --bad:#fb7185;
  --warn:#f4c76b;
  --soft:#8fb4ff;
  --shadow:0 22px 60px rgba(0,0,0,.28);
  --radius:20px;
  --radiusSm:14px;
  --max:1340px;
}
*{box-sizing:border-box}
html,body{margin:0;padding:0}
body{
  min-height:100vh;
  color:var(--text);
  font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;
  letter-spacing:.02em;
  background:
    radial-gradient(1100px 560px at 0% -8%, rgba(230,211,171,.12), transparent 48%),
    radial-gradient(900px 520px at 100% 0%, rgba(255,255,255,.04), transparent 42%),
    linear-gradient(180deg,var(--bg) 0%, var(--bg2) 100%);
}
button,input,select{font:inherit}
a{color:inherit}
.shell{max-width:var(--max);margin:0 auto;padding:24px 18px 34px}
.topbar{
  display:flex;justify-content:space-between;align-items:flex-start;gap:18px;flex-wrap:wrap;
  margin-bottom:18px;
}
.brand{display:flex;flex-direction:column;gap:10px}
.eyebrow{
  display:inline-flex;align-items:center;gap:10px;width:max-content;
  padding:8px 12px;border-radius:999px;
  background:rgba(255,255,255,.04);
  border:1px solid var(--line);
  color:var(--muted);
  font-size:12px;text-transform:uppercase;letter-spacing:.14em
}
.dot{
  width:8px;height:8px;border-radius:999px;background:var(--primary);
  box-shadow:0 0 18px rgba(230,211,171,.72)
}
h1{
  margin:0;
  font-size:clamp(34px,6vw,62px);
  line-height:1;
  letter-spacing:-.05em;
  font-weight:800
}
.sub{
  margin:0;max-width:760px;
  color:var(--muted);
  font-size:15px;line-height:1.65
}
.pillRow{display:flex;gap:10px;flex-wrap:wrap}
.pill{
  min-width:160px;
  padding:12px 14px;
  border-radius:18px;
  background:rgba(255,255,255,.045);
  border:1px solid var(--line);
  box-shadow:var(--shadow);
  backdrop-filter:blur(10px)
}
.pillLabel{
  color:var(--muted);
  font-size:11px;
  text-transform:uppercase;
  letter-spacing:.14em
}
.pillValue{
  margin-top:6px;
  font-size:18px;
  font-weight:760;
  letter-spacing:-.02em
}
.grid{display:grid;grid-template-columns:repeat(12,1fr);gap:14px}
.card{
  grid-column:span 12;
  padding:18px;
  border-radius:var(--radius);
  background:linear-gradient(180deg,rgba(255,255,255,.055),rgba(255,255,255,.04));
  border:1px solid var(--line);
  box-shadow:var(--shadow);
  backdrop-filter:blur(12px)
}
@media(min-width:960px){
  .span-4{grid-column:span 4}
  .span-5{grid-column:span 5}
  .span-6{grid-column:span 6}
  .span-7{grid-column:span 7}
  .span-8{grid-column:span 8}
}
.cardHead{
  display:flex;justify-content:space-between;align-items:flex-start;gap:12px;flex-wrap:wrap;
  margin-bottom:14px
}
.cardTitle{
  margin:0;
  font-size:12px;
  color:var(--muted);
  text-transform:uppercase;
  letter-spacing:.14em
}
.big{
  margin:0;
  font-size:28px;
  font-weight:800;
  letter-spacing:-.04em;
  line-height:1.08
}
.desc{
  margin:8px 0 0;
  color:var(--muted);
  font-size:14px;
  line-height:1.6
}
.live{
  display:inline-flex;align-items:center;gap:8px;
  font-size:13px;color:var(--muted)
}
.liveDot{
  width:7px;height:7px;border-radius:999px;background:var(--primary);
  box-shadow:0 0 14px rgba(230,211,171,.7)
}
.metricGrid,.controlGrid,.pinGrid,.inputGrid,.netGrid{
  display:grid;gap:12px
}
.metricGrid{grid-template-columns:repeat(auto-fit,minmax(150px,1fr))}
.controlGrid{grid-template-columns:repeat(auto-fit,minmax(180px,1fr))}
.pinGrid{grid-template-columns:repeat(auto-fit,minmax(290px,1fr))}
.inputGrid{grid-template-columns:repeat(auto-fit,minmax(170px,1fr))}
.netGrid{grid-template-columns:repeat(auto-fit,minmax(220px,1fr))}
.metric,.pinCard,.inputCard,.statusBox{
  padding:14px;
  border-radius:18px;
  background:rgba(255,255,255,.04);
  border:1px solid var(--line)
}
.metricKey,.tinyLabel,label,.smallCaps{
  color:var(--muted);
  font-size:11px;
  letter-spacing:.14em;
  text-transform:uppercase
}
.metricVal{
  margin-top:8px;
  font-size:22px;
  font-weight:780;
  letter-spacing:-.03em
}
.row{display:flex;gap:10px;flex-wrap:wrap}
.stack{display:flex;flex-direction:column;gap:8px}
.inlineSplit{
  display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap
}
.badges{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
.badge{
  display:inline-flex;align-items:center;gap:8px;
  padding:8px 12px;border-radius:999px;
  font-size:12px;font-weight:760;
  border:1px solid transparent;
  white-space:nowrap
}
.badge.on{background:rgba(105,208,142,.12);color:#b2f0c8;border-color:rgba(105,208,142,.24)}
.badge.off{background:rgba(251,113,133,.11);color:#ffc2ce;border-color:rgba(251,113,133,.22)}
.badge.warn{background:rgba(244,199,107,.13);color:#ffe0a1;border-color:rgba(244,199,107,.25)}
.badge.soft{background:rgba(143,180,255,.12);color:#cadcff;border-color:rgba(143,180,255,.22)}
.badge.gold{background:rgba(230,211,171,.14);color:#f8eccf;border-color:rgba(230,211,171,.25)}
.btn{
  appearance:none;border:none;cursor:pointer;
  padding:13px 15px;border-radius:16px;
  color:var(--text);
  background:linear-gradient(180deg,var(--surface3),var(--surface));
  border:1px solid var(--line);
  transition:transform .15s ease, filter .15s ease, border-color .15s ease;
  font-weight:740
}
.btn:hover{transform:translateY(-1px);filter:brightness(1.04);border-color:var(--lineStrong)}
.btn.primary{
  color:var(--primaryInk);
  background:linear-gradient(180deg,var(--primaryStrong),var(--primary));
  border-color:rgba(230,211,171,.38)
}
.btn.warn{background:linear-gradient(180deg,rgba(244,199,107,.22),rgba(244,199,107,.12))}
.btn.bad{background:linear-gradient(180deg,rgba(251,113,133,.2),rgba(251,113,133,.12))}
.btn:disabled{opacity:.58;cursor:not-allowed;transform:none!important}
.field{display:flex;flex-direction:column;gap:7px;margin-top:12px}
input[type="text"],input[type="number"],select{
  width:100%;
  padding:11px 12px;
  border-radius:14px;
  border:1px solid var(--line);
  background:rgba(0,0,0,.16);
  color:var(--text);
  outline:none
}
input[type="text"]:focus,input[type="number"]:focus,select:focus{
  border-color:rgba(230,211,171,.48);
  box-shadow:0 0 0 3px rgba(230,211,171,.08)
}
input[type="range"]{width:100%}
.footer{
  margin-top:12px;
  color:var(--muted);
  font-size:13px;
  line-height:1.6
}
.small{font-size:13px;color:var(--muted)}
.pinTop{
  display:flex;justify-content:space-between;align-items:flex-start;gap:10px
}
.pinName{
  margin:0;
  font-size:21px;
  font-weight:780;
  letter-spacing:-.03em
}
.pinMeta{
  margin-top:4px;
  color:var(--muted);
  font-size:13px;
  line-height:1.45
}
.kv{
  display:flex;justify-content:space-between;align-items:center;gap:10px;
  padding:10px 12px;
  border-radius:12px;
  background:rgba(0,0,0,.15);
  border:1px solid rgba(255,255,255,.05)
}
.kv strong{font-size:14px}
.messageBox{
  min-height:48px;
  margin-top:14px;
  padding:12px 14px;
  border-radius:16px;
  border:1px solid var(--line);
  background:rgba(255,255,255,.035);
  display:flex;align-items:center;gap:10px;
  color:var(--muted)
}
.messageBox.good{border-color:rgba(105,208,142,.28);background:rgba(105,208,142,.08);color:#c8f5d6}
.messageBox.bad{border-color:rgba(251,113,133,.28);background:rgba(251,113,133,.08);color:#ffd1da}
.messageBox.warn{border-color:rgba(244,199,107,.28);background:rgba(244,199,107,.08);color:#ffe7b6}
.messageBox.soft{border-color:rgba(143,180,255,.24);background:rgba(143,180,255,.06);color:#dce7ff}
.messageBox.idle{color:var(--muted)}
pre{
  margin:0;padding:14px;
  border-radius:16px;
  border:1px solid var(--line);
  background:rgba(0,0,0,.18);
  color:#e8e8ea;
  overflow:auto;
  font-size:12px;
  line-height:1.55
}
details{
  border-radius:18px;
  border:1px solid var(--line);
  background:rgba(255,255,255,.025)
}
summary{
  list-style:none;
  cursor:pointer;
  padding:14px 16px;
  display:flex;align-items:center;justify-content:space-between;gap:12px
}
summary::-webkit-details-marker{display:none}
.summaryText{display:flex;flex-direction:column;gap:6px}
.summaryText strong{font-size:15px}
.summaryText span{font-size:13px;color:var(--muted)}
.detailsBody{padding:0 16px 16px}
.switch{
  position:relative;display:inline-flex;align-items:center
}
.switch input{position:absolute;opacity:0;pointer-events:none}
.switch span{
  width:48px;height:28px;border-radius:999px;
  background:rgba(255,255,255,.14);
  border:1px solid var(--line);
  display:inline-block;position:relative;transition:.18s ease
}
.switch span::after{
  content:"";
  position:absolute;top:3px;left:3px;
  width:20px;height:20px;border-radius:999px;background:#fff;
  transition:.18s ease
}
.switch input:checked + span{
  background:rgba(230,211,171,.34);
  border-color:rgba(230,211,171,.35)
}
.switch input:checked + span::after{transform:translateX(20px)}
.sectionLead{margin-top:4px;color:var(--muted);font-size:14px;line-height:1.55}
.hiddenWrap{display:grid;gap:12px;margin-top:4px}
</style>
</head>
<body>
<div class="shell">
  <section class="topbar">
    <div class="brand">
      <div class="eyebrow"><span class="dot"></span> Nif-T Operations Panel</div>
      <h1>Nif-T</h1>
      <p class="sub">
        Clean device administration for live status, GPIO control, shared relay output mapping,
        and diagnostics.
      </p>
    </div>
    <div class="pillRow">
      <div class="pill">
        <div class="pillLabel">Host</div>
        <div class="pillValue" id="hostPill">--</div>
      </div>
      <div class="pill">
        <div class="pillLabel">Uptime</div>
        <div class="pillValue" id="uptimePill">--</div>
      </div>
      <div class="pill">
        <div class="pillLabel">Display</div>
        <div class="pillValue" id="displayPill">--</div>
      </div>
    </div>
  </section>

  <section class="grid">
    <article class="card span-8">
      <div class="cardHead">
        <div>
          <h2 class="cardTitle">Overview</h2>
          <h3 class="big" id="effectName">--</h3>
          <p class="desc">Live runtime state, system engines, current presentation mode, and network status.</p>
        </div>
        <div class="live"><span class="liveDot"></span><span>Live refresh every 200 ms</span></div>
      </div>

      <div class="metricGrid">
        <div class="metric">
          <div class="metricKey">Visual Mode</div>
          <div class="metricVal" id="visualModeMetric">--</div>
        </div>
        <div class="metric">
          <div class="metricKey">Audio</div>
          <div class="metricVal" id="audioMetric">--</div>
        </div>
        <div class="metric">
          <div class="metricKey">LED Engine</div>
          <div class="metricVal" id="ledEngineMetric">--</div>
        </div>
        <div class="metric">
          <div class="metricKey">Buzzer Engine</div>
          <div class="metricVal" id="buzzerEngineMetric">--</div>
        </div>
        <div class="metric">
          <div class="metricKey">Buttons</div>
          <div class="metricVal">5-click Rickroll</div>
        </div>
        <div class="metric">
          <div class="metricKey">Last Sync</div>
          <div class="metricVal" id="syncMetric">--</div>
        </div>
      </div>

      <div class="netGrid" style="margin-top:12px">
        <div class="statusBox">
          <div class="inlineSplit">
            <div>
              <div class="smallCaps">Ethernet</div>
              <div style="margin-top:8px;font-size:20px;font-weight:760">LAN</div>
            </div>
            <div id="ethTag"></div>
          </div>
          <div class="footer" id="ethIp">IP: --</div>
        </div>
        <div class="statusBox">
          <div class="inlineSplit">
            <div>
              <div class="smallCaps">Wi-Fi</div>
              <div style="margin-top:8px;font-size:20px;font-weight:760">WLAN</div>
            </div>
            <div id="wifiTag"></div>
          </div>
          <div class="footer" id="wifiSsid">SSID: --</div>
          <div class="footer" id="wifiIp">IP: --</div>
        </div>
      </div>

      <div class="messageBox idle" id="overviewMessage">Ready.</div>
    </article>

    <article class="card span-4">
      <div class="cardHead">
        <div>
          <h2 class="cardTitle">Controls</h2>
          <h3 class="big">Quick Actions</h3>
        </div>
      </div>
      <div class="controlGrid">
        <button class="btn primary" onclick="setEffect(0)">Rainbow Physical</button>
        <button class="btn primary" onclick="setEffect(1)">Purple Fire</button>
        <button class="btn" onclick="playRick()">Play Rickroll</button>
        <button class="btn" id="ledToggleBtn" onclick="toggleLeds()">Toggle LEDs</button>
        <button class="btn" id="buzzerToggleBtn" onclick="toggleBuzzer()">Toggle Buzzer</button>
        <button class="btn warn" onclick="stopAudio()">Stop Audio</button>
      </div>
      <div class="footer">
        GPIO17 stays reserved while the LED engine is enabled. GPIO16 stays reserved while the
        buzzer is enabled. Hold the hardware button to preview relay and network LED status.
      </div>
    </article>

    <article class="card span-8">
      <div class="cardHead">
        <div>
          <h2 class="cardTitle">Outputs</h2>
          <h3 class="big">Configurable GPIO</h3>
          <p class="sectionLead">Changes apply immediately. Labels save after half a second of no typing.</p>
        </div>
      </div>
      <div class="pinGrid" id="pinGrid"></div>
    </article>

    <article class="card span-4">
      <div class="cardHead">
        <div>
          <h2 class="cardTitle">Inputs</h2>
          <h3 class="big">Monitor Only</h3>
          <p class="sectionLead">Input-only pins stay here and continuously show their live values.</p>
        </div>
      </div>
      <div class="inputGrid" id="inputGrid"></div>
    </article>

    <article class="card span-12">
      <details id="systemPinsPanel">
        <summary>
          <div class="summaryText">
            <strong>Reserved System Pins</strong>
            <span>GPIO16 and GPIO17 are tucked away here unless you actually need them.</span>
          </div>
          <span class="badge gold">Hidden by default</span>
        </summary>
        <div class="detailsBody">
          <div class="hiddenWrap">
            <div class="pinGrid" id="systemPinGrid"></div>
          </div>
        </div>
      </details>
    </article>

    <article class="card span-12">
      <details id="diagPanel">
        <summary>
          <div class="summaryText">
            <strong>Diagnostics</strong>
            <span>Raw JSON only refreshes while this panel is open.</span>
          </div>
          <span class="badge gold">Verbose</span>
        </summary>
        <div class="detailsBody">
          <pre id="rawJson">{}</pre>
        </div>
      </details>
    </article>
  </section>
</div>

<script>
const POLL_MS = 200;
const LABEL_DEBOUNCE_MS = 500;

let statusBusy = false;
let initialized = false;
let lastData = null;
let lastSyncAt = 0;
let lastOutputLayoutKey = "";
let lastSystemLayoutKey = "";
let lastInputLayoutKey = "";
let lastDiagAt = 0;

const labelTimers = {};
const valueTimers = {};
const freqTimers = {};

function esc(s){
  return String(s ?? "")
    .replaceAll("&","&amp;")
    .replaceAll("<","&lt;")
    .replaceAll(">","&gt;")
    .replaceAll('"',"&quot;");
}

async function api(url){
  const r = await fetch(url, {cache:"no-store"});
  let data = null;
  const ct = r.headers.get("content-type") || "";
  if(ct.includes("application/json")){
    try { data = await r.json(); } catch (_) {}
  } else {
    try { data = await r.text(); } catch (_) {}
  }
  if(!r.ok){
    const msg = (data && data.error) ? data.error : ("HTTP " + r.status);
    throw new Error(msg);
  }
  return data;
}

function badge(kind, text){
  return `<span class="badge ${kind}">${esc(text)}</span>`;
}

function statusBadgeForPin(pin){
  if(pin.type === "disabled") return {kind:"off", text:"Disabled"};
  if(pin.type === "input") return {kind:"soft", text:"Input"};
  if(pin.type === "digital_out") return {kind:(pin.value > 0 ? "on" : "off"), text:(pin.value > 0 ? "High" : "Low")};
  if(pin.type === "pwm") return {kind:"on", text:"PWM"};
  return {kind:"on", text:"Servo"};
}

function relayBadgeState(relay){
  if(!relay) return {kind:"soft", text:"No Relay"};
  if(relay.state === "reserved") return {kind:"warn", text:"Reserved"};
  if(relay.state === "unmanaged") return {kind:"soft", text:"Unmanaged"};
  if(relay.state === "driven") return {kind:"warn", text:"Driven"};
  if(relay.state === "on") return {kind:"on", text:"On"};
  return {kind:"off", text:"Off"};
}

function groupsFromData(data){
  const outputs = [];
  const systemPins = [];
  const inputs = [];

  for(const pin of data.pins){
    if(pin.inputOnly){
      inputs.push(pin);
      continue;
    }
    if(pin.pin === 16 || pin.pin === 17){
      systemPins.push(pin);
      continue;
    }
    outputs.push(pin);
  }

  return {outputs, systemPins, inputs};
}

function relayForPin(pin){
  if(!lastData) return null;
  return lastData.relays.find(r => r.pin === pin.pin) || null;
}

function pinBadges(pin){
  const bits = [];
  if(pin.ledShared) bits.push(badge(pin.locked ? "warn" : "soft","LED Data"));
  if(pin.buzzerShared) bits.push(badge(pin.locked ? "warn" : "soft","Buzzer"));
  if(pin.locked && pin.lockReason) bits.push(badge("warn", pin.lockReason));
  return bits.join("");
}

function modeOptions(pin){
  const opts = [];
  opts.push(`<option value="disabled"${pin.type==="disabled"?" selected":""}>Disabled</option>`);
  if(!pin.inputOnly){
    opts.push(`<option value="digital_out"${pin.type==="digital_out"?" selected":""}>Digital Output</option>`);
    opts.push(`<option value="pwm"${pin.type==="pwm"?" selected":""}>PWM Output</option>`);
    opts.push(`<option value="servo"${pin.type==="servo"?" selected":""}>Servo Output</option>`);
  }
  return opts.join("");
}

function outputControls(pin){
  if(pin.type === "digital_out"){
    const checked = pin.value > 0 ? "checked" : "";
    return `
      <div class="field">
        <label>State</label>
        <div class="row" style="align-items:center">
          <label class="switch">
            <input type="checkbox" id="pinValue${pin.id}" ${checked}
              onchange="pushDigitalValue(${pin.id}, this.checked)">
            <span></span>
          </label>
          <span class="small" id="pinValueText${pin.id}">${pin.value > 0 ? "High / On" : "Low / Off"}</span>
        </div>
      </div>
    `;
  }

  if(pin.type === "pwm"){
    return `
      <div class="field">
        <label>Frequency (Hz)</label>
        <input type="number" id="pinFreq${pin.id}" min="1" max="40000" value="${pin.freq}"
          onchange="queueFreqSave(${pin.id})">
      </div>
      <div class="field">
        <label>PWM Duty</label>
        <input type="range" id="pinValue${pin.id}" min="0" max="1023" value="${pin.value}"
          oninput="document.getElementById('pinValueText${pin.id}').textContent=this.value; queueRangeValue(${pin.id}, this.value)">
        <div class="small">Value: <span id="pinValueText${pin.id}">${pin.value}</span> / 1023</div>
      </div>
    `;
  }

  if(pin.type === "servo"){
    return `
      <div class="field">
        <label>Servo Angle</label>
        <input type="range" id="pinValue${pin.id}" min="0" max="180" value="${pin.value}"
          oninput="document.getElementById('pinValueText${pin.id}').textContent=this.value; queueRangeValue(${pin.id}, this.value)">
        <div class="small">Angle: <span id="pinValueText${pin.id}">${pin.value}</span>°</div>
      </div>
    `;
  }

  return `
    <div class="field">
      <label>Output</label>
      <div class="small">No active function assigned to this pin.</div>
    </div>
  `;
}

function relayInfoBlock(pin){
  const relay = relayForPin(pin);
  if(!relay){
    return `<div class="kv"><span>Attached Relay</span><strong>None</strong></div>`;
  }
  const rs = relayBadgeState(relay);
  return `
    <div class="kv">
      <span>Attached Relay</span>
      <div class="row" style="align-items:center;justify-content:flex-end">
        <strong>${esc(relay.label)}</strong>
        <span class="badge ${rs.kind}" id="relayStateForPin${pin.id}">${rs.text}</span>
      </div>
    </div>
  `;
}

function pinCard(pin){
  const s = statusBadgeForPin(pin);
  return `
    <div class="pinCard" id="pinCard${pin.id}">
      <div class="pinTop">
        <div>
          <h3 class="pinName">GPIO ${pin.pin}</h3>
          <div class="pinMeta" id="pinMeta${pin.id}">Current: ${esc(pin.label)}</div>
        </div>
        <span class="badge ${s.kind}" id="pinStatus${pin.id}">${s.text}</span>
      </div>

      ${relayInfoBlock(pin)}

      <div class="badges" id="pinBadges${pin.id}">${pinBadges(pin)}</div>

      <div class="field">
        <label>Label</label>
        <input type="text" id="pinLabel${pin.id}" maxlength="22" value="${esc(pin.label)}"
          oninput="queueLabelSave(${pin.id})">
      </div>

      <div class="field">
        <label>Mode</label>
        <select id="pinType${pin.id}" onchange="changePinType(${pin.id})">
          ${modeOptions(pin)}
        </select>
      </div>

      <div id="pinControls${pin.id}">
        ${outputControls(pin)}
      </div>

      <div class="footer" id="pinNote${pin.id}">${esc(pin.note)}</div>
    </div>
  `;
}

function systemPinCard(pin){
  return pinCard(pin);
}

function inputCard(pin){
  return `
    <div class="inputCard" id="inputCard${pin.id}">
      <div class="tinyLabel">GPIO ${pin.pin}</div>
      <div class="metricVal" style="font-size:20px">${esc(pin.label)}</div>
      <div class="stack" style="margin-top:12px">
        <div class="kv"><span>Digital</span><strong id="pinDigital${pin.id}">${pin.digitalRead}</strong></div>
        <div class="kv"><span>Analog</span><strong id="pinAnalog${pin.id}">${pin.analogRead}</strong></div>
      </div>
    </div>
  `;
}

function outputsLayoutKey(pins){
  return pins.map(pin => [
    pin.id,pin.pin,pin.type,pin.locked,pin.lockReason,pin.ledShared,pin.buzzerShared,pin.note
  ].join(":")).join("|");
}

function inputsLayoutKey(pins){
  return pins.map(pin => [pin.id,pin.pin,pin.label].join(":")).join("|");
}

function renderOutputs(outputs){
  document.getElementById("pinGrid").innerHTML = outputs.map(pinCard).join("");
}

function renderSystemPins(systemPins){
  document.getElementById("systemPinGrid").innerHTML = systemPins.map(systemPinCard).join("");
}

function renderInputs(inputs){
  document.getElementById("inputGrid").innerHTML = inputs.map(inputCard).join("");
}

function setOverviewMessage(text, kind="soft"){
  const el = document.getElementById("overviewMessage");
  el.textContent = text;
  el.className = `messageBox ${kind}`;
  clearTimeout(setOverviewMessage._t);
  setOverviewMessage._t = setTimeout(() => {
    el.className = "messageBox idle";
    el.textContent = "Ready.";
  }, 2200);
}

function updateOverview(data){
  document.getElementById("hostPill").textContent = data.name;
  document.getElementById("uptimePill").textContent = data.uptime;
  document.getElementById("displayPill").textContent = data.display;
  document.getElementById("effectName").textContent = data.modeName;
  document.getElementById("visualModeMetric").textContent = data.modeName;
  document.getElementById("audioMetric").textContent = data.sound.track + (data.sound.playing ? " ♪" : "");
  document.getElementById("ledEngineMetric").textContent = data.system.ledsEnabled ? "Enabled" : "Disabled";
  document.getElementById("buzzerEngineMetric").textContent = data.system.buzzerEnabled ? "Enabled" : "Disabled";
  document.getElementById("syncMetric").textContent = lastSyncAt ? new Date(lastSyncAt).toLocaleTimeString() : "--";

  document.getElementById("ethTag").innerHTML = data.eth.supported
    ? badge(data.eth.connected ? "on" : "off", data.eth.connected ? "Connected" : "Disconnected")
    : badge("warn","Disabled");
  document.getElementById("ethIp").textContent = `IP: ${data.eth.ip}`;

  document.getElementById("wifiTag").innerHTML = badge(data.wifi.connected ? "on" : "off", data.wifi.connected ? "Connected" : "Disconnected");
  document.getElementById("wifiSsid").textContent = `SSID: ${data.wifi.ssid}`;
  document.getElementById("wifiIp").textContent = `IP: ${data.wifi.ip}`;

  document.getElementById("ledToggleBtn").textContent = data.system.ledsEnabled ? "Disable LEDs" : "Enable LEDs";
  document.getElementById("buzzerToggleBtn").textContent = data.system.buzzerEnabled ? "Disable Buzzer" : "Enable Buzzer";
}

function updateRelayForPin(pin){
  const relay = relayForPin(pin);
  const el = document.getElementById(`relayStateForPin${pin.id}`);
  if(!el || !relay) return;
  const rs = relayBadgeState(relay);
  el.className = `badge ${rs.kind}`;
  el.textContent = rs.text;
}

function updatePinDynamic(pin){
  const s = statusBadgeForPin(pin);
  const st = document.getElementById(`pinStatus${pin.id}`);
  if(st){
    st.className = `badge ${s.kind}`;
    st.textContent = s.text;
  }

  const meta = document.getElementById(`pinMeta${pin.id}`);
  if(meta) meta.textContent = `Current: ${pin.label}`;

  const note = document.getElementById(`pinNote${pin.id}`);
  if(note) note.textContent = pin.note;

  const badges = document.getElementById(`pinBadges${pin.id}`);
  if(badges) badges.innerHTML = pinBadges(pin);

  updateRelayForPin(pin);

  const labelEl = document.getElementById(`pinLabel${pin.id}`);
  if(labelEl && document.activeElement !== labelEl){
    labelEl.value = pin.label;
  }

  const typeEl = document.getElementById(`pinType${pin.id}`);
  if(typeEl && document.activeElement !== typeEl){
    typeEl.value = pin.type;
  }

  const freqEl = document.getElementById(`pinFreq${pin.id}`);
  if(freqEl && document.activeElement !== freqEl){
    freqEl.value = pin.freq;
  }

  if(pin.type === "digital_out"){
    const valEl = document.getElementById(`pinValue${pin.id}`);
    const textEl = document.getElementById(`pinValueText${pin.id}`);
    if(valEl && document.activeElement !== valEl) valEl.checked = pin.value > 0;
    if(textEl) textEl.textContent = pin.value > 0 ? "High / On" : "Low / Off";
  }

  if(pin.type === "pwm" || pin.type === "servo"){
    const valEl = document.getElementById(`pinValue${pin.id}`);
    const textEl = document.getElementById(`pinValueText${pin.id}`);
    if(valEl && document.activeElement !== valEl) valEl.value = pin.value;
    if(textEl) textEl.textContent = pin.value;
  }
}

function updateInputDynamic(pin){
  const dEl = document.getElementById(`pinDigital${pin.id}`);
  const aEl = document.getElementById(`pinAnalog${pin.id}`);
  if(dEl) dEl.textContent = pin.digitalRead;
  if(aEl) aEl.textContent = pin.analogRead;
}

function updateDiagnostics(data, force=false){
  const panel = document.getElementById("diagPanel");
  if(!panel || !panel.open) return;
  const now = Date.now();
  if(!force && (now - lastDiagAt) < 1000) return;
  lastDiagAt = now;
  document.getElementById("rawJson").textContent = JSON.stringify(data, null, 2);
}

function applyStatus(data, forceLayout=false){
  lastData = data;

  const {outputs, systemPins, inputs} = groupsFromData(data);

  const outKey = outputsLayoutKey(outputs);
  const sysKey = outputsLayoutKey(systemPins);
  const inKey  = inputsLayoutKey(inputs);

  if(!initialized || forceLayout || outKey !== lastOutputLayoutKey){
    renderOutputs(outputs);
    lastOutputLayoutKey = outKey;
  }
  if(!initialized || forceLayout || sysKey !== lastSystemLayoutKey){
    renderSystemPins(systemPins);
    lastSystemLayoutKey = sysKey;
  }
  if(!initialized || forceLayout || inKey !== lastInputLayoutKey){
    renderInputs(inputs);
    lastInputLayoutKey = inKey;
  }

  updateOverview(data);
  outputs.forEach(updatePinDynamic);
  systemPins.forEach(updatePinDynamic);
  inputs.forEach(updateInputDynamic);
  updateDiagnostics(data, forceLayout);

  initialized = true;
}

async function updateStatus(forceLayout=false){
  if(statusBusy) return;
  statusBusy = true;
  try{
    const data = await api("/api/status");
    lastSyncAt = Date.now();
    applyStatus(data, forceLayout);
  }catch(err){
    setOverviewMessage("Status update failed: " + (err.message || err), "bad");
  }finally{
    statusBusy = false;
  }
}

function currentPin(id){
  return lastData?.pins?.find(p => p.id === id) || null;
}

async function setEffect(mode){
  try{
    await api(`/api/effect?set=${mode}`);
    await updateStatus(false);
    setOverviewMessage("Effect updated.", "good");
  }catch(err){
    setOverviewMessage("Failed to update effect: " + (err.message || err), "bad");
  }
}

async function playRick(){
  try{
    await api("/api/audio?track=rick");
    await updateStatus(false);
    setOverviewMessage("Rickroll queued.", "good");
  }catch(err){
    setOverviewMessage("Failed to play track: " + (err.message || err), "bad");
  }
}

async function stopAudio(){
  try{
    await api("/api/audio?track=stop");
    await updateStatus(false);
    setOverviewMessage("Audio stopped.", "good");
  }catch(err){
    setOverviewMessage("Failed to stop audio: " + (err.message || err), "bad");
  }
}

async function toggleLeds(){
  try{
    const on = !!lastData?.system?.ledsEnabled;
    if(lastData){
      lastData.system.ledsEnabled = !on;
      updateOverview(lastData);
    }
    await api(`/api/system?leds=${on ? "off" : "on"}`);
    await updateStatus(true);
    setOverviewMessage(on ? "LED engine disabled." : "LED engine enabled.", "good");
  }catch(err){
    await updateStatus(true);
    setOverviewMessage("Failed to toggle LED engine: " + (err.message || err), "bad");
  }
}

async function toggleBuzzer(){
  try{
    const on = !!lastData?.system?.buzzerEnabled;
    if(lastData){
      lastData.system.buzzerEnabled = !on;
      updateOverview(lastData);
    }
    await api(`/api/system?buzzer=${on ? "off" : "on"}`);
    await updateStatus(true);
    setOverviewMessage(on ? "Buzzer engine disabled." : "Buzzer engine enabled.", "good");
  }catch(err){
    await updateStatus(true);
    setOverviewMessage("Failed to toggle buzzer engine: " + (err.message || err), "bad");
  }
}

async function changePinType(id){
  const typeEl = document.getElementById(`pinType${id}`);
  const labelEl = document.getElementById(`pinLabel${id}`);
  const freqEl = document.getElementById(`pinFreq${id}`);
  if(!typeEl) return;

  const url = new URL("/api/pin/config", window.location.origin);
  url.searchParams.set("id", id);
  url.searchParams.set("type", typeEl.value);
  if(labelEl) url.searchParams.set("label", labelEl.value);
  if(freqEl) url.searchParams.set("freq", freqEl.value);

  try{
    await api(url.pathname + url.search);
    await updateStatus(true);
    setOverviewMessage(`GPIO ${currentPin(id)?.pin ?? id} mode updated.`, "good");
  }catch(err){
    await updateStatus(true);
    setOverviewMessage("Failed to change mode: " + (err.message || err), "bad");
  }
}

function queueLabelSave(id){
  clearTimeout(labelTimers[id]);
  labelTimers[id] = setTimeout(() => savePinLabel(id), LABEL_DEBOUNCE_MS);
}

async function savePinLabel(id){
  const labelEl = document.getElementById(`pinLabel${id}`);
  if(!labelEl) return;
  const url = new URL("/api/pin/config", window.location.origin);
  url.searchParams.set("id", id);
  url.searchParams.set("label", labelEl.value);

  try{
    await api(url.pathname + url.search);
    if(lastData){
      const pin = currentPin(id);
      if(pin) pin.label = labelEl.value;
    }
    setOverviewMessage(`GPIO ${currentPin(id)?.pin ?? id} label saved.`, "soft");
  }catch(err){
    setOverviewMessage("Failed to save label: " + (err.message || err), "bad");
  }
}

function queueFreqSave(id){
  clearTimeout(freqTimers[id]);
  freqTimers[id] = setTimeout(() => savePinFreq(id), 150);
}

async function savePinFreq(id){
  const pin = currentPin(id);
  const freqEl = document.getElementById(`pinFreq${id}`);
  if(!pin || !freqEl) return;

  const url = new URL("/api/pin/config", window.location.origin);
  url.searchParams.set("id", id);
  url.searchParams.set("type", pin.type);
  url.searchParams.set("freq", freqEl.value);

  const labelEl = document.getElementById(`pinLabel${id}`);
  if(labelEl) url.searchParams.set("label", labelEl.value);

  try{
    await api(url.pathname + url.search);
    await updateStatus(false);
  }catch(err){
    await updateStatus(true);
    setOverviewMessage("Failed to update frequency: " + (err.message || err), "bad");
  }
}

async function pushDigitalValue(id, checked){
  const textEl = document.getElementById(`pinValueText${id}`);
  if(textEl) textEl.textContent = checked ? "High / On" : "Low / Off";

  try{
    await api(`/api/pin/value?id=${id}&value=${checked ? 1 : 0}`);
    await updateStatus(false);
  }catch(err){
    await updateStatus(true);
    setOverviewMessage("Failed to update output: " + (err.message || err), "bad");
  }
}

function queueRangeValue(id, value){
  clearTimeout(valueTimers[id]);
  valueTimers[id] = setTimeout(() => pushRangeValue(id, value), 40);
}

async function pushRangeValue(id, value){
  try{
    await api(`/api/pin/value?id=${id}&value=${encodeURIComponent(value)}`);
  }catch(err){
    setOverviewMessage("Failed to update value: " + (err.message || err), "bad");
  }
}

document.getElementById("diagPanel").addEventListener("toggle", () => {
  if(lastData && document.getElementById("diagPanel").open){
    updateDiagnostics(lastData, true);
  }
});

updateStatus(true);
setInterval(() => updateStatus(false), POLL_MS);
</script>
</body>
</html>
)HTML";

// ==================== HELPERS ====================
static uint8_t clamp8(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

String boolJson(bool b) {
  return b ? "true" : "false";
}

String jsonEscape(const String &in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"':  out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:   out += c; break;
    }
  }
  return out;
}

String ipToString(IPAddress ip) {
  if ((uint32_t)ip == 0) return String("0.0.0.0");
  return ip.toString();
}

String uptimeString(uint32_t ms) {
  uint32_t sec = ms / 1000;
  uint32_t days = sec / 86400; sec %= 86400;
  uint32_t hrs  = sec / 3600;  sec %= 3600;
  uint32_t mins = sec / 60;    sec %= 60;

  String s;
  if (days) s += String(days) + "d ";
  if (days || hrs) s += String(hrs) + "h ";
  if (days || hrs || mins) s += String(mins) + "m ";
  s += String(sec) + "s";
  return s;
}

bool wifiConnected() {
#if ENABLE_WIFI
  return WiFi.status() == WL_CONNECTED;
#else
  return false;
#endif
}

void startFallbackAP() {
#if ENABLE_WIFI
  if (apFallbackActive) return;

  WiFi.mode(WIFI_AP_STA);

  bool ok = false;
  if (FALLBACK_AP_PASS && strlen(FALLBACK_AP_PASS) > 0) {
    ok = WiFi.softAP(FALLBACK_AP_SSID, FALLBACK_AP_PASS);
  } else {
    ok = WiFi.softAP(FALLBACK_AP_SSID);
  }

  if (!ok) {
    Serial.println("Failed to start fallback hotspot");
    return;
  }

  apFallbackActive = true;

  Serial.println();
  Serial.println("Configured Wi-Fi not found after 10 seconds.");
  Serial.print("Fallback hotspot started: ");
  Serial.println(FALLBACK_AP_SSID);
  Serial.print("Fallback AP IP: ");
  Serial.println(WiFi.softAPIP());
#endif
}

void stopFallbackAP() {
#if ENABLE_WIFI
  if (!apFallbackActive) return;

  WiFi.softAPdisconnect(false);
  apFallbackActive = false;

  Serial.println("Primary Wi-Fi connected. Fallback hotspot stopped.");
#endif
}

const char* effectName(EffectMode m) {
  switch (m) {
    case MODE_RAINBOW_PHYSICAL: return "Rainbow Physical";
    case MODE_PURPLE_FIRE:      return "Purple Fire";
    default:                    return "Unknown";
  }
}

const char* deviceTypeKey(DeviceType t) {
  switch (t) {
    case DEV_DISABLED:    return "disabled";
    case DEV_DIGITAL_OUT: return "digital_out";
    case DEV_PWM:         return "pwm";
    case DEV_SERVO:       return "servo";
    case DEV_INPUT:       return "input";
    default:              return "disabled";
  }
}

DeviceType parseDeviceType(const String &s) {
  if (s == "digital_out") return DEV_DIGITAL_OUT;
  if (s == "pwm")         return DEV_PWM;
  if (s == "servo")       return DEV_SERVO;
  if (s == "input")       return DEV_INPUT;
  return DEV_DISABLED;
}

void setLabel(char* dst, size_t len, const String &src) {
  String trimmed = src;
  trimmed.trim();
  if (trimmed.length() == 0) trimmed = "Unnamed";
  snprintf(dst, len, "%s", trimmed.substring(0, len - 1).c_str());
}

int findDeviceIndexByPin(uint8_t pin) {
  for (uint8_t i = 0; i < PIN_DEVICE_COUNT; i++) {
    if (devices[i].pin == pin) return i;
  }
  return -1;
}

bool pinLockedBySystem(const PinDevice &d) {
  if (d.ledShared && ledEngineEnabled) return true;
  if (d.buzzerShared && buzzerEnabled) return true;
  return false;
}

String pinLockReason(const PinDevice &d) {
  if (d.ledShared && ledEngineEnabled) return "LED engine active";
  if (d.buzzerShared && buzzerEnabled) return "Buzzer active";
  return "";
}

bool pinAllowsType(const PinDevice &d, DeviceType t) {
  if (d.inputOnly) {
    return (t == DEV_DISABLED || t == DEV_INPUT);
  }
  if (t == DEV_INPUT || t == DEV_DISABLED) return true;
  if (pinLockedBySystem(d)) return false;
  return true;
}

void releasePinDevice(PinDevice &d) {
  if (d.pwmAttached) {
    ledcWrite(d.pin, 0);
    ledcDetach(d.pin);
    d.pwmAttached = false;
  }
}

uint32_t servoDutyFromAngle(int angle) {
  angle = constrain(angle, 0, 180);
  const uint32_t maxDuty = 65535;
  const uint32_t us = map(angle, 0, 180, 500, 2500);
  return (us * maxDuty) / 20000UL;
}

bool applyPinValue(PinDevice &d) {
  if (pinLockedBySystem(d) && (d.type == DEV_DIGITAL_OUT || d.type == DEV_PWM || d.type == DEV_SERVO)) {
    return false;
  }

  switch (d.type) {
    case DEV_DISABLED:
      return true;

    case DEV_INPUT:
      return true;

    case DEV_DIGITAL_OUT:
      pinMode(d.pin, OUTPUT);
      digitalWrite(d.pin, d.value > 0 ? HIGH : LOW);
      return true;

    case DEV_PWM:
      d.value = constrain(d.value, 0, 1023);
      d.freq  = constrain(d.freq, 1, 40000);
      if (!d.pwmAttached) {
        if (!ledcAttach(d.pin, d.freq, 10)) return false;
        d.pwmAttached = true;
      } else {
        if (ledcChangeFrequency(d.pin, d.freq, 10) == 0) return false;
      }
      return ledcWrite(d.pin, d.value);

    case DEV_SERVO:
      d.value = constrain(d.value, 0, 180);
      if (!d.pwmAttached) {
        if (!ledcAttach(d.pin, 50, 16)) return false;
        d.pwmAttached = true;
      } else {
        if (ledcChangeFrequency(d.pin, 50, 16) == 0) return false;
      }
      return ledcWrite(d.pin, servoDutyFromAngle(d.value));
  }
  return false;
}

bool applyPinConfig(PinDevice &d) {
  if (!pinAllowsType(d, d.type)) {
    return false;
  }

  releasePinDevice(d);

  switch (d.type) {
    case DEV_DISABLED:
      pinMode(d.pin, INPUT);
      return true;

    case DEV_INPUT:
      pinMode(d.pin, INPUT);
      return true;

    case DEV_DIGITAL_OUT:
      pinMode(d.pin, OUTPUT);
      digitalWrite(d.pin, d.value > 0 ? HIGH : LOW);
      return true;

    case DEV_PWM:
      d.freq = constrain(d.freq, 1, 40000);
      if (!ledcAttach(d.pin, d.freq, 10)) return false;
      d.pwmAttached = true;
      return ledcWrite(d.pin, constrain(d.value, 0, 1023));

    case DEV_SERVO:
      if (!ledcAttach(d.pin, 50, 16)) return false;
      d.pwmAttached = true;
      return ledcWrite(d.pin, servoDutyFromAngle(d.value));
  }
  return false;
}

int analogSampleForDevice(const PinDevice &d) {
  if (d.type != DEV_INPUT) return -1;
  if (d.pin == 35 || d.pin == 36 || d.pin == 37) {
    return analogRead(d.pin);
  }
  return -1;
}

String deviceNote(const PinDevice &d) {
  String note;
  if (d.relayIndex >= 0) {
    note += "Shared with relay ";
    note += String(d.relayIndex + 1);
    note += ". ";
  }
  if (d.ledShared) {
    note += "GPIO17 is LED data and is only available when LEDs are disabled. ";
  }
  if (d.buzzerShared) {
    note += "GPIO16 is used by the buzzer unless the buzzer engine is disabled. ";
  }
  if (d.inputOnly) {
    note += "Input-only monitoring pad. ";
  }
  if (d.type == DEV_SERVO) {
    note += "Servo mode outputs 50 Hz pulses. ";
  } else if (d.type == DEV_PWM) {
    note += "PWM mode uses hardware LEDC. ";
  }
  return note;
}

RelayMirrorState relayMirrorStateForIndex(uint8_t relayIdx) {
  if (relayIdx >= 8) return RELAY_UNMANAGED;
  uint8_t pin = relayPins[relayIdx];
  int devIdx = findDeviceIndexByPin(pin);
  if (devIdx < 0) return RELAY_UNMANAGED;

  const PinDevice &d = devices[devIdx];

  if (d.buzzerShared && buzzerEnabled) return RELAY_RESERVED;

  switch (d.type) {
    case DEV_DIGITAL_OUT: {
      bool levelHigh = d.value > 0;
      bool relayOn = RELAY_ACTIVE_LOW ? !levelHigh : levelHigh;
      return relayOn ? RELAY_ON : RELAY_OFF;
    }
    case DEV_PWM:
    case DEV_SERVO:
      return RELAY_DRIVEN;
    case DEV_DISABLED:
    case DEV_INPUT:
    default:
      return RELAY_UNMANAGED;
  }
}

const char* relayStateKey(RelayMirrorState s) {
  switch (s) {
    case RELAY_RESERVED:  return "reserved";
    case RELAY_UNMANAGED: return "unmanaged";
    case RELAY_OFF:       return "off";
    case RELAY_ON:        return "on";
    case RELAY_DRIVEN:    return "driven";
    default:              return "unmanaged";
  }
}

String relayStateNote(RelayMirrorState s) {
  switch (s) {
    case RELAY_RESERVED:  return "Reserved by buzzer output.";
    case RELAY_UNMANAGED: return "Shared line is not actively driven.";
    case RELAY_OFF:       return "Shared line is driving relay OFF.";
    case RELAY_ON:        return "Shared line is driving relay ON.";
    case RELAY_DRIVEN:    return "Shared line is being modulated by PWM/servo output.";
    default:              return "";
  }
}

// ==================== SOUND ENGINE ====================
void ensureBuzzerReady() {
  if (!buzzerEnabled) return;
  ledcAttach(PIN_BUZZER, 2000, 10);
  ledcWriteTone(PIN_BUZZER, 0);
}

void stopTune() {
  if (buzzerEnabled) {
    ledcWriteTone(PIN_BUZZER, 0);
  }
  sound = TunePlayer();
}

void startTune(const int* melody, size_t length, int tempo, const char* name) {
  if (!buzzerEnabled) return;
  ensureBuzzerReady();
  ledcWriteTone(PIN_BUZZER, 0);
  sound.data = melody;
  sound.length = length;
  sound.tempo = tempo;
  sound.index = 0;
  sound.active = true;
  sound.toneOn = false;
  sound.noteOffAtMs = 0;
  sound.nextStepAtMs = 0;
  sound.name = name;
}

void updateSound() {
  if (!buzzerEnabled) return;
  if (!sound.active) return;

  uint32_t now = millis();

  if (sound.toneOn && now >= sound.noteOffAtMs) {
    ledcWriteTone(PIN_BUZZER, 0);
    sound.toneOn = false;
  }

  if (now < sound.nextStepAtMs) return;

  if (sound.index >= sound.length) {
    stopTune();
    return;
  }

  int note = sound.data[sound.index++];
  int divider = sound.data[sound.index++];

  int wholeNote = (60000 * 4) / sound.tempo;
  int noteDuration = 0;

  if (divider > 0) {
    noteDuration = wholeNote / divider;
  } else {
    noteDuration = wholeNote / abs(divider);
    noteDuration = (int)(noteDuration * 1.5f);
  }

  if (note == REST) {
    ledcWriteTone(PIN_BUZZER, 0);
    sound.toneOn = false;
  } else {
    ledcWriteTone(PIN_BUZZER, note);
    sound.toneOn = true;
    sound.noteOffAtMs = now + (noteDuration * 9) / 10;
  }

  sound.nextStepAtMs = now + noteDuration;
}

// ==================== EFFECTS ====================
void setPhysicalXY(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
  int8_t idx = physToIndex[y][x];
  if (idx >= 0) {
    strip.setPixelColor(idx, strip.Color(r, g, b));
  }
}

void setPhysicalCell(uint8_t cell, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t y = cell / 2;
  uint8_t x = cell % 2;
  setPhysicalXY(x, y, r, g, b);
}

void wheel(uint8_t p, uint8_t &r, uint8_t &g, uint8_t &b) {
  p = 255 - p;
  if (p < 85) {
    r = 255 - p * 3;
    g = 0;
    b = p * 3;
  } else if (p < 170) {
    p -= 85;
    r = 0;
    g = p * 3;
    b = 255 - p * 3;
  } else {
    p -= 170;
    r = p * 3;
    g = 255 - p * 3;
    b = 0;
  }
}

void scaleColorByPercent(uint8_t &r, uint8_t &g, uint8_t &b, uint8_t pct) {
  r = (uint16_t)r * pct / 100;
  g = (uint16_t)g * pct / 100;
  b = (uint16_t)b * pct / 100;
}

void purpleFireColor(uint8_t h, uint8_t y, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (h < 80) {
    r = h * 10 / 80;
    g = 0;
    b = h * 70 / 80;
  } else if (h < 160) {
    uint8_t t = h - 80;
    r = 10 + t * 80 / 80;
    g = 0;
    b = 70 + t * 90 / 80;
  } else if (h < 220) {
    uint8_t t = h - 160;
    r = 90 + t * 115 / 60;
    g = 0;
    b = 160 + t * 65 / 60;
  } else {
    uint8_t t = h - 220;
    r = 205 + t * 50 / 35;
    g = t * 40 / 35;
    b = 225 + t * 25 / 35;
  }

  r = clamp8(r);
  g = clamp8(g);
  b = clamp8(b);

  if (y <= 1 && h > 245) {
    uint8_t tint = (h - 245) * 2;
    r = clamp8(r + tint);
    g = clamp8(g + tint);
    b = clamp8(b + tint);
  }

  scaleColorByPercent(r, g, b, rowLevelPct[y]);
}

void fxPurpleFireBetter() {
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 2; x++) {
      uint8_t cooldown = random(4, 14) + (uint8_t)((4 - y) * 12);
      heat[y][x] = (heat[y][x] > cooldown) ? (heat[y][x] - cooldown) : 0;
    }
  }

  for (int x = 0; x < 2; x++) {
    heat[0][x] = (heat[1][x] + heat[2][x] + heat[2][x]) / 4;
    heat[1][x] = (heat[2][x] + heat[3][x] + heat[3][x]) / 4;
    heat[2][x] = (heat[3][x] + heat[4][x] + heat[4][x]) / 4;
    heat[3][x] = (heat[4][x] + heat[4][x]) / 3;
  }

  for (int y = 0; y < 5; y++) {
    uint8_t avg = (heat[y][0] + heat[y][1]) / 2;
    heat[y][0] = (heat[y][0] * 3 + avg) / 4;
    heat[y][1] = (heat[y][1] * 3 + avg) / 4;
  }

  if (random(100) < 55) {
    int x = random(2);
    heat[4][x] = clamp8(heat[4][x] + random(120, 220));
  }
  if (random(100) < 18) {
    int x = random(2);
    heat[3][x] = clamp8(heat[3][x] + random(40, 100));
  }

  for (uint8_t i = 0; i < LED_COUNT; i++) {
    uint8_t y = ledY[i];
    uint8_t x = ledX[i];
    uint8_t h = heat[y][x];
    uint8_t r, g, b;
    purpleFireColor(h, y, r, g, b);
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
}

void fxRainbowPhysical() {
  for (uint8_t y = 0; y < 5; y++) {
    for (uint8_t x = 0; x < 2; x++) {
      uint8_t r, g, b;
      uint8_t hue = (uint8_t)(tick * 3 + y * 38 + x * 20);
      wheel(hue, r, g, b);
      setPhysicalXY(x, y, r, g, b);
    }
  }
}

void drawRelayStatusOverlay() {
  for (uint8_t i = 0; i < 8; i++) {
    RelayMirrorState s = relayMirrorStateForIndex(i);
    switch (s) {
      case RELAY_ON:        setPhysicalCell(i, 0, 255, 0); break;
      case RELAY_OFF:       setPhysicalCell(i, 255, 0, 0); break;
      case RELAY_DRIVEN:    setPhysicalCell(i, 255, 160, 0); break;
      case RELAY_RESERVED:  setPhysicalCell(i, 255, 200, 0); break;
      case RELAY_UNMANAGED:
      default:              setPhysicalCell(i, 18, 40, 120); break;
    }
  }

#if ENABLE_ETHERNET
  setPhysicalCell(8, ethConnected ? 0 : 255, ethConnected ? 255 : 0, 0);
#else
  setPhysicalCell(8, 255, 180, 0);
#endif

  setPhysicalCell(9, wifiConnected() ? 0 : 255, wifiConnected() ? 255 : 0, 0);
}

// ==================== SYSTEM TOGGLES ====================
void clearAndShowStrip() {
  strip.clear();
  strip.show();
}

void setLedEngineEnabled(bool on) {
  if (on == ledEngineEnabled) return;
  ledEngineEnabled = on;

  int idx = findDeviceIndexByPin(PIN_LED_DATA);
  if (on && idx >= 0) {
    devices[idx].type = DEV_DISABLED;
    devices[idx].value = 0;
    applyPinConfig(devices[idx]);
    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    clearAndShowStrip();
  } else if (!on) {
    clearAndShowStrip();
  }
}

void setBuzzerEnabled(bool on) {
  if (on == buzzerEnabled) return;

  if (!on) {
    stopTune();
    ledcWriteTone(PIN_BUZZER, 0);
    ledcDetach(PIN_BUZZER);
    buzzerEnabled = false;
    return;
  }

  int idx = findDeviceIndexByPin(PIN_BUZZER);
  if (idx >= 0) {
    devices[idx].type = DEV_DISABLED;
    devices[idx].value = 0;
    applyPinConfig(devices[idx]);
  }

  buzzerEnabled = true;
  ensureBuzzerReady();
}

// ==================== BUTTON / CLICK HANDLER ====================
void handleButton() {
  int reading = digitalRead(PIN_BUTTON);

  if (reading != btnLastReading) {
    btnLastChangeMs = millis();
    btnLastReading = reading;
  }

  if ((millis() - btnLastChangeMs) > BUTTON_DEBOUNCE_MS) {
    if (reading != btnStableState) {
      btnStableState = reading;

      if (btnStableState == HIGH) {
        pendingClicks++;
        if (pendingClicks > 12) pendingClicks = 12;
        lastClickReleaseMs = millis();
      }
    }
  }
}

void processPendingClicks() {
  if (!pendingClicks) return;
  if (millis() - lastClickReleaseMs < MULTICLICK_TIMEOUT_MS) return;

  if (pendingClicks >= RICKROLL_CLICK_COUNT) {
    startTune(rickTune, ARRAY_LEN(rickTune), 114, "rickroll");
  } else {
    for (uint8_t i = 0; i < pendingClicks; i++) {
      effectMode = (EffectMode)((effectMode + 1) % MODE_COUNT);
    }
    startTune(clickTune, ARRAY_LEN(clickTune), 240, "click");
  }

  pendingClicks = 0;
}

// ==================== NETWORK ====================
#if ENABLE_ETHERNET
void onNetworkEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname(HOSTNAME);
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      ethConnected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_LOST_IP:
    case ARDUINO_EVENT_ETH_STOP:
      ethConnected = false;
      break;
    default:
      break;
  }
}
#endif

void maintainWiFi() {
#if ENABLE_WIFI
  if (strlen(WIFI_SSID) == 0) return;

  if (wifiConnected()) {
    stopFallbackAP();
    return;
  }

  // After 10 seconds without connecting, bring up fallback AP.
  if (!apFallbackActive && (millis() - wifiConnectStartMs >= WIFI_FAILOVER_MS)) {
    startFallbackAP();
  }

  // Keep retrying the configured STA network in the background.
  if (millis() - lastWiFiAttemptMs >= WIFI_RETRY_MS) {
    lastWiFiAttemptMs = millis();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
#endif
}

// ==================== JSON ====================
String buildStatusJson() {
  String s;
  s.reserve(9000);

  s += "{";
  s += "\"name\":\"" + jsonEscape(String(PRODUCT_NAME)) + "\",";
  s += "\"hostname\":\"" + jsonEscape(String(HOSTNAME)) + "\",";
  s += "\"uptime\":\"" + uptimeString(millis()) + "\",";
  s += "\"mode\":" + String((int)effectMode) + ",";
  s += "\"modeName\":\"" + jsonEscape(String(effectName(effectMode))) + "\",";
  s += "\"display\":\"" + jsonEscape(String((btnStableState == LOW && ledEngineEnabled) ? "Relay Status Overlay" : (ledEngineEnabled ? effectName(effectMode) : "LED Engine Disabled"))) + "\",";

  s += "\"system\":{";
  s += "\"ledsEnabled\":" + boolJson(ledEngineEnabled) + ",";
  s += "\"buzzerEnabled\":" + boolJson(buzzerEnabled);
  s += "},";

  s += "\"sound\":{";
  s += "\"playing\":" + boolJson(sound.active) + ",";
  s += "\"track\":\"" + jsonEscape(String(sound.name)) + "\"";
  s += "},";

  s += "\"eth\":{";
#if ENABLE_ETHERNET
  s += "\"supported\":true,";
  s += "\"connected\":" + boolJson(ethConnected) + ",";
  s += "\"ip\":\"" + ipToString(ETH.localIP()) + "\"";
#else
  s += "\"supported\":false,";
  s += "\"connected\":false,";
  s += "\"ip\":\"0.0.0.0\"";
#endif
  s += "},";

  s += "\"wifi\":{";
  s += "\"connected\":" + boolJson(wifiConnected() || apFallbackActive) + ",";
  s += "\"stationConnected\":" + boolJson(wifiConnected()) + ",";
  s += "\"apFallback\":" + boolJson(apFallbackActive) + ",";

#if ENABLE_WIFI
  String wifiName = apFallbackActive
    ? String(FALLBACK_AP_SSID)
    : (wifiConnected() ? WiFi.SSID() : String(WIFI_SSID));

  IPAddress wifiIp = apFallbackActive ? WiFi.softAPIP() : WiFi.localIP();

  s += "\"ssid\":\"" + jsonEscape(wifiName) + "\",";
  s += "\"ip\":\"" + ipToString(wifiIp) + "\",";
#else
  s += "\"ssid\":\"\",\"ip\":\"0.0.0.0\",";
#endif

  s += "\"note\":\"" + jsonEscape(String(
    apFallbackActive ? "Fallback hotspot active" :
    (wifiConnected() ? "Connected to configured Wi-Fi" : "Trying configured Wi-Fi")
  )) + "\"";
  s += "},";

  s += "\"relays\":[";
  for (uint8_t i = 0; i < 8; i++) {
    if (i) s += ",";
    RelayMirrorState rs = relayMirrorStateForIndex(i);
    s += "{";
    s += "\"id\":" + String(i + 1) + ",";
    s += "\"label\":\"R" + String(i + 1) + "\",";
    s += "\"pin\":" + String(relayPins[i]) + ",";
    s += "\"state\":\"" + String(relayStateKey(rs)) + "\",";
    s += "\"note\":\"" + jsonEscape(relayStateNote(rs)) + "\"";
    s += "}";
  }
  s += "],";

  s += "\"pins\":[";
  for (uint8_t i = 0; i < PIN_DEVICE_COUNT; i++) {
    const PinDevice &d = devices[i];
    if (i) s += ",";
    s += "{";
    s += "\"id\":" + String(i) + ",";
    s += "\"pin\":" + String(d.pin) + ",";
    s += "\"label\":\"" + jsonEscape(String(d.label)) + "\",";
    s += "\"type\":\"" + String(deviceTypeKey(d.type)) + "\",";
    s += "\"value\":" + String(d.value) + ",";
    s += "\"freq\":" + String(d.freq) + ",";
    s += "\"inputOnly\":" + boolJson(d.inputOnly) + ",";
    s += "\"inputCapable\":true,";
    s += "\"relayShared\":" + boolJson(d.relayIndex >= 0) + ",";
    s += "\"ledShared\":" + boolJson(d.ledShared) + ",";
    s += "\"buzzerShared\":" + boolJson(d.buzzerShared) + ",";
    s += "\"locked\":" + boolJson(pinLockedBySystem(d)) + ",";
    s += "\"lockReason\":\"" + jsonEscape(pinLockReason(d)) + "\",";
    s += "\"digitalRead\":" + String(digitalRead(d.pin)) + ",";
    s += "\"analogRead\":" + String(analogSampleForDevice(d)) + ",";
    s += "\"note\":\"" + jsonEscape(deviceNote(d)) + "\"";
    s += "}";
  }
  s += "]";

  s += "}";
  return s;
}

// ==================== HTTP ====================
void sendJsonError(int code, const String &msg) {
  server.send(code, "application/json", "{\"ok\":false,\"error\":\"" + jsonEscape(msg) + "\"}");
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleApiStatus() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send(200, "application/json", buildStatusJson());
}

void handleApiEffect() {
  if (!server.hasArg("set")) {
    sendJsonError(400, "missing set");
    return;
  }
  int m = server.arg("set").toInt();
  if (m < 0 || m >= MODE_COUNT) {
    sendJsonError(400, "bad mode");
    return;
  }
  effectMode = (EffectMode)m;
  startTune(clickTune, ARRAY_LEN(clickTune), 240, "click");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiAudio() {
  String track = server.arg("track");
  track.toLowerCase();

  if (track == "rick") {
    startTune(rickTune, ARRAY_LEN(rickTune), 114, "rickroll");
  } else if (track == "boot") {
    startTune(bootTune, ARRAY_LEN(bootTune), 220, "boot");
  } else if (track == "click") {
    startTune(clickTune, ARRAY_LEN(clickTune), 240, "click");
  } else if (track == "stop") {
    stopTune();
  } else {
    sendJsonError(400, "bad track");
    return;
  }

  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiSystem() {
  if (server.hasArg("leds")) {
    String v = server.arg("leds");
    v.toLowerCase();
    setLedEngineEnabled(v == "on" || v == "1" || v == "true");
  }

  if (server.hasArg("buzzer")) {
    String v = server.arg("buzzer");
    v.toLowerCase();
    setBuzzerEnabled(v == "on" || v == "1" || v == "true");
  }

  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiPinConfig() {
  if (!server.hasArg("id")) {
    sendJsonError(400, "missing id");
    return;
  }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= PIN_DEVICE_COUNT) {
    sendJsonError(400, "bad id");
    return;
  }

  PinDevice &d = devices[id];

  if (server.hasArg("label")) {
    setLabel(d.label, sizeof(d.label), server.arg("label"));
  }

  if (server.hasArg("type")) {
    DeviceType newType = parseDeviceType(server.arg("type"));
    if (!pinAllowsType(d, newType)) {
      sendJsonError(409, "pin currently reserved by active subsystem or unsupported mode");
      return;
    }
    d.type = newType;
  }

  if (server.hasArg("freq")) {
    d.freq = constrain(server.arg("freq").toInt(), 1, 40000);
  }

  if (!applyPinConfig(d)) {
    sendJsonError(500, "failed to apply pin config");
    return;
  }

  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiPinValue() {
  if (!server.hasArg("id") || !server.hasArg("value")) {
    sendJsonError(400, "missing id/value");
    return;
  }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= PIN_DEVICE_COUNT) {
    sendJsonError(400, "bad id");
    return;
  }

  PinDevice &d = devices[id];
  d.value = server.arg("value").toInt();

  switch (d.type) {
    case DEV_DIGITAL_OUT: d.value = d.value ? 1 : 0; break;
    case DEV_PWM:         d.value = constrain(d.value, 0, 1023); break;
    case DEV_SERVO:       d.value = constrain(d.value, 0, 180); break;
    default: break;
  }

  if (!applyPinValue(d)) {
    sendJsonError(500, "failed to apply value");
    return;
  }

  server.send(200, "application/json", "{\"ok\":true}");
}

void setupServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/effect", HTTP_GET, handleApiEffect);
  server.on("/api/audio", HTTP_GET, handleApiAudio);
  server.on("/api/system", HTTP_GET, handleApiSystem);
  server.on("/api/pin/config", HTTP_GET, handleApiPinConfig);
  server.on("/api/pin/value", HTTP_GET, handleApiPinValue);
  server.on("/favicon.ico", HTTP_GET, []() { server.send(204, "text/plain", ""); });
  server.onNotFound([]() {
    sendJsonError(404, "not found");
  });
  server.begin();
}

// ==================== INIT ====================
void initDevices() {
  for (uint8_t i = 0; i < PIN_DEVICE_COUNT; i++) {
    const PinMeta &m = PIN_META[i];
    PinDevice &d = devices[i];

    d.pin = m.pin;
    snprintf(d.label, sizeof(d.label), "%s", m.defaultLabel);
    d.inputOnly = m.inputOnly;
    d.relayIndex = m.relayIndex;
    d.ledShared = m.ledShared;
    d.buzzerShared = m.buzzerShared;
    d.pwmAttached = false;
    d.freq = 1000;
    d.value = 0;

    if (m.inputOnly) {
      d.type = DEV_INPUT;
    } else if (m.relayIndex >= 0 && !(m.buzzerShared && buzzerEnabled)) {
      d.type = DEV_DIGITAL_OUT;
      d.value = DEFAULT_SHARED_LINES_ON ? 1 : 0;
    } else {
      d.type = DEV_DISABLED;
    }
  }

  for (uint8_t i = 0; i < PIN_DEVICE_COUNT; i++) {
    applyPinConfig(devices[i]);
  }
}

void initLeds() {
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.clear();
  strip.show();
}

void initWiFi() {
#if ENABLE_WIFI
  if (strlen(WIFI_SSID) == 0) return;

  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  lastWiFiAttemptMs = millis();
  wifiConnectStartMs = millis();

  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(WIFI_SSID);
#endif
}

void initEthernet() {
#if ENABLE_ETHERNET
  Network.onEvent(onNetworkEvent);
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_POWER, ETH_CLK_MODE);
#endif
}

// ==================== ARDUINO ====================
void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(PIN_BUTTON, INPUT);
  randomSeed(micros());

  initWiFi();
  initEthernet();

  if (buzzerEnabled) {
    ensureBuzzerReady();
  }

  initDevices();
  initLeds();
  setupServer();

  if (buzzerEnabled) {
    startTune(bootTune, ARRAY_LEN(bootTune), 220, "boot");
  }

  Serial.println();
  Serial.println("Nif-T booted");
  Serial.println("Admin panel ready.");
  Serial.println("5 quick button clicks = Rickroll");
  Serial.println("Hold button = temporary relay/network LED status");
}

void loop() {
  server.handleClient();
  maintainWiFi();
  handleButton();
  processPendingClicks();
  updateSound();

  if (!ledEngineEnabled) return;

  if (millis() - lastFrameMs < FRAME_MS) return;
  lastFrameMs = millis();
  tick++;

  strip.clear();

  if (btnStableState == LOW) {
    drawRelayStatusOverlay();
  } else {
    switch (effectMode) {
      case MODE_RAINBOW_PHYSICAL:
        fxRainbowPhysical();
        break;
      case MODE_PURPLE_FIRE:
        fxPurpleFireBetter();
        break;
      default:
        break;
    }
  }

  strip.show();
}