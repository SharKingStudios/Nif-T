/*
  Nif-T Firmware (ESP32-WROOM-32E + LAN8720A)
  - Ethernet (LAN8720) using GPIO0 50MHz clock out
  - MQTT control for 8 relays
  - SK6812 MINI GRBW strip (10 LEDs) on GPIO17
  - Home Assistant MQTT Discovery

  Topics:
    nif-t/<chipid>/relay/<n>/set   payload: ON/OFF
    nif-t/<chipid>/relay/<n>/state payload: ON/OFF
    nif-t/<chipid>/led/set         payload: JSON {"state":"ON","brightness":0-255,"r":0-255,"g":0-255,"b":0-255,"w":0-255}
    nif-t/<chipid>/led/state       same JSON

  If you use Home Assistant, discovery will auto-create entities.
*/

#include <WiFi.h>
#include <ETH.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>

// -------------------- USER CONFIG --------------------
static const char* MQTT_HOST = "192.168.1.10"; // <-- set to your broker / Home Assistant IP
static const uint16_t MQTT_PORT = 1883;

// If your relays are active-low, set to true:
static const bool RELAY_ACTIVE_LOW = false;

// Relay 8 shares GPIO16 with buzzer.
// Set this true if GPIO16 is BUZZER_DATA, not relay8:
static const bool USE_GPIO16_AS_BUZZER = false;

// -------------------- PIN MAP (from your message) --------------------
// Ethernet PHY (LAN8720A)
static const int PIN_ETH_POWER_OR_RESET = 5;  // IO5 8720RST (Arduino ETH uses this as "power pin" toggle)
static const int PIN_ETH_MDC = 23;            // IO23 MDC
static const int PIN_ETH_MDIO = 18;           // IO18 MDIO
static const int PIN_ETH_ADDR = 0;            // common LAN8720 phy addr is 0 (sometimes 1)

// Ethernet MAC pins you listed (handled internally by ESP-IDF):
// GPIO19 TXD0
// GPIO22 TXD1
// GPIO21 TXEN
// GPIO25 RXD0 / MODE0
// GPIO26 RXD1 / MODE1
// GPIO27 CRS_DV / MODE2
// Clock: GPIO0 ETH_CLK + BOOT (we use ETH_CLOCK_GPIO0_IN)

// Relays
static const int PIN_RLY1 = 32;
static const int PIN_RLY2 = 33;
static const int PIN_RLY3 = 13;
static const int PIN_RLY4 = 14;
static const int PIN_RLY5 = 2;
static const int PIN_RLY6 = 4;
static const int PIN_RLY7 = 15;
static const int PIN_RLY8 = 16; // unless USE_GPIO16_AS_BUZZER

// LEDs
static const int PIN_LED_DATA = 17;
static const int LED_COUNT = 10;

// User button (GPIO34 is input-only; typically needs external pullup/pulldown)
static const int PIN_USER_BUTTON = 34;

// Buzzer
static const int PIN_BUZZER = 16;

// -------------------- GLOBALS --------------------
static bool ethConnected = false;

WiFiClient netClient;          // Works with ETH as well on ESP32 Arduino core
PubSubClient mqtt(netClient);

Adafruit_NeoPixel strip(LED_COUNT, PIN_LED_DATA, NEO_GRBW + NEO_KHZ800);

String baseTopic;              // nif-t/<chipid>
String deviceName;             // nif-t-<chipid>

bool relayState[8] = {false,false,false,false,false,false,false,false};

// LED “logical” state
bool ledOn = true;
uint8_t ledBri = 255;
uint8_t ledR=0, ledG=0, ledB=0, ledW=0;

// -------------------- HELPERS --------------------
static uint32_t chipId32() {
  uint64_t mac = ESP.getEfuseMac();
  return (uint32_t)(mac & 0xFFFFFFFF);
}

static const int RELAY_PINS[8] = {
  PIN_RLY1, PIN_RLY2, PIN_RLY3, PIN_RLY4, PIN_RLY5, PIN_RLY6, PIN_RLY7, PIN_RLY8
};

static void writeRelayHW(int idx, bool on) {
  if (idx < 0 || idx > 7) return;
  int pin = RELAY_PINS[idx];
  if (idx == 7 && USE_GPIO16_AS_BUZZER) return; // Relay8 disabled if buzzer uses GPIO16

  bool level = on;
  if (RELAY_ACTIVE_LOW) level = !on;
  digitalWrite(pin, level ? HIGH : LOW);
}

static void publishRelayState(int idx) {
  String t = baseTopic + "/relay/" + String(idx+1) + "/state";
  mqtt.publish(t.c_str(), relayState[idx] ? "ON" : "OFF", true);
}

static void setRelay(int idx, bool on, bool publish=true) {
  relayState[idx] = on;
  writeRelayHW(idx, on);
  if (publish && mqtt.connected()) publishRelayState(idx);
}

static void applyLed() {
  uint8_t r = ledOn ? ledR : 0;
  uint8_t g = ledOn ? ledG : 0;
  uint8_t b = ledOn ? ledB : 0;
  uint8_t w = ledOn ? ledW : 0;

  // Apply brightness by scaling channels (simple + predictable)
  auto scale = [&](uint8_t v)->uint8_t {
    return (uint16_t(v) * uint16_t(ledBri)) / 255;
  };

  uint32_t c = strip.Color(scale(r), scale(g), scale(b), scale(w));
  for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, c);
  strip.show();
}

static void publishLedState() {
  String t = baseTopic + "/led/state";
  // Minimal JSON (no extra deps)
  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"state\":\"%s\",\"brightness\":%u,\"r\":%u,\"g\":%u,\"b\":%u,\"w\":%u}",
           ledOn ? "ON" : "OFF",
           (unsigned)ledBri,
           (unsigned)ledR,(unsigned)ledG,(unsigned)ledB,(unsigned)ledW);
  mqtt.publish(t.c_str(), buf, true);
}

static bool parseJsonByte(const String& s, const char* key, uint8_t& out) {
  String k = String("\"") + key + "\":";
  int i = s.indexOf(k);
  if (i < 0) return false;
  i += k.length();
  while (i < (int)s.length() && (s[i] == ' ')) i++;
  int j = i;
  while (j < (int)s.length() && isDigit((unsigned char)s[j])) j++;
  if (j == i) return false;
  int v = s.substring(i, j).toInt();
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  out = (uint8_t)v;
  return true;
}

static bool parseJsonState(const String& s, bool& outOn) {
  int i = s.indexOf("\"state\"");
  if (i < 0) return false;
  int q1 = s.indexOf('"', i + 7);
  if (q1 < 0) return false;
  int q2 = s.indexOf('"', q1 + 1);
  if (q2 < 0) return false;
  String val = s.substring(q1 + 1, q2);
  val.toUpperCase();
  outOn = (val == "ON");
  return true;
}

// -------------------- HOME ASSISTANT DISCOVERY --------------------
static void haPublishRelayDiscovery(int idx) {
  // discovery topic: homeassistant/switch/<unique>/relayN/config
  String uid = deviceName + "_relay_" + String(idx+1);
  String topic = "homeassistant/switch/" + uid + "/config";

  String name = "Nif-T Relay " + String(idx+1);
  String cmd_t = baseTopic + "/relay/" + String(idx+1) + "/set";
  String st_t  = baseTopic + "/relay/" + String(idx+1) + "/state";
  String av_t  = baseTopic + "/availability";

  // device block ties everything into one device in HA
  String payload =
    "{"
      "\"name\":\"" + name + "\","
      "\"unique_id\":\"" + uid + "\","
      "\"command_topic\":\"" + cmd_t + "\","
      "\"state_topic\":\"" + st_t + "\","
      "\"payload_on\":\"ON\","
      "\"payload_off\":\"OFF\","
      "\"availability_topic\":\"" + av_t + "\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"retain\":true,"
      "\"device\":{"
        "\"identifiers\":[\"" + deviceName + "\"],"
        "\"name\":\"" + deviceName + "\","
        "\"model\":\"Nif-T\","
        "\"manufacturer\":\"Custom\""
      "}"
    "}";

  mqtt.publish(topic.c_str(), payload.c_str(), true);
}

static void haPublishLedDiscovery() {
  // Make this a "light" entity
  String uid = deviceName + "_led";
  String topic = "homeassistant/light/" + uid + "/config";

  String name = "Nif-T LEDs";
  String cmd_t = baseTopic + "/led/set";
  String st_t  = baseTopic + "/led/state";
  String av_t  = baseTopic + "/availability";

  String payload =
    "{"
      "\"name\":\"" + name + "\","
      "\"unique_id\":\"" + uid + "\","
      "\"command_topic\":\"" + cmd_t + "\","
      "\"state_topic\":\"" + st_t + "\","
      "\"availability_topic\":\"" + av_t + "\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"retain\":true,"
      "\"supported_color_modes\":[\"rgbw\"],"
      "\"brightness\":true,"
      "\"schema\":\"json\","
      "\"device\":{"
        "\"identifiers\":[\"" + deviceName + "\"],"
        "\"name\":\"" + deviceName + "\","
        "\"model\":\"Nif-T\","
        "\"manufacturer\":\"Custom\""
      "}"
    "}";

  mqtt.publish(topic.c_str(), payload.c_str(), true);
}

static void haPublishAllDiscovery() {
  for (int i = 0; i < 8; i++) {
    if (i == 7 && USE_GPIO16_AS_BUZZER) continue;
    haPublishRelayDiscovery(i);
  }
  haPublishLedDiscovery();
}

// -------------------- MQTT CALLBACK --------------------
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t(topic);
  String msg;
  msg.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  // Relay set: nif-t/<chipid>/relay/N/set
  for (int i = 0; i < 8; i++) {
    String rt = baseTopic + "/relay/" + String(i+1) + "/set";
    if (t == rt) {
      String up = msg; up.toUpperCase();
      setRelay(i, (up == "ON" || up == "1" || up == "TRUE"));
      return;
    }
  }

  // LED set: nif-t/<chipid>/led/set   (JSON)
  if (t == (baseTopic + "/led/set")) {
    bool newOn = ledOn;
    uint8_t v;

    parseJsonState(msg, newOn);

    if (parseJsonByte(msg, "brightness", v)) ledBri = v;
    if (parseJsonByte(msg, "r", v)) ledR = v;
    if (parseJsonByte(msg, "g", v)) ledG = v;
    if (parseJsonByte(msg, "b", v)) ledB = v;
    if (parseJsonByte(msg, "w", v)) ledW = v;

    ledOn = newOn;
    applyLed();
    if (mqtt.connected()) publishLedState();
    return;
  }
}

// -------------------- ETHERNET EVENTS --------------------
static void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      ETH.setHostname(deviceName.c_str());
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      ethConnected = true;
      Serial.print("ETH IP: ");
      Serial.println(ETH.localIP());
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
    case SYSTEM_EVENT_ETH_STOP:
      ethConnected = false;
      break;
    default:
      break;
  }
}

// -------------------- MQTT CONNECT --------------------
static void mqttConnect() {
  if (!ethConnected) return;
  if (mqtt.connected()) return;

  Serial.println("Connecting MQTT...");

  // LWT availability
  String avail = baseTopic + "/availability";

  // Client ID
  String clientId = deviceName;

  if (mqtt.connect(clientId.c_str(), avail.c_str(), 0, true, "offline")) {
    mqtt.publish(avail.c_str(), "online", true);

    // Subscribe
    for (int i = 0; i < 8; i++) {
      if (i == 7 && USE_GPIO16_AS_BUZZER) continue;
      String rt = baseTopic + "/relay/" + String(i+1) + "/set";
      mqtt.subscribe(rt.c_str());
    }
    mqtt.subscribe((baseTopic + "/led/set").c_str());

    // Discovery + initial states
    haPublishAllDiscovery();

    for (int i = 0; i < 8; i++) {
      if (i == 7 && USE_GPIO16_AS_BUZZER) continue;
      publishRelayState(i);
    }
    publishLedState();

    Serial.println("MQTT connected.");
  } else {
    Serial.print("MQTT failed, rc=");
    Serial.println(mqtt.state());
  }
}

// -------------------- SETUP/LOOP --------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  uint32_t id = chipId32();
  deviceName = "nif-t-" + String(id, HEX);
  baseTopic  = "nif-t/" + String(id, HEX);

  Serial.println();
  Serial.println("Booting " + deviceName);

  // GPIO init
  for (int i = 0; i < 8; i++) {
    if (i == 7 && USE_GPIO16_AS_BUZZER) continue;
    pinMode(RELAY_PINS[i], OUTPUT);
  }

  // default all relays OFF
  for (int i = 0; i < 8; i++) {
    if (i == 7 && USE_GPIO16_AS_BUZZER) continue;
    setRelay(i, false, false);
  }

  // Button input (GPIO34 has no internal pullup)
  pinMode(PIN_USER_BUTTON, INPUT);

  // LEDs
  strip.begin();
  strip.show();
  applyLed();

  // Ethernet
  WiFi.onEvent(WiFiEvent);

  // IMPORTANT: clock mode uses GPIO0 OUT 50MHz as you specified
  ETH.begin(
    PIN_ETH_ADDR,
    PIN_ETH_POWER_OR_RESET,
    PIN_ETH_MDC,
    PIN_ETH_MDIO,
    ETH_PHY_LAN8720,
    ETH_CLOCK_GPIO0_IN
  );

  // MQTT
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
}

void loop() {
  mqttConnect();
  mqtt.loop();

  // Example: button toggles Relay1 (optional convenience)
  static int lastBtn = 0;
  int btn = digitalRead(PIN_USER_BUTTON);
  if (btn != lastBtn) {
    lastBtn = btn;
    // if your button is active-low, you may want: if (btn == LOW)
    if (btn == HIGH) {
      setRelay(0, !relayState[0]);
      // simple beep if buzzer mode
      if (USE_GPIO16_AS_BUZZER) {
        pinMode(PIN_BUZZER, OUTPUT);
        digitalWrite(PIN_BUZZER, HIGH);
        delay(40);
        digitalWrite(PIN_BUZZER, LOW);
      }
    }
  }

  delay(5);
}
