<p align="center">
  <img src="./Renders/banner.png" alt="Banner"/>
</p>

# Nif-T - An all in one smart home automation board

### A No-Nonsense Ethernet + PoE Smart-Home Controller with Opinions

---

## Renders  
*Visuals because everyone loves eye candy.*

![Side PCB Render](/Renders/sidePCBRender.png)
![Top PCB Render](/Renders/topPCBRender.png)
![Bottom PCB Render](/Renders/bottomPCBRender.png)
![Routing Glamour Shot](/Renders/PCBRender.png)
![Schematic Picture](/Renders/schematicRoot.png)
![Schematic Picture](/Renders/schematicPowerAndIO.png)

---

## BOM  
All the silicon, magnetics, and clicky-relay goodness live **[here](/BOM.csv)**.

**Main characters:**
- **MCU:** ESP32-WROOM-32E
- **Ethernet PHY:** LAN8720A (RMII, external 50 MHz oscillator)
- **Power over Ethernet:** IEEE 802.3af PoE (on-board PD + isolation)
- **Relays:** 8× PCB-mount power relays
- **RGB LEDs:** 10× SK6812 Mini
- **Buzzer:** On-board piezo
- **User Input:** Programmable user button
- **Connectivity:** Ethernet (primary), USB-UART for flashing/debug
- **Power:** PoE-powered with on-board regulation for ESP32 + peripherals
- **Misc:** Boot circuitry, reset logic, pull-ups, decoupling, and other necessary magic

(See `BOM.md` for full part values, footprints, and sourcing.)

---

## What is this?

**Nif-T** is a **wired smart-home automation controller** built around an **ESP32-WROOM-32E**, designed to be the thing that actually flips real switches when you ask your house to do something.

It’s meant for:
- **Lights**
- **Outlets**
- **Relays**
- **Buzzers**
- **Addressable LEDs**
- And other “turn this on when I say so” tasks

Unlike a lot of Wi-Fi-only smart devices, **Nif-T speaks Ethernet** — and it can be powered over the same cable using **PoE**. One cable in, automation out.

At a high level, Nif-T:
- Exposes **physical relays and LEDs**
- Talks to **Home Assistant**
- Which then talks to **Google Home**
- So you can say things like:

> *“Hey Google, turn on the big light.”*

…and something actually happens.

---

## Features

### Relay Control
- **8× independent relay outputs**
- Designed for:
  - Lighting circuits
  - Power control
  - Low-voltage automation tasks
- Each relay is individually addressable via firmware
- Can be controlled via:
  - MQTT
  - Home Assistant
  - Google Home (through HA)

*(Please don’t hot-switch mains unless you know what you’re doing.)*

---

### Ethernet + PoE
Because reliability matters — and wall warts are annoying.

- **LAN8720A Ethernet PHY**
  - RMII interface
  - Dedicated 50 MHz oscillator on the PHY
- **IEEE 802.3af Power over Ethernet**
  - Single-cable power + data
  - On-board PD controller and isolation
  - Clean DC rails generated locally
- No Wi-Fi congestion
- No random disconnects
- No external power brick required

Plug in Ethernet. That’s it.

---

### RGB Status LEDs
Because silent boards are boring.

- **10× SK6812 Mini (GRBW) LEDs**
- Can indicate:
  - Network status
  - MQTT / Home Assistant connection
  - Relay activity
  - Boot / error states
  - Or just run rainbow animations for morale
- Fully firmware-controlled

---

### Buttons & Feedback
- **User button**
  - Can be mapped to:
    - Toggle a relay
    - Enter config mode
    - Trigger a scene
- **On-board buzzer**
  - Boot beeps
  - Error alerts
  - Confirmation chirps
  - Mildly annoying sounds (optional but encouraged)

---

## Connectivity & IO

- **ESP32-WROOM-32E**
  - Dual-core MCU
  - Plenty of GPIO for:
    - Relays
    - LEDs
    - Buttons
    - Buzzers
- **USB-UART**
  - Firmware flashing
  - Serial debugging
- **Screw terminals**
  - For relays and power connections that shouldn’t wiggle loose

---

## Pin Highlights (High-Level)

| Function        | Notes                                   |
|-----------------|------------------------------------------|
| Relays          | 8× GPIO-controlled outputs               |
| LED Data        | SK6812 GRBW strip (10 LEDs)              |
| Ethernet RMII   | LAN8720A w/ external 50 MHz oscillator   |
| PoE Input       | IEEE 802.3af compliant                   |
| User Button     | Input-only GPIO (external pull-ups)      |
| Buzzer          | GPIO-driven piezo output                 |

*(See schematic for full pin mapping and signal names.)*

---

## How It Works

At a high level:

1. **You plug in Ethernet** (which also provides power via PoE)
2. The **PoE PD negotiates power** and generates isolated DC rails
3. The **ESP32 boots** and brings up Ethernet
4. It connects to:
   - An **MQTT broker** (usually Home Assistant)
5. Home Assistant:
   - Discovers Nif-T automatically
   - Exposes relays and LEDs as entities
6. Google Home talks to Home Assistant
7. You talk to Google
8. Nif-T clicks relays and blinks LEDs

---

## Example Use Cases

- “Hey Google, turn on the **workbench lights**”
- “Turn off **everything downstairs**”
- LED status strip that shows:
  - Green = online
  - Red = offline
  - Blue = updating
- Physical button toggles a relay *and* triggers a Home Assistant scene
- Buzzer chirps when automations fire (or silently if you prefer)

---

## Why?

Because:
- Wi-Fi smart plugs are flaky
- Ethernet is king
- PoE makes installs clean
- And sometimes you just want a **solid, hackable, relay board** that plays nicely with modern home automation

Nif-T exists to be:
- Boring in the best way
- Reliable
- Predictable
- And extremely easy to integrate into a real automation stack

---

## Contribute

Ideas, improvements, and constructive roasting welcome.

- Open an **issue**
- Submit a **PR**
- Suggest features
- Argue about relay counts

---

## Disclaimer

This project is open-source, built for fun, and tested only in theory.

If you:
- Wire mains incorrectly
- Ignore clearance rules
- Overload a relay
- Or let the magic smoke out

That’s on you.

Be careful, be smart, and enjoy having a house that actually listens.