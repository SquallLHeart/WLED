#pragma once
/*
 * PixMob NOVA Event Receiver for WLED / ESP32-C3
 *
 * Default hardware:
 *   38 kHz demodulator OUT -> GPIO4
 *
 * The ISR captures only edge timing into a fixed ring buffer.
 * It never drives LEDs, prints serial data, allocates memory, or delays.
 *
 * This is intentionally a documented-protocol compatibility receiver.
 * It targets NOVA FRENCH VANILLA v3.1 / firmware 0x05.
 */

#include "wled.h"
#include "PixMobNOVAProtocol.h"

class PixMobNovaEventReceiver : public Usermod {
private:
  static constexpr uint8_t DEFAULT_IR_PIN=4;
  static constexpr uint32_t DEFAULT_TIMEOUT_MS=10000;
  static constexpr size_t EDGE_RING=256;
  static constexpr size_t MAX_BITS=256;

  struct Edge { uint32_t dt; uint8_t level; };
  Edge ring[EDGE_RING];

  volatile uint16_t head=0, tail=0;
  volatile uint32_t lastEdgeUs=0;
  volatile bool overflow=false;

  uint8_t irPin=DEFAULT_IR_PIN;
  uint32_t timeoutMs=DEFAULT_TIMEOUT_MS;
  uint32_t lastPacketMs=0;
  bool active=false, initialized=false, enabled=true;

  enum Mode:uint8_t { AUTO=0,NOVA=1,WLED=2 };
  Mode mode=AUTO;

  uint8_t groupIds[8]{};
  uint8_t groupSel=0;
  uint8_t repeatCount=0;
  uint16_t repeatDelayMs=0;
  uint16_t gstMs=64;

  uint8_t bits[MAX_BITS]{};
  size_t bitCount=0;

  struct Effect {
    PixMobNOVA::RGB a{},b{};
    uint32_t startMs=0;
    uint16_t attack=0,sustain=0,release=0,delay=0;
    uint16_t repeats=0;
    bool two=false,running=false;
  } effect;

  static PixMobNovaEventReceiver *self;

  static void IRAM_ATTR isr() {
    if(!self) return;
    uint32_t now=micros();
    uint32_t dt=now-self->lastEdgeUs;
    self->lastEdgeUs=now;

    uint16_t next=uint16_t((self->head+1)%EDGE_RING);
    if(next==self->tail) { self->overflow=true; return; }

    self->ring[self->head].dt=dt;
    self->ring[self->head].level=(uint8_t)gpio_get_level((gpio_num_t)self->irPin);
    self->head=next;
  }

  bool pop(Edge &e) {
    noInterrupts();
    if(tail==head) { interrupts(); return false; }
    e=ring[tail];
    tail=uint16_t((tail+1)%EDGE_RING);
    interrupts();
    return true;
  }

  void appendRun(uint32_t us,uint8_t level) {
    if(us<250) return;
    float cells=float(us)/float(PixMobNOVA::SYMBOL_US);
    int n=int(lroundf(cells));
    if(n<1 || n>16) return;
    if(fabsf(cells-float(n))>0.45f) return;

    // Standard demodulator: LOW=mark=logical 1, HIGH=logical 0.
    uint8_t bit=level ? 0 : 1;
    while(n-- && bitCount<MAX_BITS) bits[bitCount++]=bit;
  }

  void resetFrame() { bitCount=0; }

  void decodeFrame() {
    if(bitCount<48) return;
    PixMobNOVA::Packet p;
    if(!PixMobNOVA::decode(bits,bitCount,p)) return;

    lastPacketMs=millis();
    if(mode==AUTO) active=true;

    Serial.printf("[NOVA] len=%u flags=%u cmd=%u action=%u onstart=%u gsten=%u\n",
                  p.length,p.flags,(unsigned)p.command,p.action,
                  p.onStart,p.gsten);
    Serial.printf("[NOVA] RGB1=%u,%u,%u\n",p.color1.r,p.color1.g,p.color1.b);

    execute(p);
  }

  void consume() {
    Edge e;
    while(pop(e)) {
      if(e.dt>=PixMobNOVA::FRAME_GAP_US) {
        decodeFrame();
        resetFrame();
        continue;
      }
      appendRun(e.dt,e.level);
    }

    // Flush the last frame even if no subsequent packet has arrived.
    noInterrupts();
    uint32_t since=micros()-lastEdgeUs;
    interrupts();
    if(bitCount && since>=PixMobNOVA::FRAME_GAP_US) {
      decodeFrame();
      resetFrame();
    }
  }

  bool groupAllowed(const PixMobNOVA::Packet &p) const {
    if(p.command!=PixMobNOVA::CMD_SINGLE_COLOR_EXT) return true;
    if(p.groupId==0) return true;
    return p.groupId==groupIds[groupSel];
  }

  uint32_t color(PixMobNOVA::RGB c) { return RGBW32(c.r,c.g,c.b,0); }

  void setMain(PixMobNOVA::RGB c) {
    Segment &seg=strip.getSegment(strip.getMainSegmentId());
    seg.setColor(0,color(c));
    seg.setOption(SEG_OPTION_ON,true);
    strip.trigger();
  }

  void startSingle(const PixMobNOVA::Packet &p) {
    if(!groupAllowed(p)) return;

    if(p.command==PixMobNOVA::CMD_SINGLE_COLOR_EXT) {
      uint8_t chance=PixMobNOVA::chancePercent(p.chanceCode);
      if(chance<100 && random(100)>=chance) return;
    }

    effect.a=p.color1;
    effect.b=p.color1;
    effect.attack=(p.command==PixMobNOVA::CMD_SINGLE_COLOR_EXT)
      ? PixMobNOVA::timeCodeMs(p.attackCode) : 0;

    uint16_t s=(p.command==PixMobNOVA::CMD_SINGLE_COLOR_EXT)
      ? PixMobNOVA::timeCodeMs(p.sustainCode) : 120;

    effect.sustain=(p.gsten && p.sustainCode==7) ? gstMs : s;
    effect.release=(p.command==PixMobNOVA::CMD_SINGLE_COLOR_EXT)
      ? PixMobNOVA::timeCodeMs(p.releaseCode) : 32;
    effect.delay=repeatDelayMs;
    effect.repeats=repeatCount;
    effect.two=false;
    effect.startMs=millis();
    effect.running=true;
  }

  void startTwo(const PixMobNOVA::Packet &p) {
    effect.a=p.color1; effect.b=p.color2;
    effect.attack=0; effect.sustain=250; effect.release=0;
    effect.delay=repeatDelayMs; effect.repeats=repeatCount;
    effect.two=true; effect.startMs=millis(); effect.running=true;
  }

  void execute(const PixMobNOVA::Packet &p) {
    switch(p.command) {
      case PixMobNOVA::CMD_SINGLE_COLOR:
      case PixMobNOVA::CMD_SINGLE_COLOR_EXT: startSingle(p); break;

      case PixMobNOVA::CMD_TWO_COLORS: startTwo(p); break;

      case PixMobNOVA::CMD_SET_GROUP_SEL:
        groupSel=p.decoded[8]&7; break;

      case PixMobNOVA::CMD_SET_GROUP_ID: {
        uint8_t sel=p.decoded[5]&7;
        groupIds[sel]=p.decoded[6]&0x1F;
        break;
      }

      case PixMobNOVA::CMD_SET_REPEAT_DELAY:
        repeatDelayMs=PixMobNOVA::timeCodeMs(p.decoded[6]&7);
        break;

      case PixMobNOVA::CMD_SET_REPEAT_COUNT:
        repeatCount=uint8_t(p.decoded[5]|((p.decoded[6]&3)<<6));
        break;

      case PixMobNOVA::CMD_SET_GST:
        gstMs=PixMobNOVA::gstCodeMs((p.decoded[4]>>2)&7);
        break;

      case PixMobNOVA::CMD_RESET:
        effect.running=false; setMain({0,0,0}); break;

      default:
        // Valid but not confidently renderable: do nothing.
        break;
    }
  }

  void render() {
    if(!effect.running) return;

    uint32_t now=millis();
    uint32_t t=now-effect.startMs;

    if(t<effect.delay) return;
    t-=effect.delay;

    if(effect.two) {
      if(t<effect.sustain) setMain(effect.a);
      else if(t<uint32_t(effect.sustain)*2) setMain(effect.b);
      else finish();
      return;
    }

    uint32_t total=uint32_t(effect.attack)+effect.sustain+effect.release;

    if(t<effect.attack) {
      float f=effect.attack?float(t)/effect.attack:1.0f;
      setMain({
        uint8_t(effect.a.r*f),uint8_t(effect.a.g*f),uint8_t(effect.a.b*f)
      });
    } else if(t<uint32_t(effect.attack)+effect.sustain) {
      setMain(effect.a);
    } else if(t<total) {
      uint32_t rt=t-effect.attack-effect.sustain;
      float f=effect.release?1.0f-float(rt)/effect.release:0.0f;
      setMain({
        uint8_t(effect.a.r*max(0.0f,f)),
        uint8_t(effect.a.g*max(0.0f,f)),
        uint8_t(effect.a.b*max(0.0f,f))
      });
    } else finish();
  }

  void finish() {
    if(effect.repeats) {
      --effect.repeats;
      effect.startMs=millis();
      return;
    }
    effect.running=false;
    setMain({0,0,0});
  }

  const char *modeName() const {
    return mode==NOVA?"nova":mode==WLED?"wled":"auto";
  }

public:
  void setup() override {
    self=this;
    pinMode(irPin,INPUT);
    lastEdgeUs=micros();
    attachInterrupt(digitalPinToInterrupt(irPin),isr,CHANGE);
    initialized=true;
  }

  void loop() override {
    if(!enabled || !initialized) return;

    consume();

    if(mode==WLED) {
      active=false;
      effect.running=false;
      return;
    }

    render();

    if(mode==AUTO && active && millis()-lastPacketMs>timeoutMs) {
      active=false;
      effect.running=false;
    }

    if(overflow) {
      noInterrupts();
      overflow=false;
      tail=head;
      interrupts();
      resetFrame();
    }
  }

  void addToJsonInfo(JsonObject &root) override {
    JsonObject u=root["u"];
    if(u.isNull()) u=root.createNestedObject("u");
    JsonObject n=u.createNestedObject("PixMob NOVA");
    n["target_fw"]="0x05";
    n["ir_pin"]=irPin;
    n["mode"]=modeName();
    n["active"]=active;
    n["last_packet_ms"]=lastPacketMs;
  }

  void addToJsonState(JsonObject &root) override {
    if(!initialized) return;
    JsonObject n=root["PixMob NOVA"];
    if(n.isNull()) n=root.createNestedObject("PixMob NOVA");
    n["mode"]=modeName();
    n["active"]=active;
    n["timeout_ms"]=timeoutMs;
    n["group_sel"]=groupSel;
    n["repeat_count"]=repeatCount;
    n["repeat_delay_ms"]=repeatDelayMs;
    n["gst_ms"]=gstMs;
  }

  void readFromJsonState(JsonObject &root) override {
    JsonObject n=root["PixMob NOVA"];
    if(n.isNull()) return;
    String m=n["mode"]|modeName();
    mode=(m=="nova")?NOVA:(m=="wled")?WLED:AUTO;
    timeoutMs=n["timeout_ms"]|timeoutMs;
    timeoutMs=constrain(timeoutMs,250UL,30000UL);
  }

  void addToConfig(JsonObject &root) override {
    JsonObject n=root.createNestedObject("PixMob NOVA");
    n["enabled"]=enabled;
    n["ir_pin"]=irPin;
    n["timeout_ms"]=timeoutMs;
    n["mode"]=modeName();
    n["group_sel"]=groupSel;
    n["repeat_count"]=repeatCount;
    n["repeat_delay_ms"]=repeatDelayMs;
    n["gst_ms"]=gstMs;
    JsonArray ids=n.createNestedArray("group_ids");
    for(uint8_t i=0;i<8;i++) ids.add(groupIds[i]);
  }

  bool readFromConfig(JsonObject &root) override {
    JsonObject n=root["PixMob NOVA"];
    if(n.isNull()) return false;

    enabled=n["enabled"]|enabled;
    irPin=n["ir_pin"]|irPin;
    timeoutMs=n["timeout_ms"]|timeoutMs;

    String m=n["mode"]|"auto";
    mode=(m=="nova")?NOVA:(m=="wled")?WLED:AUTO;

    groupSel=n["group_sel"]|groupSel;
    repeatCount=n["repeat_count"]|repeatCount;
    repeatDelayMs=n["repeat_delay_ms"]|repeatDelayMs;
    gstMs=n["gst_ms"]|gstMs;

    JsonArray ids=n["group_ids"].as<JsonArray>();
    if(!ids.isNull())
      for(uint8_t i=0;i<8 && i<ids.size();i++) groupIds[i]=ids[i];

    return true;
  }

  void appendConfigData() override {}
};

PixMobNovaEventReceiver *PixMobNovaEventReceiver::self=nullptr;
static PixMobNovaEventReceiver pixMobNovaEventReceiver;
REGISTER_USERMOD(pixMobNovaEventReceiver);
