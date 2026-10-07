#pragma once
#include <Arduino.h>

namespace PixMobNOVA {

static constexpr uint8_t MAGIC = 0x80;
static constexpr uint32_t SYMBOL_US = 694;
static constexpr uint32_t FRAME_GAP_US = 4500;
static constexpr uint8_t MAX_BYTES = 9;

static const uint8_t ENCODING_MAP[64] = {
  0x21,0x32,0x54,0x65,0xA9,0x9A,0x6D,0x29,
  0x56,0x92,0xA1,0xB4,0xB2,0x84,0x66,0x2A,
  0x4C,0x6A,0xA6,0x95,0x62,0x51,0x42,0x24,
  0x35,0x46,0x8A,0xAC,0x8C,0x6C,0x2C,0x4A,
  0x59,0x86,0xA4,0xA2,0x91,0x64,0x55,0x44,
  0x22,0x31,0xB1,0x52,0x85,0x96,0xA5,0x69,
  0x5A,0x2D,0x4D,0x89,0x45,0x34,0x61,0x25,
  0x36,0xAD,0x94,0xAA,0x8D,0x49,0x99,0x26
};

enum CommandType : uint8_t {
  CMD_UNKNOWN=0, CMD_SINGLE_COLOR=1, CMD_SINGLE_COLOR_EXT=2,
  CMD_TWO_COLORS=3, CMD_SET_CONFIG=4, CMD_SET_COLOR=5,
  CMD_SET_GROUP_SEL=6, CMD_SET_GROUP_ID=7, CMD_SET_REPEAT_DELAY=8,
  CMD_SET_REPEAT_COUNT=9, CMD_SET_GST=10, CMD_RESET=11
};

struct RGB { uint8_t r=0,g=0,b=0; };

struct Packet {
  uint8_t encoded[MAX_BYTES]{};
  uint8_t decoded[MAX_BYTES]{};
  uint8_t length=0, flags=0, action=0;
  CommandType command=CMD_UNKNOWN;
  RGB color1, color2;
  uint8_t chanceCode=0, attackCode=0, sustainCode=0, releaseCode=0;
  uint8_t groupId=0;
  bool repeat=false, gsten=false, onStart=false;
};

inline int8_t decodeSymbol(uint8_t v) {
  for (uint8_t i=0;i<64;i++) if (ENCODING_MAP[i]==v) return i;
  return -1;
}

inline uint8_t expand6(uint8_t x) {
  x &= 0x3F;
  return uint8_t((x<<2)|(x>>4));
}

inline uint16_t timeCodeMs(uint8_t c) {
  static const uint16_t t[8]={0,32,96,192,480,960,2400,3840};
  return t[c&7];
}

inline uint16_t gstCodeMs(uint8_t c) {
  static const uint16_t t[8]={64,112,160,208,480,960,2400,3840};
  return t[c&7];
}

inline uint8_t chancePercent(uint8_t c) {
  static const uint8_t p[8]={100,88,67,50,32,16,10,4};
  return p[c&7];
}

/* Mirrors the published project's handling of omitted leading/trailing zeros. */
inline bool bitsToBytes(const uint8_t *bits,size_t n,uint8_t *out,uint8_t &nbytes) {
  memset(out,0,MAX_BYTES);
  size_t lead=0;
  while(lead<n && bits[lead]==0) ++lead;
  if(lead>=n) return false;

  size_t highest=0;
  for(size_t i=lead;i<n;i++) {
    long bitPos=long(i)+7L-long(lead);
    if(bitPos<0) continue;
    size_t bi=size_t(bitPos)/8;
    if(bi>=MAX_BYTES) return false;
    if(bits[i]) out[bi] |= uint8_t(1U<<(bitPos%8));
    if(bits[i] && bi>highest) highest=bi;
  }
  nbytes=uint8_t(highest+1);
  return nbytes==6 || nbytes==9;
}

inline bool decode(const uint8_t *bits,size_t n,Packet &p) {
  memset(&p,0,sizeof(p));
  uint8_t nbytes=0;
  if(!bitsToBytes(bits,n,p.encoded,nbytes)) return false;
  if(p.encoded[0]!=MAGIC) return false;

  p.length=nbytes;
  p.decoded[0]=MAGIC;

  uint16_t sum=0;
  for(uint8_t i=2;i<nbytes;i++) {
    sum += p.encoded[i];
    int8_t d=decodeSymbol(p.encoded[i]);
    if(d<0) return false;
    p.decoded[i]=uint8_t(d);
  }

  if(p.encoded[1] != ENCODING_MAP[(sum>>2)&0x3F]) return false;

  p.onStart=(p.decoded[2]&1)!=0;
  p.flags=(p.decoded[2]>>1)&7;
  p.gsten=(p.decoded[2]>>4)&1;

  p.color1.g=expand6(p.decoded[3]);
  p.color1.r=expand6(p.decoded[4]);
  p.color1.b=expand6(p.decoded[5]);

  if(nbytes==9) {
    p.color2.g=expand6(p.decoded[6]);
    p.color2.r=expand6(p.decoded[7]);
    p.color2.b=expand6(p.decoded[8]);
    p.action=p.decoded[7]&0x1F;
  }

  if(nbytes==6 && p.flags==0) {
    p.command=CMD_SINGLE_COLOR;
    return true;
  }

  if(nbytes==9 && p.flags==0) {
    p.command=CMD_SINGLE_COLOR_EXT;
    p.chanceCode=p.decoded[6]&7;
    p.attackCode=(p.decoded[6]>>3)&7;
    p.sustainCode=p.decoded[7]&7;
    p.releaseCode=(p.decoded[7]>>3)&7;
    p.groupId=p.decoded[8]&0x1F;
    p.repeat=(p.decoded[8]>>5)&1;
    return true;
  }

  if(nbytes==9 && p.flags==2) {
    p.command=CMD_TWO_COLORS;
    return true;
  }

  if(nbytes==6 && p.flags==1) {
    p.command=CMD_SET_CONFIG;
    return true;
  }

  if(nbytes==9 && p.flags==7) {
    switch(p.action) {
      case 0:  p.command=CMD_SET_COLOR; break;
      case 1:  p.command=CMD_SET_GROUP_SEL; break;
      case 2:  p.command=CMD_SET_GROUP_ID; break;
      case 7:  p.command=CMD_SET_REPEAT_DELAY; break;
      case 8:  p.command=CMD_SET_REPEAT_COUNT; break;
      case 9:  p.command=CMD_SET_GST; break;
      case 15: p.command=CMD_RESET; break;
      default: p.command=CMD_UNKNOWN; break;
    }
    return true;
  }

  return true;
}

} // namespace PixMobNOVA
