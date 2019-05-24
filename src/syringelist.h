#ifndef _SYRINGELIST_H_
#define _SYRINGELIST_H_

struct syringeSpec {
  uint16_t vol;  // mL
  float stroke;  // mm
};

static const syringeSpec syringeInfo[] = {
  {.vol=20 , .stroke=63.66},
  {.vol=50 , .stroke=109.97},
  {.vol=100, .stroke=141.17},
  {.vol=250, .stroke=151.61},
  {.vol=500, .stroke=207.80}
};

enum syringeType {st20=0, st50, st100, st250, st500};

#endif // _SYRINGELIST_H_
