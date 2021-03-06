#ifndef _SYRINGELIST_H_
#define _SYRINGELIST_H_

struct syringeSpec {
  uint16_t vol;  // mL
  float stroke;  // mm
  uint16_t lowSpeedSGT;
  uint16_t lowSpeedCurrent;  // mA
  uint16_t highSpeedSGT;
  uint16_t highSpeedCurrent;  // mA
};

static const syringeSpec syringeInfo[] = {
  {.vol=20 , .stroke=63.66 , .lowSpeedSGT=10, .lowSpeedCurrent=300, .highSpeedSGT=15, .highSpeedCurrent=400},
  {.vol=50 , .stroke=109.97, .lowSpeedSGT=10, .lowSpeedCurrent=350, .highSpeedSGT=20, .highSpeedCurrent=450},
  {.vol=100, .stroke=141.17, .lowSpeedSGT=10, .lowSpeedCurrent=400, .highSpeedSGT=20, .highSpeedCurrent=500},
  {.vol=250, .stroke=151.61, .lowSpeedSGT=10, .lowSpeedCurrent=450, .highSpeedSGT=20, .highSpeedCurrent=550},
  {.vol=500, .stroke=207.80, .lowSpeedSGT=10, .lowSpeedCurrent=500, .highSpeedSGT=20, .highSpeedCurrent=600}
};

enum syringeType {st20=0, st50, st100, st250, st500};

#endif // _SYRINGELIST_H_
