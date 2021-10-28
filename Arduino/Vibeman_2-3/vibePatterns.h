enum CURVE {
  LINEAR = 0,
  QUADRATIC,
  EASEOUT,
  EASEIN,
  COSINE,
  NA
};

struct vibePattern {
  uint8_t pwr;
  float deltaT;
  CURVE curve;
};

vibePattern vWave[] = {
  {64, 60, EASEOUT},
  {255, 30, EASEIN},
  {128, 30, EASEOUT},
  {255, 60, EASEIN},
};

vibePattern vStep[] = {
  {0, 60, NA},
  {64, 60, NA},
  {128, 60, NA},
  {194, 60, NA},
  {255, 60, NA},
  {255, 60, LINEAR}
};
