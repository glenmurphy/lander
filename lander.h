// http://www.crait.net/tochars/
const unsigned char spr_smoke[] PROGMEM {
  0x3c, 0x42, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3c,
};

const unsigned char spr_smoke_fill[] PROGMEM {
  0x00, 0x3c, 0x7e, 0x7e, 0x7e, 0x7e, 0x3c, 0x00,
};

// http://www.andrewlowndes.co.uk/blog/graphics/arduboy-image-converter
const unsigned char spr_sm_smoke[] PROGMEM {
  0x60, 0x90, 0x90, 0x60,
  0x00, 0x00, 0x00, 0x00
};

const unsigned char spr_sm_smoke_fill[] PROGMEM {
  0x00, 0x60, 0x60, 0x00,
  0x00, 0x00, 0x00, 0x00,
};

// http://fuopy.github.io/arduboy-image-converter/
// Intro screen; 46 x 16
const unsigned char spr_title[] PROGMEM {
  0xff, 0xff, 0xc0, 0xc0, 0xc0, 0xc0, 0x00, 0x00,
  0xff, 0xff, 0x03, 0x33, 0xff, 0xff, 0x00, 0x00,
  0xff, 0xff, 0x03, 0x03, 0xff, 0xff, 0x00, 0x00,
  0xff, 0xff, 0xc3, 0xc3, 0x3c, 0x3c, 0x00, 0x00,
  0xff, 0xff, 0xcf, 0xcf, 0xc3, 0xc3, 0x00, 0x00,
  0xff, 0xff, 0x33, 0x33, 0xcf, 0xcf,
};

// 0-25: ALPHA, 26-35: Num, !?.
const unsigned char spr_chars[] PROGMEM {
  0x0f, 0x05, 0x0f, 0x0f, 0x0b, 0x0e, 0x0f, 0x09,
  0x09, 0x0f, 0x09, 0x06, 0x0f, 0x0b, 0x09, 0x0f,
  0x05, 0x01, 0x0f, 0x09, 0x0d, 0x0f, 0x04, 0x0f,
  0x00, 0x0f, 0x00, 0x08, 0x08, 0x0f, 0x0f, 0x04,
  0x0a, 0x0f, 0x08, 0x08, 0x0f, 0x02, 0x0f, 0x0f,
  0x01, 0x0f, 0x0f, 0x09, 0x0f, 0x0f, 0x05, 0x07,
  0x07, 0x05, 0x0f, 0x0f, 0x05, 0x0b, 0x0b, 0x09,
  0x0d, 0x01, 0x0f, 0x01, 0x0f, 0x08, 0x0f, 0x07,
  0x08, 0x07, 0x0f, 0x0c, 0x0f, 0x09, 0x06, 0x09,
  0x03, 0x0c, 0x03, 0x0d, 0x09, 0x0b, 0x0f, 0x09,
  0x0f, 0x00, 0x01, 0x0f, 0x0d, 0x09, 0x0b, 0x09,
  0x0b, 0x0f, 0x07, 0x04, 0x0f, 0x0b, 0x09, 0x0d,
  0x0f, 0x0a, 0x0e, 0x01, 0x01, 0x0f, 0x0f, 0x0d,
  0x0f, 0x07, 0x05, 0x0f, 0x0b, 0x00, 0x00, 0x01,
  0x0b, 0x03, 0x08, 0x00, 0x00,
};

PROGMEM const byte str_press_a[] = {
  7,// Number characters
  15, 17, 4, 18, 18, 255, 0
};

PROGMEM const byte str_highscore[] = {
  9,// Number characters
  7, 8, 6, 7, 18, 2, 14, 17, 4
};

PROGMEM const byte str_planet[] = {
  6,
  15, 11, 0, 13, 4, 19
};

PROGMEM const byte str_yay[] = {
  4,
  24, 0, 24, 36
};

PROGMEM const byte str_ouch[] {
  5,
  14, 20, 2, 7, 36
};

PROGMEM const byte str_ready[] {
  6,
  17, 4, 0, 3, 24, 36
};

enum GameState {
  START,
  LANDER
};

struct Vec2D {
  float x;
  float y;
};

struct Player {
  Vec2D pos;
  Vec2D vel;
  float angle;
  float fuel;
  float thrust_time;
  bool alive;
  bool landed;
  bool orbiting;
  unsigned int score;
};

enum ParticleType {
  NONE,
  THRUST,
  SMOKE,
  TEXT
};

enum TextType {
  YAY,
  OUCH,
  READY,
  NEWHIGHSCORE
};

struct Particle {
  Vec2D pos;
  Vec2D vel;
  float age;
  ParticleType type;
  TextType text; // general purpose
};
