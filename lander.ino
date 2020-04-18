#include <Arduboy2.h>
#include <EEPROM.h>
#include "lander.h";

#define FRAME_RATE 60
#define PARTICLES 8
#define PARTICLE_MAX_AGE 120
#define LANDRES 64
#define FUEL_MAX 24
#define START_X 30
#define START_Y 60
#define START_VEL_X 12

#define BASE_FLATNESS 2.0f
#define BASE_TERRAIN_HEIGHT 8
#define BASE_GRAVITY -2
#define BASE_FRICTION 0.9f

#define EEPROM_ID_BYTE1 0x4C
#define EEPROM_ID_BYTE2 0x4E

// Higher is lower chance
#define SMOKE_CHANCE 8

// STRUCTURES ------------------------------------------------------------------
float getMagnitude(Vec2D v) {
  return sqrt(pow(v.x, 2) + pow(v.y, 2));
}

// UNIVERSAL -------------------------------------------------------------------
Arduboy2 arduboy;
GameState state;
int w = 128;
int h = 64;
const float FRAME_TIME = 1.0 / FRAME_RATE;
const float LANDSCALE = w / LANDRES;
int frame_count = 0;
int highscore = 0;

// LEVEL VARS ------------------------------------------------------------------
Player player;
Particle particle[PARTICLES];
int last_particle = 0;
int smoke_accumulator = 1;
float elevation[LANDRES];

float scene_width = w;
float base_thrust = 5;
float base_thrust_addition = 1.5; // Thrust increase per second
float turn_rate = PI * 0.8;

float exhaust_speed = 50;
float smoke_rise_max = 0.23;
float smoke_rise_min = -0.05;

float death_velocity = 2.0;
float death_angle = PI / 11;
float death_gradient = 0.25;

float current_flatness = BASE_FLATNESS;
float current_terrain_height = BASE_TERRAIN_HEIGHT;
float current_gravity = BASE_GRAVITY;
float current_friction = BASE_FRICTION;

// VIEWPORT --------------------------------------------------------------------
Vec2D view_pos = {0, 0};
float view_scale = 1;

float dX(float x) {
  x -= view_pos.x;
  if (x < 0) x += scene_width;
  if (x >= scene_width) x -= scene_width;
  return round(x * view_scale);
}

float dY(float y) {
  return round((view_pos.y - y) * view_scale);
}

// BASIC -----------------------------------------------------------------------
void readHighscore() {
  if (EEPROM.read(0) == EEPROM_ID_BYTE1 && EEPROM.read(1)) {
    highscore = word(EEPROM.read(2), EEPROM.read(3));
  } else {
    highscore = 0;
  }
}

void saveHighscore(int score) {
  highscore = score;
  EEPROM.write(0, EEPROM_ID_BYTE1);
  EEPROM.write(1, EEPROM_ID_BYTE2);
  EEPROM.write(2, highByte(highscore));
  EEPROM.write(3, lowByte(highscore));
}

void setup() {
  arduboy.boot();
  arduboy.setFrameRate(FRAME_RATE);
  arduboy.initRandomSeed();
  w = arduboy.width();
  h = arduboy.height();
  if (arduboy.pressed(B_BUTTON)) {
    saveHighscore(0);
  }
  readHighscore();

  state = START;
  startGame(); // Skip straight into it
}

void loop() {
  if (!(arduboy.nextFrame()))
    return;

  frame_count++;
  arduboy.clear();

  if (state == START) {
    //updateDrawStart();
  } else if (state == LANDER) {
    if (player.alive) {
      updateInput();
      updatePlayer();
      updateCamera();
    } else {
      if (arduboy.pressed(B_BUTTON)) {
        startGame();
      }
    }

    drawBackground();

    if (player.alive) {
      drawPlayer();
    }

    updateDrawParticles();

    if (player.orbiting) {
      if (player.score == 0)
        drawStartUI();
      else
        drawWorldIntro();
    } else {
      drawHUD();
    }
  }

  arduboy.display();
}

// LANDER ----------------------------------------------------------------------
void startGame() {
  player.fuel = 12;
  player.score = 0;
  beginLanding();
}

void beginLanding() {
  state = LANDER;

  // Reset player
  player.alive = true;
  player.pos.x = START_X;
  player.pos.y = START_Y;
  player.vel.x = START_VEL_X;
  player.vel.y = 0;
  player.angle = PI;
  player.landed = false;
  player.orbiting = true;

  // Generate level data
  current_flatness = BASE_FLATNESS - (player.score / 15);
  current_terrain_height = BASE_TERRAIN_HEIGHT + player.score / 2;
  current_gravity = BASE_GRAVITY - (player.score / 20.0);
  current_friction = BASE_FRICTION;

  if (current_flatness < 1) current_flatness = 1;
  if (current_terrain_height > 16) current_terrain_height = 16;

  // Wipe particles
  for (int i = 0; i < PARTICLES; i++) {
    particle[i].type = NONE;
  }

  // Generate terrain
  float current = random(0, current_terrain_height * 2);
  float min = current;
  for (int i = 0; i < LANDRES; i++) {
    if (i > 0 && i > LANDRES - abs(elevation[0] - current)) {
      current += random((elevation[0] - current) / (LANDRES - i), elevation[0] - current);
    } else {
      current += (random(0, current_terrain_height) - (current / 2)) / current_flatness;
    }
    elevation[i] = current;
    if (current < min) min = current;
  }

  // Bring heights down
  min -= 0.6;
  for (int i = 0; i < LANDRES; i++) {
    elevation[i] -= min;
  }

  // Ensure there's at least one landing spot
  int i = random(1, LANDRES);
  elevation[i] = elevation[i - 1];
}

float getElevation(float x) {
  float pos = (x / (float)LANDSCALE);
  int left = (int)(pos);
  float transition = pos - left;

  int right = left + 1;
  if (right >= LANDRES) right -= LANDRES;

  float delta = elevation[right] - elevation[left];
  return elevation[left] + (delta * transition);
}

float getGradient(float x) {
  int left = (int)(x / LANDSCALE);
  int right = left + 1;
  if (right >= LANDRES) right -= LANDRES;
  return (elevation[right] - elevation[left]) / (w / (float)LANDRES);
}

void drawBackground() {
  //arduboy.drawFastHLine(0, arduboy.height() - 1, arduboy.width());
  float step = w / LANDRES;

  int prev = LANDRES - 1;
  for (int i = 0; i < LANDRES; i++) {
    float x1 = dX(i * step - step);
    float x2 = dX(i * step);
    if (x1 > x2) {
      // For segments at the screen edge
      arduboy.drawLine(
          x2 - step * view_scale, dY(elevation[prev]),
          x2, dY(elevation[i]));
      arduboy.drawLine(
          x1, dY(elevation[prev]),
          x1 + step * view_scale, dY(elevation[i]));
    } else {
      arduboy.drawLine(
          x1, dY(elevation[prev]),
          x2, dY(elevation[i]));
    }
    prev = i;
  }
}

void updateInput() {
  if (arduboy.pressed(LEFT_BUTTON))
    player.angle += turn_rate * FRAME_TIME;

  if (arduboy.pressed(RIGHT_BUTTON))
    player.angle -= turn_rate * FRAME_TIME;

  if (arduboy.pressed(A_BUTTON)) {
    player.orbiting = false;
    thrustPlayer();
    player.thrust_time += FRAME_TIME;
  } else {
    player.thrust_time = 0;
  }
}

void thrustPlayer() {
  if (player.fuel <= 0) {
    player.thrust_time = 0;
    return;
  }

  player.fuel -= FRAME_TIME;
  if (player.fuel < 0) player.fuel = 0;

  float thrust = (base_thrust + player.thrust_time * base_thrust_addition) * FRAME_TIME;
  player.vel.x += cos(player.angle) * thrust;
  player.vel.y += sin(player.angle) * thrust;

  if (random(SMOKE_CHANCE) < smoke_accumulator) {
    initParticle(THRUST,
        player.pos.x, player.pos.y,
        player.vel.x - cos(player.angle) * exhaust_speed,
        player.vel.y - sin(player.angle) * exhaust_speed);
    smoke_accumulator = 0;
  } else {
    smoke_accumulator++;
  }
}

void updateCamera() {
  // Pan left and right
  float min = view_pos.x + 20;
  float max = view_pos.x + (w - 40.0) / view_scale;
  if (player.pos.x < min)
    view_pos.x += player.pos.x - min;
  if (player.pos.x > max)
    view_pos.x += player.pos.x - max;

  if (player.orbiting)
    view_pos.x = player.pos.x - START_X;

  // Zoom in to terrain
  if (player.pos.y > 25) {
    view_scale = 1;
    view_pos.y = h;
  } else if (player.pos.y > 16) {
    // Brittttle!s
    view_scale = 1 + (25 - player.pos.y) / 3;
    view_pos.y = h / view_scale;
  } else {
    view_scale = 4;
    view_pos.y = 16;
  }
}

bool safeVelocity() {
  return (getMagnitude(player.vel) < death_velocity);
}

bool safeAngle() {
  return (player.angle < PI/2 + death_angle &&
          player.angle > PI/2 - death_angle);
}

bool safeGradient() {
  return (abs(getGradient(player.pos.x)) < death_gradient);
}

void crashPlayer() {
  player.alive = false;
  createExplosion(player.pos.x, player.pos.y);
  int p = initParticle(TEXT, player.pos.x + random(-4, 2), player.pos.y + 3 + random(2), 0, 0);
  particle[p].text = OUCH;
}

void landPlayer() {
  if (player.fuel < FUEL_MAX) {
    player.fuel += FRAME_TIME * 10;
    if (player.fuel >= FUEL_MAX) {
      player.fuel = FUEL_MAX;
      int p = initParticle(TEXT, player.pos.x + random(-4, 2), player.pos.y + 3 + random(2), 0, 0);
      particle[p].text = READY;
    }
  }

  // Landed!
  if (!player.landed) {
    player.landed = true;
    player.score++;

    if (player.score > highscore) {
      saveHighscore(player.score);
      int p_highscore = initParticle(TEXT, player.pos.x + random(-4, 2), player.pos.y + 3 + random(2), 0, 0);
      particle[p_highscore].text = NEWHIGHSCORE;
    }

    int p = initParticle(TEXT, player.pos.x + random(-4, 2), player.pos.y + 3 + random(3), 0, 0);
    particle[p].text = YAY;
  }

  player.vel.y *= -0.1;
  player.vel.x *= -0.1;
  player.angle = PI / 2;
}

void updatePlayer() {
  // Basic physics
  if (!player.orbiting) {
    player.vel.y += current_gravity * FRAME_TIME;
    player.pos.y += player.vel.y * FRAME_TIME;
  }

  player.pos.x += player.vel.x * FRAME_TIME;

  // Wrap angles
  while (player.angle < -PI) { player.angle += PI * 2; }
  while (player.angle > PI) { player.angle -= PI * 2; }

  // Wrap around scene
  if (player.pos.x < 0) {
    player.pos.x += scene_width;
    view_pos.x += scene_width;
  } else if (player.pos.x >= scene_width) {
    player.pos.x -= scene_width;
    view_pos.x -= scene_width;
  }

  // Check for terrain collision
  float elevation = getElevation(player.pos.x);
  if (player.pos.y <= elevation) {
    player.pos.y = elevation;

    if (!safeGradient() || !safeVelocity() || !safeAngle()) {
      crashPlayer();
    } else {
      landPlayer();
    }
  }

  // Check for exiting scene
  if (player.pos.y > h && player.landed)
    beginLanding();
}

void drawPlayer() {
  arduboy.drawLine(dX(player.pos.x), dY(player.pos.y),
                   dX(player.pos.x + cos(player.angle) * 2),
                   dY(player.pos.y + sin(player.angle) * 2));
}

void createExplosion(float x, float y) {
  for (int i = 0; i < PARTICLES; i++) {
    if (random(2) < 1) {
      particle[i].type = THRUST;
      particle[i].pos.x = x;
      particle[i].pos.y = y + 1.0;
      particle[i].vel.x = random(-25, 25);
      particle[i].vel.y = random(-25, 25);
    } else {
      particle[i].type = SMOKE;
      particle[i].pos.x = x + random(-50, 50) / 50.0;
      particle[i].pos.y = y + 1.0 + random(-50, 50) / 50.0;
      particle[i].vel.x = random(-50, 50) / 10.0;
      particle[i].vel.y = random(-50, 50) / 10.0;
    }
    particle[i].age = 1;
  }
}

void endParticle(int index) {
  particle[index].type = NONE;
  last_particle = index;
}

void updateParticle(int i) {
  particle[i].pos.x += particle[i].vel.x * FRAME_TIME;
  particle[i].pos.y += particle[i].vel.y * FRAME_TIME;

  // Wrap around scene
  if (particle[i].pos.x < 0)
    particle[i].pos.x += scene_width;
  else if (particle[i].pos.x >= scene_width)
    particle[i].pos.x -= scene_width;

  particle[i].vel.y += random(smoke_rise_min * 10000.0, smoke_rise_max * 10000.0) / 10000.0;
  particle[i].vel.x *= current_friction;
  particle[i].vel.y *= current_friction;

  float elevation = getElevation(particle[i].pos.x);

  if (particle[i].pos.y <= elevation) {
    if (particle[i].type == THRUST) {
      // Transition THRUST to SMOKE if it hits the ground
      particle[i].pos.y = elevation;
      particle[i].type = SMOKE;
      particle[i].vel.x += 0.5 * ((random(2) < 1) ? -particle[i].vel.y : particle[i].vel.y);
      particle[i].vel.y *= -0.03;
    } else {
      // Smoke follows ground
      particle[i].pos.y = elevation;
      particle[i].vel.x *= 0.85;
      particle[i].vel.y += abs(particle[i].vel.x / 2.5);
    }
  }

  if (particle[i].type == SMOKE || particle[i].type == THRUST) {
    if (getMagnitude(particle[i].vel) < 0.5)
      endParticle(i);
  }
}

void drawParticle(int i) {
  float vel;
  switch (particle[i].type) {
    case SMOKE:
      vel = getMagnitude(particle[i].vel);
      if (vel < 1.5)
        arduboy.drawBitmap(dX(particle[i].pos.x) - 2, dY(particle[i].pos.y) - 5, spr_sm_smoke, 4, 4, WHITE);
      else if (vel < 10)
        arduboy.drawBitmap(dX(particle[i].pos.x) - 4, dY(particle[i].pos.y) - 4, spr_smoke, 8, 8, WHITE);
      break;
    case THRUST:
      if (!player.alive || getMagnitude(particle[i].vel) > 25)
        arduboy.drawPixel(dX(particle[i].pos.x), dY(particle[i].pos.y), WHITE);
      break;
    case TEXT:
      if (particle[i].text == YAY)
        drawText(dX(particle[i].pos.x), dY(particle[i].pos.y), str_yay);
      else if (particle[i].text == OUCH)
        drawText(dX(particle[i].pos.x), dY(particle[i].pos.y), str_ouch);
      else if (particle[i].text == NEWHIGHSCORE)
        drawText(dX(particle[i].pos.x), dY(particle[i].pos.y), str_highscore);
      else if (particle[i].text == READY)
        drawText(dX(particle[i].pos.x), dY(particle[i].pos.y), str_ready);
      break;
  }
}

void drawForegroundSmoke() {
  float vel;
  for (int i = 0; i < PARTICLES; i++) {
    if (particle[i].type != SMOKE) continue;

    vel = getMagnitude(particle[i].vel);
    if (vel < 1.5) {
      arduboy.drawBitmap(
          dX(particle[i].pos.x) - 2, dY(particle[i].pos.y) - 5,
          spr_sm_smoke_fill, 4, 4, BLACK);
    } else if (vel < 10) {
      arduboy.drawBitmap(
          dX(particle[i].pos.x) - 4, dY(particle[i].pos.y) - 4,
          spr_smoke_fill, 8, 8, BLACK);
    }
  }
}

void updateDrawParticles() {
  for (int i = 0; i < PARTICLES; i++) {
    if (particle[i].type == NONE) continue;
    if (particle[i].age > PARTICLE_MAX_AGE) {
      endParticle(i);
      continue;
    } else {
      particle[i].age++;
    }

    // Basic physics
    updateParticle(i);
    drawParticle(i);
  }

  drawForegroundSmoke();
}

int initParticle(ParticleType type, float x, float y, float xVel, float yVel) {
  int i;

  if (last_particle >= 0) {
    i = last_particle;
    last_particle = -1;
  } else {
    int noneIndex = -1;
    int oldestIndex = 0;
    for (i = 0; i < PARTICLES; i++) {
      if (particle[i].type == NONE) {
        noneIndex = i;
        break;
      }
      if (particle[i].age >= particle[oldestIndex].age) {
        oldestIndex = i;
      }
    }
    i = (noneIndex >= 0) ? noneIndex : oldestIndex;
  }

  particle[i].type = type;
  particle[i].pos.x = x;
  particle[i].pos.y = y;
  particle[i].vel.x = xVel;
  particle[i].vel.y = yVel;
  particle[i].age = 0;
  return i;
}

void drawText(int x, int y, const uint8_t *message) {
  byte strLen = pgm_read_byte(message);
  int x_offset = 0;
  for (int i = 0; i < strLen; i++) {
    byte letter = pgm_read_byte(++message);

    if (letter < 64)
      arduboy.drawBitmap(x + x_offset, y, spr_chars + letter * 3, 3, 4, WHITE);

    x_offset += 4;
  }
}

void drawStartUI() {
  arduboy.drawBitmap((w - 46) / 2, (h - 16) / 2, spr_title, 46, 8, WHITE);

  if (highscore) {
    drawText((w - 9 * 4) / 2 - 4, h / 2 + 4, str_highscore);
    drawNumber((w + 9 * 4) / 2 - 3, h / 2 + 4, highscore);
  } else {
    drawText((w - 7 * 4) / 2, h / 2 + 4, str_press_a);
  }
}

// From https://stackoverflow.com/questions/8671845/iterating-through-digits-in-integer-in-c
int numDigits(int x) {
  x = abs(x);
  return (x < 10 ? 1 :
         (x < 100 ? 2 :
         (x < 1000 ? 3 :
         (x < 10000 ? 4 :
         (x < 100000 ? 5 :
         (x < 1000000 ? 6 :
         (x < 10000000 ? 7 :
         (x < 100000000 ? 8 :
         (x < 1000000000 ? 9 :
         10)))))))));
}

void printDigits(int x, int y, int n) {
  if (n >= 10)
    printDigits(x - 4, y, n / 10);

  int digit = n % 10;
  arduboy.drawBitmap(x, y, spr_chars + 26 * 3 + digit * 3, 3, 4, WHITE);
}

void drawNumber(int x, int y, int n) {
  printDigits(x + numDigits(n) * 4 - 4, y, n);
}

void drawWorldIntro() {
  // Draw PLANET [NUM]
  drawText(w / 2 - 16, h / 2, str_planet);
  drawNumber(w / 2 + 10, h / 2, player.score + 1);
}

void drawHUD() {
  // Fuel
  int fuel_h = ceil((player.fuel / FUEL_MAX) * 11);
  arduboy.drawFastHLine(w - fuel_h, 63, fuel_h);

  if (!safeGradient())
    arduboy.drawBitmap(w - 11, 58, spr_chars + 6 * 3, 3, 4, WHITE);

  if (!safeAngle())
    arduboy.drawBitmap(w - 7, 58, spr_chars + 0, 3, 4, WHITE);

  if (!safeVelocity())
    arduboy.drawBitmap(w - 3, 58, spr_chars + 18 * 3, 3, 4, WHITE);
}
