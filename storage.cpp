#include "storage.h"
#include <SD.h>

static bool sdOk = false;
static File logFile;
static uint32_t lastFlush = 0;

struct QHeader {
  uint32_t magic;      // 'QTB1'
  uint16_t version;
  uint16_t nAlpha, nAdot, nAct;
  uint16_t reserved;
};
static constexpr uint32_t Q_MAGIC = 0x51544231;

bool Storage::begin() {
  sdOk = SD.begin(BUILTIN_SDCARD);
  return sdOk;
}
bool Storage::available() { return sdOk; }

bool Storage::saveQTable(const char *path, const float *data, size_t count) {
  if (!sdOk) return false;
  SD.remove(path);
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  QHeader h = { Q_MAGIC, 1, QL_N_ALPHA, QL_N_ADOT, QL_N_ACT, 0 };
  f.write((uint8_t*)&h, sizeof(h));
  f.write((uint8_t*)data, count * sizeof(float));
  f.close();
  return true;
}

bool Storage::loadQTable(const char *path, float *data, size_t count) {
  if (!sdOk) return false;
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  QHeader h;
  if (f.read((uint8_t*)&h, sizeof(h)) != sizeof(h) ||
      h.magic != Q_MAGIC ||
      h.nAlpha != QL_N_ALPHA || h.nAdot != QL_N_ADOT || h.nAct != QL_N_ACT) {
    f.close();
    return false;
  }
  const size_t n = f.read((uint8_t*)data, count * sizeof(float));
  f.close();
  return n == count * sizeof(float);
}

bool Storage::logStart() {
  if (!sdOk) return false;
  if (logFile) logFile.close();
  SD.mkdir("/logs");
  char name[32];
  for (int i = 1; i < 10000; i++) {
    snprintf(name, sizeof(name), "/logs/log_%04d.csv", i);
    if (!SD.exists(name)) break;
  }
  logFile = SD.open(name, FILE_WRITE);
  if (!logFile) return false;
  logFile.println("time_ms,mode,episode,theta,alpha,theta_dot,alpha_dot,"
                  "duty,action,reward,episode_reward,epsilon");
  return true;
}

void Storage::logRow(uint32_t ms, uint8_t mode, uint32_t episode,
                     const PendulumState &s, int8_t action,
                     float stepReward, float episodeReward, float epsilon) {
  if (!logFile) return;
  logFile.printf("%lu,%u,%lu,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%.3f,%.2f,%.3f\n",
                 ms, mode, episode, s.theta, s.alpha, s.thetaDot, s.alphaDot,
                 s.duty, action, stepReward, episodeReward, epsilon);
  if (millis() - lastFlush > 1000) { logFile.flush(); lastFlush = millis(); }
}

void Storage::logStop() {
  if (logFile) { logFile.flush(); logFile.close(); }
}
