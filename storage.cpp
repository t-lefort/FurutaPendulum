#include "storage.h"
#include <SD.h>
#include <math.h>

namespace {
bool sdOk = false;
File logFile;
uint32_t lastFlush = 0;

// Python : struct.pack("<IHHHIIH", magic, version, Tg, Tl, gp, lp, actions)
struct __attribute__((packed)) SplitHeader {
  uint32_t magic;               // 'SPL1'
  uint16_t version;
  uint16_t globalTilings;
  uint16_t localTilings;
  uint32_t globalPerTiling;
  uint32_t localPerTiling;
  uint16_t actions;
};
constexpr uint32_t SPLIT_MAGIC = 0x53504C31u;
static_assert(sizeof(SplitHeader) == 20, "L'en-tete SPL1 doit faire 20 octets");

struct __attribute__((packed)) StateFile {
  uint32_t magic;               // 'QST1'
  uint16_t version;
  uint16_t payloadBytes;
  QLearning::PersistState state;
  uint32_t weightsChecksum;
  uint32_t payloadChecksum;
};
constexpr uint32_t STATE_MAGIC = 0x51535431u;

uint32_t fnv1a(const uint8_t *bytes, size_t count) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < count; ++i) {
    hash ^= bytes[i];
    hash *= 16777619u;
  }
  return hash;
}

uint32_t stateChecksum(const StateFile &file) {
  return fnv1a(reinterpret_cast<const uint8_t*>(&file.state),
               sizeof(file.state) + sizeof(file.weightsChecksum));
}
} // namespace

bool Storage::begin() {
  sdOk = SD.begin(BUILTIN_SDCARD);
  return sdOk;
}

bool Storage::available() { return sdOk; }

uint32_t Storage::qChecksum(const float *data, size_t count) {
  return fnv1a(reinterpret_cast<const uint8_t*>(data),
               count * sizeof(float));
}

bool Storage::saveQTable(const char *path, const float *data, size_t count) {
  if (!sdOk || count != QLearning::WEIGHT_COUNT) return false;
  SD.remove(path);
  File file = SD.open(path, FILE_WRITE);
  if (!file) return false;

  const SplitHeader header = {
      SPLIT_MAGIC, 1,
      (uint16_t)TC_GLOBAL_TILINGS, (uint16_t)TC_LOCAL_TILINGS,
      QLearning::GLOBAL_PER_TILING, QLearning::LOCAL_PER_TILING,
      (uint16_t)QL_N_ACT};
  const size_t headerWritten =
      file.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  const size_t dataWritten =
      file.write(reinterpret_cast<const uint8_t*>(data), count * sizeof(float));
  file.flush();
  file.close();
  return headerWritten == sizeof(header) &&
         dataWritten == count * sizeof(float);
}

bool Storage::loadQTable(const char *path, float *data, size_t count) {
  if (!sdOk || count != QLearning::WEIGHT_COUNT) return false;
  File file = SD.open(path, FILE_READ);
  if (!file) return false;
  const size_t expectedBytes = sizeof(SplitHeader) + count * sizeof(float);
  if ((size_t)file.size() != expectedBytes) {
    file.close();
    return false;
  }

  SplitHeader header;
  const bool headerOk =
      file.read(reinterpret_cast<uint8_t*>(&header), sizeof(header)) ==
          sizeof(header) &&
      header.magic == SPLIT_MAGIC &&
      header.version == 1 &&
      header.globalTilings == TC_GLOBAL_TILINGS &&
      header.localTilings == TC_LOCAL_TILINGS &&
      header.globalPerTiling == QLearning::GLOBAL_PER_TILING &&
      header.localPerTiling == QLearning::LOCAL_PER_TILING &&
      header.actions == QL_N_ACT;
  if (!headerOk) {
    file.close();
    return false;
  }

  const size_t bytes = count * sizeof(float);
  const bool dataOk =
      file.read(reinterpret_cast<uint8_t*>(data), bytes) == bytes;
  file.close();
  return dataOk;
}

bool Storage::saveQLState(const char *path,
                          const QLearning::PersistState &state,
                          uint32_t weightsChecksum) {
  if (!sdOk) return false;
  StateFile stateFile = {
      STATE_MAGIC, 1, (uint16_t)sizeof(QLearning::PersistState),
      state, weightsChecksum, 0};
  stateFile.payloadChecksum = stateChecksum(stateFile);

  SD.remove(path);
  File file = SD.open(path, FILE_WRITE);
  if (!file) return false;
  const bool ok =
      file.write(reinterpret_cast<const uint8_t*>(&stateFile),
                 sizeof(stateFile)) == sizeof(stateFile);
  file.flush();
  file.close();
  return ok;
}

bool Storage::loadQLState(const char *path, QLearning::PersistState &state,
                          uint32_t weightsChecksum) {
  if (!sdOk) return false;
  File file = SD.open(path, FILE_READ);
  if (!file || (size_t)file.size() != sizeof(StateFile)) {
    if (file) file.close();
    return false;
  }
  StateFile stateFile;
  const bool readOk =
      file.read(reinterpret_cast<uint8_t*>(&stateFile), sizeof(stateFile)) ==
          sizeof(stateFile);
  file.close();
  if (!readOk ||
      stateFile.magic != STATE_MAGIC ||
      stateFile.version != 1 ||
      stateFile.payloadBytes != sizeof(QLearning::PersistState) ||
      stateFile.weightsChecksum != weightsChecksum ||
      stateFile.payloadChecksum != stateChecksum(stateFile))
    return false;
  state = stateFile.state;
  return true;
}

bool Storage::logStart() {
  if (!sdOk) return false;
  if (logFile) logFile.close();
  SD.mkdir("/logs");
  char name[32];
  for (int i = 1; i < 10000; ++i) {
    snprintf(name, sizeof(name), "/logs/log_%04d.csv", i);
    if (!SD.exists(name)) break;
  }
  logFile = SD.open(name, FILE_WRITE);
  if (!logFile) return false;
  logFile.println("time_ms,mode,episode,theta,alpha,theta_dot,alpha_dot,"
                  "duty,action,reward,episode_reward,epsilon,epsilon_top,"
                  "learning_rate");
  return true;
}

void Storage::logRow(uint32_t ms, uint8_t mode, uint32_t episode,
                     const PendulumState &s, int8_t action,
                     float stepReward, float episodeReward, float epsilon,
                     float epsilonTop, float learningRate) {
  if (!logFile) return;
  logFile.printf("%lu,%u,%lu,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%.3f,%.2f,"
                 "%.4f,%.4f,%.6f\n",
                 ms, mode, episode, s.theta, s.alpha, s.thetaDot, s.alphaDot,
                 s.duty, action, stepReward, episodeReward, epsilon,
                 epsilonTop, learningRate);
  if (millis() - lastFlush > 1000) {
    logFile.flush();
    lastFlush = millis();
  }
}

void Storage::logStop() {
  if (logFile) {
    logFile.flush();
    logFile.close();
  }
}
