#include "database.h"

const int16_t kTableSize = 10000 * sizeof(SavedExperiment) + sizeof(EDB_Header);

File savedExperimentsFile;

inline void writer(unsigned long address, const byte *data,
                   unsigned int datalen) {
  bool seekOk = savedExperimentsFile.seek(address);
  if (seekOk) {
    int written = savedExperimentsFile.write(data, datalen);
    savedExperimentsFile.flush();
    Serial.print("Wrote db record: ");
    Serial.println((char *)data);
  }
  // TODO error handling
}

inline void reader(unsigned long address, byte *data, unsigned int datalen) {
  bool seekOk = savedExperimentsFile.seek(address);
  if (seekOk) {
    int result = savedExperimentsFile.read(data, datalen);
  } else {
    memset(data, 0, datalen);
  }
}

SavedExperimentsDB::SavedExperimentsDB(SdFat &sd, const char *filename)
    : _sd(sd), _db(&writer, &reader) {
  if (_sd.exists(filename)) {
    savedExperimentsFile = _sd.open(filename, O_RDWR);
    // Apparently sometimes fails on first attempt so retry
    if (!savedExperimentsFile) {
      Serial.println("Failed to open db, retrying");
      savedExperimentsFile = _sd.open(filename, O_RDWR);
    }
    if (savedExperimentsFile) {
      EDB_Status result = _db.open(0);
      if (result != EDB_OK) {
        result = _db.create(0, kTableSize, sizeof(SavedExperiment));
        // TODO error handling
      }
      else {
        Serial.println("Opened db");
      }
    }
  } else {
    savedExperimentsFile = _sd.open(filename, FILE_WRITE);
    if (savedExperimentsFile) {
      Serial.println("Creating db");
      EDB_Status result = _db.create(0, kTableSize, sizeof(SavedExperiment));
      // TODO error handling
      Serial.print("Staus was: ");
      Serial.println(result);
    }
  }
}

EDB_Status SavedExperimentsDB::addExperiment(const SavedExperiment &exp)
{
    EDB_Status result = _db.appendRec(EDB_REC exp);
    return result;
}


// Note this is 1 indexed due to the underlying library
EDB_Status SavedExperimentsDB::getExperiment(int idx, SavedExperiment &exp)
{
  EDB_Status result = _db.readRec(idx, EDB_REC exp);
  return result;
}