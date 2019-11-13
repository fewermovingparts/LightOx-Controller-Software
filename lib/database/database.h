#include <Arduino.h>
#include <EDB.h>
#include <SdFat.h>
#include <stdint.h>

struct SavedExperiment {
  char name[41];
  int datetime[7];
  int32_t time;
  int32_t irradience;
};

class SavedExperimentsDB {
 public:
  SavedExperimentsDB(SdFat &sd, const char *filename);
  EDB_Status addExperiment(const SavedExperiment &exp);
  EDB_Status getExperiment(int number, SavedExperiment &exp);
  unsigned long count() { return _db.count(); }
  void clear() { _db.clear(); }

 private:
  SdFat &_sd;
  EDB _db;
};