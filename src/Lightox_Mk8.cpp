// Version 8.0
// Conversion to capacitive touch screen
// Improved switching between screens as clear added to "Quit"
// Improved screen starting by removing some lines in the tagval.

// Version 7.0 - As released back to Lightox on 07/03/19
// fixed from V6:
// 10s LED warm up - addressed, but has a v.low start up light level (5/255 =
// 2%). Lid switch was too slow - improved Software version added to info screen
// Not yet fixed:
// Occasional skip on project squiggle
// Screen upside down - there does not seem to be an easy fix
// FTImpl.Cmd_Rotate(32768) seems to apply to bitmaps not the screen? Null
// files? Memory stick check - partially written, but remmed out as seems to get
// stuck with something in the serial buffer?

#include "SystemHeaders.h"

#include "database.h"

enum NotepadResult { kNotepadResultQuit, kNotepadResultSave };
// Buffer for the notepads
#define MAX_FT_LINES 2  // Max FT_LINES allows to Display
struct Notepad_buffer {
  char *temp;  // The "*" makes it an indirection operator, which means it
               // points to the address of a variable, so temp = x, points to
               // the address of x.
  char notepad[MAX_FT_LINES][80];
} Buffer;

// Function prototypes
int16_t BootupConfigure();
void Calibrate();
void CheckStorageDevicePresence();
String ConvertTimeDate(int TimeDate[]);
int32_t Dec2Ascii(char *pSrc, int32_t value);
void flash_data(char *pstring, boolean Print);
void Loadimage2ram();
NotepadResult Notepad(const char *initialText = "\0");
void ReadTimeDate(int TimeDate[]);
void RTC_init();
void SetTimeDate(int d, int mo, int y, int h, int mi, int s);

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(53);  // Data wire is plugged into pin 53 on the Arduino pulled
                      // up with 4.7k resistor

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Software serial for Flash drive
SoftwareSerial mySerial(A15, A14);  // RX, TX

#define FLASH_IN A13
#define RTC_CS 26   // RTC chip select
#define RTC_GND 38  // RTC 0V for power
#define RTC_5V 36   // RTC 5V for power
#define FAN 22      // Fan on/off
#define LID 48      // Lid detection
#define XFAULT 2
#define PWM 6
#define ADM 5

// Adafruit UV sensor
Adafruit_VEML6070 uv = Adafruit_VEML6070();  // UV sensor

/* SD card object*/
FT_SD FtSd(FT_SD_CSPIN);
/* sd_present holds error values during initialization.  0 means no error and
 * all other errors are non zero value */
FT_SDStatus sd_present;
/*SD file object for file accessing*/
FT_SDFile Imagefile;
/* Global object for transport */
FT801IMPL_SPI FTImpl(FT_CS_PIN, FT_PDN_PIN, FT_INT_PIN);

// File system object.
SdFat sd;

// Log file.
SdFile file;

//                              0           1         2         3        4 5 6
//                              7         8           9          10
char imagename[11][12] = {"Lightox.jpg", "Run.jpg", "Run.jpg", "Run.jpg",
                          "Run.jpg",     "Run.jpg", "Run.jpg", "Sam.jpg",
                          "Carrie.jpg",  "Run.jpg", "Run.jpg"};
int Screen = 0;  // Screen =   0             1          2            3 4 5 6
// char imagename[7][12] =
// {"Lightox.jpg","Menu.jpg","Date.jpg","Project.jpg","TIE.jpg","Sam.jpg","Carrie.jpg"};
boolean RecordOn = true;
int32_t Current = 100, NewCurrent, NC;  // Current in %
float SetCurrent = 1.2 / 5.0 * 255;     // SetCurrent in 0-5V converted to 0-255
int iTime, ds, OldiTime, it;
unsigned long msTime, msTimeLid;
int uv2 = 0;
uint16_t uvValue;
char uvPrintVal[8];
char uvPrint[15] = {'u', 'v', ' ', '=', ' ', '0', '0', '0', '0', '0', '\0'};
char tempPrintValA[3], tempPrintValB[3];
char tempPrint[15] = {'t', 'e', 'm', 'p', ' ', '=', ' ',
                      '0', '0', '0', '.', '0', '\0'};
char OutputValue[5];
// int TimeStepUp = 1, TimeStepUpCount = 0, TimeStepDown = 1, TimeStepDownCount
// = 0;
char TimeDateString[18];
int TimeAndDate[7];
int TimePointer[] = {4, 5, 6, 2, 1, 0};
int TimeMax[] = {31, 12, 99, 23, 59, 59};
int TimeMin[] = {1, 1, 19, 0, 0, 0};
int TimeDigit = 2;
char ProjectString[] =
    "WWWWWWWWWW"
    "WWWWWWWWWW"
    "WWWWWWWWWW"
    "WWWWW67890";
/*
char ProjectString[150] = {'P', 'r', 'o', 'j', 'e', 'c', 't', ' ', 'd', 'e',
                           's', 'c', 'r', 'i', 'p', 't', 'i', 'o', 'n', ' ',
                           'i', 'n', ' ', 'u', 'p', ' ', 't', 'o', ' ', '4',
                           '0', ' ', 'c', 'h', 'a', 'r', 's', '\0'};
                           */
char LogFileName[20] = {'/', 'L', 'O', 'G', 'S', '/', 'L', 'O', 'G',
                        '0', '0', '0', '0', '0', '.', 'C', 'S', 'V'};
int LogRef = 0, LineCount = 0;
char LineCountString[5];
int ProjectDigit = 9;
int line = 0;
int key = 0;
char Letter;
uint8_t DallasAddress[8];
int32_t EnergyDensity, NewEnergyDensity,
    NED;  // value stored in EEPROM from callibration in mW/mm^2
int eeAddress = 0;
const int kCalibrationAddress = 10;
const byte kCalibrationMagicByte = 0xAA;
const int kCalibrationNumBytes = 24;
bool needsCalibration = true;
bool FirstPass = false, LidOpen = false;

const int32_t kDefaultIrradience = 50;
const int32_t kDefaultTime = 5 * 60;

struct Experiment {
  char name[41];
  int datetime[7];
  int32_t time;
  int32_t irradience;
  int32_t energy;
};

Experiment currentExp;
int16_t browseExperimentsStartExpIdx = -1;

// Issues including FTImpl twice so this need to be here :(
class KeyPressTracker {
 public:
  KeyPressTracker(FT801IMPL_SPI *FTImpl)
      : ftImpl(FTImpl), buttonPressLastTag(0){};

  // Returns the tag that has been pressed and release, or zero if
  // no tag pressed this call. Returns the currently pressed tag in
  // currentTag.
  uint8_t getButtonPressTag(uint8_t &currentTag) {
    uint8_t buttonPressTag = 0;
    currentTag = ftImpl->Read(REG_TOUCH_TAG);

    if (currentTag != 0) {
      if (buttonPressLastTag != currentTag) {
        buttonPressLastTag = currentTag;
      }
    } else {
      if (buttonPressLastTag != 0) {
        if (!ftImpl->IsPendown()) {
          buttonPressTag = buttonPressLastTag;
        }
        buttonPressLastTag = 0;
      }
    }
    return buttonPressTag;
  };

  uint8_t waitForChange(uint8_t &currentTag) {
    uint8_t buttonPressTag = 0;
    const uint8_t prevSelectedTag = currentTag;
    do {
      buttonPressTag = getButtonPressTag(currentTag);
    } while (0 == buttonPressTag && prevSelectedTag == currentTag);
    return buttonPressTag;
  };

 private:
  FT801IMPL_SPI *ftImpl;
  uint8_t buttonPressLastTag;
};

// Font width utility functions

// get the address of the font width table
static uint32_t getFontTableStartAddr() {
  const uint32_t addr = FTImpl.Read32(0xffffc);
  return addr;
}

// Internal: Get the width of a character in a given font
static uint8_t getCharWidth(uint32_t fontTableStartAddr, uint8_t character,
                            uint8_t font) {
  const uint32_t charWidthAddr =
      (fontTableStartAddr + (148 * (font - 16))) + character;
  const uint8_t width = FTImpl.Read(charWidthAddr);
  return width;
}

// API used to calculate the width of the character
uint8_t Ft_Gpu_Rom_Font_WH(uint8_t Char, uint8_t font) {
  const uint32_t ptr = getFontTableStartAddr();
  // Read Width of the character
  const uint32_t Wptr =
      (ptr + (148 * (font - 16))) + Char;  // (table starts at font 16)
  const uint8_t Width = FTImpl.Read(Wptr);
  return Width;
}

// Calculate the display width of a null terminated string in a given font
static int16_t stringPixelWidth(const char *str, uint8_t font) {
  const uint32_t fontTableStartAddr = getFontTableStartAddr();
  int16_t width = 0;
  for (; *str; ++str) {
    width += getCharWidth(fontTableStartAddr, *str, font);
  }
  return width;
}

// Screen calibration

static bool getCalibrationRequired() {
  return EEPROM.read(kCalibrationAddress) != kCalibrationMagicByte;
}

static void calibrateAndStore() {
  Calibrate();
  Serial.println("Storing calibration:");
  for (int i = 0; i < kCalibrationNumBytes; ++i) {
    const uint8_t val = FTImpl.Read(REG_CTOUCH_TRANSFORM_A + i);
    EEPROM.write(kCalibrationAddress + 1 + i, val);
    Serial.println(val);
  }
  EEPROM.write(kCalibrationAddress, kCalibrationMagicByte);
}

static void loadCalibration() {
  for (int i = 0; i < kCalibrationNumBytes; ++i) {
    const uint8_t val = EEPROM.read(kCalibrationAddress + 1 + i);
    FTImpl.Write(REG_CTOUCH_TRANSFORM_A + i, val);
  }
}

void setup(void) {
  pinMode(11, INPUT);  // Uno SPI pins (11, 12 & 13) not used on Mega
  pinMode(12, INPUT);  // but are connected to SPI by wires from 51, 52 & 53 on
  pinMode(13, INPUT);  // this version, so made inputs to avoid complications
  pinMode(FAN, OUTPUT);
  pinMode(LID, INPUT);
  pinMode(FLASH_IN, INPUT);

  pinMode(RTC_CS, OUTPUT);     // chip select
  digitalWrite(RTC_CS, HIGH);  // deselect RTC
  pinMode(RTC_GND, OUTPUT);
  pinMode(RTC_5V, OUTPUT);
  digitalWrite(RTC_GND, LOW);
  digitalWrite(RTC_5V, HIGH);
  digitalWrite(FAN, LOW);

  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println("Setup....");

  sensors.begin();  // start up the Dallas sensors library
  // Dallas Address = 40,121,248,228,7,0,0,181 for Box 2.
  sensors.getAddress(DallasAddress, 0);  // get address of device 0
  // reading temperatures here works if before RTC_init, but fails if after
  Serial.print("Dallas Address = ");
  for (int8_t i = 0; i < 8; i++) {
    Serial.print(DallasAddress[i]);
    if (i < 7) Serial.print(",");
  }
  Serial.println(" ");

  /*Initialize the SD object. Screen error message can only be displayed when
   * the FT801 is ready*/
  sd_present = FtSd.Init();  //<---starts SPI and kills onewire
  BootupConfigure();
  CheckStorageDevicePresence();

  Serial.print("CS pins (4,10,26) = ");
  Serial.print(digitalRead(FT_SD_CSPIN));
  Serial.print(digitalRead(FT_CS_PIN));
  Serial.println(digitalRead(RTC_CS));

  RTC_init();  // kills OneWire if we don't make the correction

  uv.begin(VEML6070_1_T);  // pass in the integration time constant

  yield();  // from bmp program - what does it do?

  Serial.print("Initializing SD card...");  //<---is there a clash with FT_SD?
  if (!sd.begin(FT_SD_CSPIN)) {             // Was SD_CS
    Serial.println("failed!");
  }
  Serial.println("OK!");

  EEPROM.get(eeAddress, EnergyDensity);
  // EnergyDensity = 188;
  Serial.print("Stored Energy Density = ");
  Serial.println(EnergyDensity);
  // Energy =
  //    (float)Time * (float)EnergyDensity / 1000.0 * (float)Intensity / 100.0;

  pinMode(ADM, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(XFAULT, INPUT);
  analogWrite(ADM, 0);  // works well if current starts at zero.
  analogWrite(PWM, 0);
  delay(1000);

  Serial.print("CS pins (4,10,26) = ");
  Serial.print(digitalRead(FT_SD_CSPIN));
  Serial.print(digitalRead(FT_CS_PIN));
  Serial.println(digitalRead(RTC_CS));
  Serial.println("Reading date & time...");
  ReadTimeDate(TimeAndDate);
  Serial.println("D&T read...");
  Serial.println(ConvertTimeDate(TimeAndDate));

  if (getCalibrationRequired()) {
    calibrateAndStore();
  } else {
    loadCalibration();
  }

  Screen = 0;
  Loadimage2ram();

  Serial.println("Setup complete.....");
}

const uint32_t kColourPrimary = 0x2A5673 + 0x001000;  // dark greeny
const uint32_t kColourSeconday = 0x6A8CA5;            // lighter
const uint32_t kColourLight = 0xD2D8E1;

enum HeldSlider { kHeldSliderTime, kHeldSliderIrradience, kHeldSliderEnergy };
static HeldSlider heldSlider = kHeldSliderIrradience;

enum class DisplayScreen {
  kDisplayScreenHome = 0,
  kDisplayScreenNewExp,
  kDisplayScreenBrowseExperiments,
  kDisplayScreenShowSavedExp,
  kDisplayScreenExpSettings,
  kDisplayScreenRun,
  OptionsScreen,
  kDisplayScreenNone,
};
bool screenJustSelected = true;
DisplayScreen currentScreen = DisplayScreen::kDisplayScreenNone;  // Home;
const uint8_t kFont = 26;

static void setNextScreen(DisplayScreen screen) {
  screenJustSelected = true;
  currentScreen = screen;
}

const int16_t kBorderPixels = 10;
const int16_t kBottomButtonHeight = 38;
const int16_t kBottomButtonWidth = 94;

static uint16_t getButtonOptions(uint8_t buttonTag, uint8_t currentPressedTag) {
  return buttonTag == currentPressedTag ? FT_OPT_FLAT : 0;
}

static void drawTaggedButton(uint8_t tag, int16_t x, int16_t y, uint16_t w,
                             uint16_t h, const char *label,
                             uint8_t currentPressedTag) {
  FTImpl.Tag(tag);
  FTImpl.Cmd_Button(x, y, w, h, kFont, getButtonOptions(tag, currentPressedTag),
                    label);
}

static void drawBottomLeftButton(uint8_t tag, const char *label,
                                 uint8_t currentPressedTag) {
  drawTaggedButton(tag, kBorderPixels,
                   FT_DISPLAYHEIGHT - kBottomButtonHeight - kBorderPixels,
                   kBottomButtonWidth, kBottomButtonHeight, label,
                   currentPressedTag);
}

static void drawBottomMiddleButton(uint8_t tag, const char *label,
                                   uint8_t currentPressedTag) {
  drawTaggedButton(tag, (FT_DISPLAYWIDTH - kBottomButtonWidth) / 2,
                   FT_DISPLAYHEIGHT - kBottomButtonHeight - kBorderPixels,
                   kBottomButtonWidth, kBottomButtonHeight, label,
                   currentPressedTag);
}

static void drawBottomRightButton(uint8_t tag, const char *label,
                                  uint8_t currentPressedTag) {
  drawTaggedButton(tag, FT_DISPLAYWIDTH - kBottomButtonWidth - kBorderPixels,
                   FT_DISPLAYHEIGHT - kBottomButtonHeight - kBorderPixels,
                   kBottomButtonWidth, kBottomButtonHeight, label,
                   currentPressedTag);
}

void showExpHeader(const char *expName, int TimeDate[7]) {
  FTImpl.SaveContext();
  FTImpl.ScissorXY(kBorderPixels, kBorderPixels);
  const int16_t headerWidth = FT_DISPLAYWIDTH - 2 * kBorderPixels;
  FTImpl.ScissorSize(headerWidth, 40);
  FTImpl.ClearColorRGB(kColourPrimary);
  FTImpl.Clear(1, 0, 0);
  FTImpl.ColorRGB(0xff, 0xff, 0xff);
  String s(expName);
  if (TimeDate) {
    s.concat(" - ");
    s.concat(ConvertTimeDate(TimeDate));
  }

  const uint8_t font = [&s]() {
    const uint8_t fontList[] = {28, 27, 26};
    for (const auto &testFont : fontList) {
      if (stringPixelWidth(s.c_str(), testFont) <= headerWidth - 2) {
        return testFont;
      }
    }
    return uint8_t(26);
  }();
  FTImpl.Cmd_Text(FT_DISPLAYWIDTH / 2, 30, font, FT_OPT_CENTER, s.c_str());
  FTImpl.RestoreContext();
}

void homeScreen() {
  Serial.println("TRACE: homeScreen()");

  // Reset some global state
  browseExperimentsStartExpIdx = -1;

  DisplayScreen nextScreen = DisplayScreen::kDisplayScreenHome;
  KeyPressTracker kpt(&FTImpl);
  uint8_t selectedTag = 0;
  do {
    // Display the logo
    FTImpl.Cmd_DLStart();

    FTImpl.Begin(FT_BITMAPS);      // Start a new graphics primitive
    FTImpl.Vertex2ii(0, 0, 0, 0);  // Draw primitive
    FTImpl.BitmapHandle(0);
    FTImpl.BitmapSource(0);
    FTImpl.BitmapLayout(FT_RGB565, 480L * 2, 272);
    FTImpl.BitmapSize(FT_NEAREST, FT_BORDER, FT_BORDER, 480, 272);

    FTImpl.Cmd_FGColor(kColourPrimary);
    FTImpl.TagMask(1);

    const int16_t buttonWidth = 120;
    const uint8_t kNewExperimentTag = 11;
    drawTaggedButton(kNewExperimentTag, FT_DISPLAY_HSIZE / 4 - buttonWidth / 2,
                     200, buttonWidth, 30, "New experiment", selectedTag);

    const uint8_t kPrevExperimentTag = 12;
    drawTaggedButton(kPrevExperimentTag,
                     FT_DISPLAY_HSIZE * 3 / 4 - buttonWidth / 2, 200,
                     buttonWidth, 30, "Rerun experiment", selectedTag);

    const uint8_t kOptionsButtonTag = 13;
    drawTaggedButton(kOptionsButtonTag, FT_DISPLAYWIDTH - 10 - 20, 10, 20, 20,
                     "O", selectedTag);
    FTImpl.DLEnd();

    uint8_t buttonPressTag = kpt.waitForChange(selectedTag);

    Serial.print("Home screen button pressed: ");
    Serial.print(buttonPressTag);
    Serial.print(" selectedTag: ");
    Serial.println(selectedTag);

    switch(buttonPressTag) {
      case kNewExperimentTag:
      setNextScreen(DisplayScreen::kDisplayScreenNewExp);
      strcpy(currentExp.name, ProjectString);
      currentExp.irradience = kDefaultIrradience;
      currentExp.time = kDefaultTime;
      currentExp.energy = kDefaultTime * kDefaultIrradience;
      break;
    case kPrevExperimentTag:
      setNextScreen(DisplayScreen::kDisplayScreenBrowseExperiments);
      break;
      case kOptionsButtonTag:
      setNextScreen(DisplayScreen::OptionsScreen);
    }
  } while (currentScreen == DisplayScreen::kDisplayScreenHome);
}

void newExpScreen(uint16_t currentScreen) {
  NotepadResult result = Notepad(currentExp.name);
  if (kNotepadResultSave == result) {
    strncpy(currentExp.name, Buffer.notepad[0],
            min(sizeof(currentExp.name), sizeof(Buffer.notepad[0])));
    currentExp.name[sizeof(currentExp.name) - 1] = '\0';
    setNextScreen(DisplayScreen::kDisplayScreenExpSettings);
  } else {
    setNextScreen(DisplayScreen::kDisplayScreenHome);
  }
}

void drawHoldToggle(uint16_t x, uint16_t y, bool on) {
  const char *kHoldToggleText = "\xffHold";
  const uint16_t kHoldToggleWidth = 40;
  FTImpl.Cmd_Toggle(x, y, kHoldToggleWidth, kFont, 0, on ? UINT16_MAX : 0,
                    kHoldToggleText);
}

void drawSliderOrProgress(int16_t x, int16_t y, uint16_t w, uint16_t h,
                          uint16_t val, uint16_t Range, bool isProgress) {
  if (isProgress) {
    FTImpl.Cmd_Progress(x, y, w, h, 0, val, Range);
  } else {
    FTImpl.Cmd_Slider(x, y, w, h, 0, val, Range);
  }
}

void saveCurrentExp() {
  SavedExperimentsDB db(sd, "experiments.db");
  SavedExperiment exp;
  strcpy(exp.name, currentExp.name);
  for (int i = 1; i < 7; ++i) {
    exp.datetime[i] = currentExp.datetime[i];
  }
  exp.irradience = currentExp.irradience;
  exp.time = currentExp.time;
  db.addExperiment(exp);
}

void experimentSettingsScreen() {
  uint8_t currentTag = 0;
  KeyPressTracker kpt(&FTImpl);
  Serial.println("TRACE: experimentSettingsScreen()");

  int32_t &time = currentExp.time;
  int32_t &irradience = currentExp.irradience;
  int32_t &energy = currentExp.energy;

  while (DisplayScreen::kDisplayScreenExpSettings == currentScreen) {
    FTImpl.Cmd_DLStart();
    FTImpl.ClearColorRGB(255, 255, 255);
    FTImpl.Clear(1, 1, 1);
    FTImpl.ColorRGB(255, 255, 255);
    FTImpl.TagMask(1);

    const uint8_t kRunTag = 14;
    const uint8_t kBackTag = 15;
    drawBottomRightButton(kRunTag, "Run", currentTag);
    drawBottomLeftButton(kBackTag, "Back", currentTag);

    // Todo add back button, currently the global quit button is displayed

    const int timeSliderMinsTag = 19;
    const int timeSliderSecsTag = 20;
    const int irradienceSliderTag = 21;
    const int energySliderTag = 22;
    const int kTimeHoldTag = 23;
    const int kIrradienceHoldTag = 24;
    const int kEnergyHoldTag = 25;
    const int16_t sliderLeft = 40;
    const int16_t sliderWidth = 300;
    const int16_t topSliderY = 86;
    const int16_t sliderYSpacing = 50;
    // Slider definition and operation
    /* Set the tracker for 3 sliders */
    FTImpl.Cmd_Track(sliderLeft, topSliderY, sliderWidth / 2 - 20, 8,
                     timeSliderMinsTag);  // duration in minutes
    FTImpl.Cmd_Track(sliderLeft + sliderWidth / 2 + 20, topSliderY,
                     sliderWidth / 2 - 20, 8,
                     timeSliderSecsTag);  // duration in minutes
    FTImpl.Cmd_Track(sliderLeft, topSliderY + sliderYSpacing, sliderWidth, 8,
                     irradienceSliderTag);
    FTImpl.Cmd_Track(sliderLeft, topSliderY + 2 * sliderYSpacing, sliderWidth,
                     8, energySliderTag);

    const uint8_t buttonPressTag = kpt.getButtonPressTag(currentTag);
    uint32_t TrackRegisterVal = FTImpl.Read32(REG_TRACKER);
    uint8_t currentSliderTag = TrackRegisterVal & 0xFF;
    int32_t sliderTrackerVal = TrackRegisterVal >> 16;
    // Serial.print("Button press tag: "); Serial.println(buttonPressTag);
    // Serial.print("Track register tag: "); Serial.println(TrackRegisterVal &&
    // 0xFF); Serial.print("Slider val: "); Serial.println(sliderTrackerVal);

    const int32_t kMinIrradience = 5;
    const int32_t kMaxIrradience = 100;  // needs units not percentage?

    const int32_t kMaxMinutes = 30;
    const int32_t kMaxSeconds = 59;
    int32_t minutes = time / 60;
    int32_t seconds = time - minutes * 60;

    if (kHeldSliderTime != heldSlider &&
        (timeSliderMinsTag == currentSliderTag ||
         timeSliderSecsTag == currentSliderTag)) {
      if (timeSliderMinsTag == currentSliderTag) {
        minutes = kMaxMinutes * sliderTrackerVal / 65535;
      } else if (timeSliderSecsTag == currentSliderTag) {
        seconds = kMaxSeconds * sliderTrackerVal / 65535;
      }
      if (minutes == kMaxMinutes) {
        seconds = 0;
      } else if (minutes == 0) {
        seconds = max(seconds, 1);
      }
      time = 60 * minutes + seconds;
      if (kHeldSliderEnergy == heldSlider) {
        if (time == 0) {
          irradience = kMaxIrradience + 1;
        } else {
          irradience = energy / time;
        }
        if (irradience > kMaxIrradience) {
          irradience = kMaxIrradience;
          time = energy / irradience;
        } else if (irradience < 1) {
          irradience = 1;
          time = energy / irradience;
        }
      } else {
        energy = irradience * time;
      }
    } else if (kHeldSliderIrradience != heldSlider &&
               irradienceSliderTag == currentSliderTag) {
      irradience = kMaxIrradience * sliderTrackerVal / 65535;
      irradience = max(irradience, kMinIrradience);

      if (kHeldSliderEnergy == heldSlider) {
        if (irradience == 0) {
          time = 60 * kMaxMinutes + 1;
        } else {
          time = energy / irradience;
        }
        if (time > 60 * kMaxMinutes) {
          time = 60 * kMaxMinutes;
          irradience = energy / time;
        } else if (time < 1) {
          time = 1;
          irradience = energy / time;
        }
      } else {
        energy = irradience * time;
      }
    } else if (kHeldSliderEnergy != heldSlider &&
               energySliderTag == currentSliderTag) {
      energy =
          (kMaxMinutes * (60 / 15) * kMaxIrradience) * sliderTrackerVal /
          (65535 / 15);  // TODO check as max minutes and max irradience change
      energy = max(energy, 1);
      if (kHeldSliderIrradience == heldSlider) {
        if (irradience != 0) {
          time = energy / irradience;
        } else if (energy == 0) {
          time = 0;
        }
        if (time > 60 * kMaxMinutes) {
          time = 60 * kMaxMinutes;
          energy = time * irradience;
        } else if (time < 1) {
          time = 1;
          energy = time * irradience;
        }
      } else if (time != 0) {
        irradience = energy / time;
        if (irradience > kMaxIrradience) {
          irradience = kMaxIrradience;
          energy = irradience * time;
        } else if (irradience < kMinIrradience) {
          irradience = 1;
          energy = irradience * time;
        }
      }
    }
    minutes = time / 60;
    seconds = time - minutes * 60;

    switch (buttonPressTag) {
      case kIrradienceHoldTag:
        heldSlider = kHeldSliderIrradience;
        break;
      case kEnergyHoldTag:
        heldSlider = kHeldSliderEnergy;
        break;
      case kTimeHoldTag:
        heldSlider = kHeldSliderTime;
        break;
      case kRunTag:
        ReadTimeDate(currentExp.datetime);
        saveCurrentExp();
        setNextScreen(DisplayScreen::kDisplayScreenRun);
        break;
      case kBackTag:
        setNextScreen(DisplayScreen::kDisplayScreenNewExp);
        break;
      default:
        break;
    }

    const float kFullIrrandiencePowerPerArea = 60;

    const uint16_t sliderHeight = 15;

    // ColorRGB is active part of slider
    // FGColor is slider handle
    // BGColor is inactive part of slider
    FTImpl.ColorRGB(kColourSeconday);
    FTImpl.Cmd_FGColor(kColourPrimary);
    FTImpl.Cmd_BGColor(kColourLight);
    FTImpl.Tag(timeSliderMinsTag);
    drawSliderOrProgress(sliderLeft, topSliderY, sliderWidth / 2 - 20,
                         sliderHeight, minutes, kMaxMinutes,
                         heldSlider == kHeldSliderTime);
    FTImpl.Tag(timeSliderSecsTag);
    drawSliderOrProgress(sliderLeft + sliderWidth / 2 + 20, topSliderY,
                         sliderWidth / 2 - 20, sliderHeight, seconds,
                         kMaxSeconds, heldSlider == kHeldSliderTime);
    FTImpl.Tag(kTimeHoldTag);
    drawHoldToggle(sliderLeft + sliderWidth + 60, topSliderY,
                   heldSlider == kHeldSliderTime);

    // TODO do zero values make sense...?
    FTImpl.Tag(irradienceSliderTag);
    drawSliderOrProgress(sliderLeft, topSliderY + sliderYSpacing, sliderWidth,
                         sliderHeight, irradience, kMaxIrradience,
                         heldSlider == kHeldSliderIrradience);
    FTImpl.Tag(kIrradienceHoldTag);
    drawHoldToggle(sliderLeft + sliderWidth + 60, topSliderY + sliderYSpacing,
                   heldSlider == kHeldSliderIrradience);

    FTImpl.Tag(energySliderTag);
    drawSliderOrProgress(sliderLeft, topSliderY + 2 * sliderYSpacing,
                         sliderWidth, sliderHeight, energy / 60,
                         kMaxMinutes * kMaxIrradience,
                         heldSlider == kHeldSliderEnergy);
    FTImpl.Tag(kEnergyHoldTag);
    drawHoldToggle(sliderLeft + sliderWidth + 60,
                   topSliderY + 2 * sliderYSpacing,
                   heldSlider == kHeldSliderEnergy);

    FTImpl.TagMask(0);
    FTImpl.ColorRGB(0x00, 0x00, 0x00);
    showExpHeader(currentExp.name, nullptr);

    char labelBuffer[30];
    const int16_t kLabelYOffset = -18;
    const int16_t kLabelXOffset = -10;
    FTImpl.Cmd_Text(sliderLeft + sliderWidth + 60 + kLabelXOffset,
                    topSliderY + kLabelYOffset, kFont, FT_OPT_CENTERY, "Hold");
    sprintf(labelBuffer, "Duration: %2ld minutes", minutes);
    FTImpl.Cmd_Text(sliderLeft + kLabelXOffset, topSliderY + kLabelYOffset,
                    kFont, FT_OPT_CENTERY, labelBuffer);
    sprintf(labelBuffer, "%2ld seconds", seconds);
    FTImpl.Cmd_Text(sliderLeft + sliderWidth / 2 + 20 + kLabelXOffset,
                    topSliderY + kLabelYOffset, kFont, FT_OPT_CENTERY,
                    labelBuffer);

    sprintf(labelBuffer, "Irradiance: %3ld mW/cm^2", irradience);
    FTImpl.Cmd_Text(sliderLeft + kLabelXOffset,
                    topSliderY + sliderYSpacing + kLabelYOffset, kFont,
                    FT_OPT_CENTERY, labelBuffer);

    sprintf(labelBuffer, "Energy: %7ld mJ/cm^2", energy);
    FTImpl.Cmd_Text(sliderLeft + kLabelXOffset,
                    topSliderY + 2 * sliderYSpacing + kLabelYOffset, kFont,
                    FT_OPT_CENTERY, labelBuffer);
    FTImpl.DLEnd();
  }
}

// Trys to find a numbered log file that has not been used
// Returns the opened file or null on failure
File tryOpenLogfile() {
  do {
    LogRef++;  // increment logfile count and look for next free file
    LogFileName[11] = char(48 + int(LogRef / 100));
    LogFileName[12] = char(48 + int((LogRef - int(LogRef / 100) * 100) / 10));
    LogFileName[13] = char(48 + int(LogRef - int(LogRef / 10) * 10));
    Serial.println(LogFileName);
  } while (sd.exists(LogFileName));

  File logFile = sd.open(LogFileName, FILE_WRITE);
  if (!logFile) {  // Open and ensure file created o.k.
    Serial.print(F("File error"));
  }
  return logFile;
}

void startRunLog(File logFile) {
  // TODO use correct variables and units
  logFile.println("Deliberately left blank");  // Sometimes lose first
                                               // line, so make it a dummy.
  logFile.print("Test ID: ");                  // Record project data
  logFile.println(currentExp.name);
  logFile.print("Duration (s): ");
  logFile.print(currentExp.time);
  logFile.print(", ");
  logFile.print("Selected Intensity (%): ");
  logFile.print(currentExp.irradience);
  logFile.print(", ");
  logFile.print("Selected Current (%): ");
  logFile.println(Current);
  logFile.print("Calculation power density (uW/mm2): ");
  logFile.println(EnergyDensity);
  logFile.print("Energy applied (mW/cm2): ");
  logFile.println(currentExp.energy);
  logFile.print("Start Date/Time: ");
  logFile.println(ConvertTimeDate(TimeAndDate));
  logFile.println("Time (s), UV (relative), Temperature (Deg C)");
}

static void showSpinner(const char *message) {
  FTImpl.ClearColorRGB(255, 255, 255);
  FTImpl.DLStart();
  FTImpl.ColorRGB(kColourPrimary);
  if (message) {
    FTImpl.Cmd_Text(230, 80, 28, FT_OPT_CENTER, message);
  }
  FTImpl.Cmd_Spinner(FT_DISPLAY_HSIZE / 2, 150, 0, 0);

  FTImpl.Finish();
}

void checkAndWaitForLid() {
  if (digitalRead(LID)) {
    showSpinner("Close Lid");
    Serial.println("Close Lid");
    do {
      delay(100);
    } while (digitalRead(LID));
  }
}

void runScreen(uint8_t currentTag) {
  showSpinner(nullptr);

  Serial.println(SetCurrent);
  ReadTimeDate(TimeAndDate);
  Serial.println(ConvertTimeDate(TimeAndDate));

  File LogFile2 = tryOpenLogfile();
  // TODO error handling for logfile
  startRunLog(LogFile2);

  checkAndWaitForLid();  // Make sure lid is closed before start
  analogWrite(ADM, 0);
  analogWrite(PWM, 5);  // LEDs power setting - see Mk6 - starting with
                        // PWM = 0 causes LED's to fail.
  delay(100);
  digitalWrite(FAN, HIGH);
  delay(1000);  // to make sure current setting voltage R-C circuit is
                // charged
  analogWrite(ADM, SetCurrent);  // turn on LEDs (slowly)

  showSpinner("Starting, please wait....");

  delay(1000);  // TODO return to 10000
  analogWrite(PWM,
              int((float)currentExp.irradience * 2.55));  // LEDs power setting

  msTime = millis();

  OldiTime = -1;
  LidOpen = false;
  uvPrintVal[0] = '\0';

  uint8_t currentPressedTag = 0;
  KeyPressTracker kpt(&FTImpl);
  do {
    if (digitalRead(LID) && !LidOpen)  // lid just opened
    {
      LidOpen = true;
      msTimeLid =
          millis();  // need to keep a record of how long lid is open for
      analogWrite(PWM, 0);
      // Serial.println("Close Lid");
      // analogWrite(ADM,0);
      // analogWrite(PWM,int((float)Intensity*2.55));              //reset
      // start case
    }

    if (LidOpen && !digitalRead(LID))  // lid was open, but now closed.
    {
      analogWrite(PWM, int((float)currentExp.irradience * 2.55));
      msTime = msTime + (millis() - msTimeLid);  // correct the time
      LidOpen = false;
    }
    if (!LidOpen) iTime = currentExp.time - int((millis() - msTime) / 1000);

    float Temperature;
    if (OldiTime != iTime)  // into a new second, so save results
    {
      OldiTime = iTime;
      Serial.print("Time = ");
      Serial.print(currentExp.time - iTime);
      LogFile2.print(iTime);
      Serial.print(", uv = ");
      LogFile2.print(", ");
      uvValue = uv.readUV();
      sprintf(uvPrintVal, "%05i",
              uvValue);  //%0 left pads the number with zeros, 5 = width,
                         // i = signed decimal integer
      for (int8_t i = 0; i < 5; i++) {
        uvPrint[i + 5] = uvPrintVal[i];
      }
      Serial.print(uvValue);
      LogFile2.print(uvValue);
      Serial.print(", t = ");
      LogFile2.print(", ");
      int16_t i = 0;
      do  // fiddle to sort out SPI clash
      {
        i++;
        SPI.end();
      } while ((SPCR & 64) == 64);
      // Serial.println(i);
      delay(10);
      sensors.requestTemperaturesByAddress(
          DallasAddress);  // can take up to 750ms to get temperature?
      Temperature = sensors.getTempC(DallasAddress);
      if (Temperature != -127) {
        Serial.println(Temperature);
      } else {
        Serial.println("Read Error");
      }
      LogFile2.println(Temperature);
      // You can have more than one IC on the same bus.
      // 0 refers to the first IC on the wire
      do {
        i--;
        SPI.begin();
      } while (i > 0);
      Serial.println(iTime);
    }

    FTImpl.Cmd_DLStart();
    FTImpl.ClearColorRGB(255, 255, 255);
    FTImpl.Clear(1, 1, 1);

    showExpHeader(currentExp.name, currentExp.datetime);
    const uint8_t kAbortButtonTag = 13;
    drawBottomMiddleButton(kAbortButtonTag, "Abort", currentPressedTag);

    FTImpl.ColorRGB(0x000000);

    const uint16_t leftColumnStart = 2 * kBorderPixels;

    char labelBuffer[12] = {'\0'};
    FTImpl.Cmd_Text(leftColumnStart, 60, 31, 0, "Time remaining:");
    sprintf_P(labelBuffer, PSTR("%d:%02d"), iTime / 60, iTime % 60);
    FTImpl.Cmd_Text(FT_DISPLAYWIDTH - leftColumnStart, 60, 31, FT_OPT_RIGHTX,
                    labelBuffer);

    if (LidOpen) {
      FTImpl.Cmd_Text(FT_DISPLAYWIDTH / 2, 120, 31, FT_OPT_CENTER, "Close Lid");
    } else {
      const uint8_t settingsFont = 27;
      const uint16_t leftColumnValEnd = 2 * kBorderPixels + 256;

      sprintf(labelBuffer, "%" PRId32 ":%02" PRId32, currentExp.time / 60,
              currentExp.time % 60);
      FTImpl.Cmd_Text(leftColumnStart, 120, settingsFont, 0, "Duration:");
      FTImpl.Cmd_Text(leftColumnValEnd, 120, settingsFont, FT_OPT_RIGHTX,
                      labelBuffer);
      sprintf(labelBuffer, "%" PRId32 " mW/mm", currentExp.irradience);
      FTImpl.Cmd_Text(leftColumnStart, 150, settingsFont, 0, "Intensity:");
      const uint8_t superscriptCharWidth = Ft_Gpu_Rom_Font_WH('2', 26);
      const uint16_t leftColValEndSS =
          leftColumnValEnd - 1 - superscriptCharWidth;
      FTImpl.Cmd_Text(leftColValEndSS, 150, settingsFont, FT_OPT_RIGHTX,
                      labelBuffer);
      FTImpl.Cmd_Text(leftColValEndSS + 1, 150 - 3, 26, 0, "2");
      //      sprintf(OutputValue, "%03" PRId32, Current);s
      //      FTImpl.Cmd_Text(300, 150, 28, FT_OPT_CENTERX, "Current (%):");
      //      FTImpl.Cmd_Text(450, 150, 28, FT_OPT_RIGHTX, OutputValue);
      sprintf(labelBuffer, "%" PRId32 " mJ/mm", currentExp.energy);
      FTImpl.Cmd_Text(leftColumnStart, 150, settingsFont, 0, "");
      FTImpl.Cmd_Text(leftColumnStart, 180, settingsFont, 0, "Energy density:");
      FTImpl.Cmd_Text(leftColValEndSS, 180, settingsFont, FT_OPT_RIGHTX,
                      labelBuffer);
      FTImpl.Cmd_Text(leftColValEndSS + 1, 180 - 3, 26, 0, "2");

      FTImpl.Cmd_Text(310, 160, 29, 0, "Temp:");
      Temperature = 25.1;
      if (Temperature != -127) {
        sprintf_P(labelBuffer, PSTR("%d.%01d C"), int(Temperature),
                  int(Temperature * 10) % 10);
      } else {
        strcpy_P(labelBuffer, PSTR("Error"));
      }
      FTImpl.Cmd_Text(FT_DISPLAYWIDTH - leftColumnStart, 160, 29, FT_OPT_RIGHTX,
                      labelBuffer);

      FTImpl.Cmd_Text(340, 120, 29, 0, "UV:");
      sprintf_P(labelBuffer, PSTR("%d"), uvValue);
      FTImpl.Cmd_Text(FT_DISPLAYWIDTH - leftColumnStart, 120, 29, FT_OPT_RIGHTX,
                      labelBuffer);
    }
    FTImpl.Display();
    FTImpl.Cmd_Swap();
    FTImpl.Finish();

    const uint8_t buttonPressTag = kpt.getButtonPressTag(currentPressedTag);
    if (kAbortButtonTag == buttonPressTag) {
      Serial.println("Abort hit");
      LogFile2.println("Abort hit");
      LidOpen = false;
      goto EscapeNestedLoops;
    }
  } while (iTime > 0);  // end of do loop

  analogWrite(PWM, 0);

EscapeNestedLoops:
  Serial.println(iTime);
  digitalWrite(FAN, LOW);
  analogWrite(ADM, 0);
  Serial.print("End Date/Time: ");
  LogFile2.print("End Date/Time: ");
  ReadTimeDate(TimeAndDate);
  LogFile2.println(ConvertTimeDate(TimeAndDate));
  Serial.println(ConvertTimeDate(TimeAndDate));
  LogFile2.close();
  setNextScreen(DisplayScreen::kDisplayScreenHome);
}

// Font is 26;
static String getWidthLimitedExpName(const char *name, int16_t width,
                                     uint8_t font) {
  int16_t nameWidth = stringPixelWidth(name, font);
  Serial.println("Name width: ");
  Serial.println(nameWidth);
  String str(name);
  if (nameWidth > width) {
    uint32_t fontTableAddr = getFontTableStartAddr();
    const char *elipsis = "...";
    const int16_t elipsisWidth = stringPixelWidth(elipsis, font);
    int16_t idx = str.length() - 1;
    for (; idx >= 0; --idx) {
      nameWidth -= getCharWidth(fontTableAddr, str[idx], font);
      if (nameWidth + elipsisWidth < width) {
        break;
      }
    }
    str = str.substring(0, idx) + elipsis;
  }
  return str;
}

// Needs topDisplayedExperiment global to return to correct point in the
// experiments list from review prev experiment page
void browseExperimentsScreen(uint8_t ignore) {
  Serial.println(PSTR("TRACE: browseExperimentesScreen"));

  const int experimentsPerScreen = 7;
  SavedExperiment browseExperiments[experimentsPerScreen];
  int16_t browseExperimentsIdx[experimentsPerScreen];
  SavedExperimentsDB db(sd, "experiments.db");

  // db.clear();
  if (db.count() < 102) {
    for (int i = db.count(); i <= 102; ++i) {
      SavedExperiment s;
      String name = "Test ";
      name.concat("-----");
      name.concat("--------------------");
      name.concat("-------");
      name.concat(i);
      strcpy(s.name, name.c_str());
      s.datetime[0] = 0;
      s.datetime[1] = i % 60;
      s.datetime[2] = i / 10;
      s.datetime[3] = 0;
      s.datetime[4] = 12;
      s.datetime[5] = 11;
      s.datetime[6] = 2019;
      s.irradience = max(i, 5);
      s.time = 100 * i;
      db.addExperiment(s);
    }
  }

  int16_t numSavedExperiments = db.count();
  int16_t topDisplayedExperiment = browseExperimentsStartExpIdx < 1
                                       ? numSavedExperiments
                                       : browseExperimentsStartExpIdx;
  int16_t lastTopDisplayedExperiment = -1;
  int32_t experimentsToDisplay = 0;
  uint8_t currentPressedTag = 0;
  KeyPressTracker kpt(&FTImpl);
  while (true) {
    if (topDisplayedExperiment != lastTopDisplayedExperiment) {
      experimentsToDisplay = min(experimentsPerScreen, topDisplayedExperiment);
      Serial.print("Loading experiments ");
      Serial.println(millis());
      for (int i = 0; i < experimentsToDisplay; ++i) {
        int expDisplayIdx = experimentsToDisplay - 1 - i;
        int16_t expIdx = topDisplayedExperiment - experimentsToDisplay + i + 1;
        db.getExperiment(expIdx, browseExperiments[expDisplayIdx]);
        browseExperimentsIdx[expDisplayIdx] = expIdx;
      }
      Serial.print("Loaded experiments ");
      Serial.println(millis());
      lastTopDisplayedExperiment = topDisplayedExperiment;
    }

    // Render the current state
    const uint8_t kNextPageTag = experimentsPerScreen + 10;
    const uint8_t kPrevPageTag = experimentsPerScreen + 11;
    const uint8_t kBackButtonTag = experimentsPerScreen + 12;

    FTImpl.Cmd_DLStart();
    FTImpl.ClearColorRGB(255, 255, 255);
    FTImpl.Clear(1, 1, 1);
    FTImpl.ColorRGB(0, 0, 0);
    FTImpl.TagMask(1);
    for (int i = 0; i < experimentsToDisplay; ++i) {
      FTImpl.Tag(i + 1);  // minimum tag value is 1
      if (currentPressedTag == i + 1) {
        FTImpl.Cmd_FGColor(0xDDDDDD);
      } else {
        FTImpl.Cmd_FGColor(0xFFFFFF);
      }
      const int kRowButtonHeight = 28;
      const int kRowSpacing = 28;
      FTImpl.Cmd_Button(10, 20 + i * kRowSpacing, FT_DISPLAYWIDTH - 20, 26,
                        kFont, FT_OPT_FLAT, "");

      // date string is 135 width in font 26, leaving 480 - 2*20 -135 - 10 px
      // seperation for exp name
      const String displayExpName = getWidthLimitedExpName(
          browseExperiments[i].name, 480 - 2 * 20 - 135 - 10, kFont);
      FTImpl.Cmd_Text(20, 20 + kRowButtonHeight / 2 + i * kRowSpacing, kFont,
                      FT_OPT_CENTERY, displayExpName.c_str());
      String datetime = ConvertTimeDate(browseExperiments[i].datetime);
      FTImpl.Cmd_Text(FT_DISPLAYWIDTH - 20,
                      20 + kRowButtonHeight / 2 + i * kRowSpacing, kFont,
                      FT_OPT_CENTERY | FT_OPT_RIGHTX, datetime.c_str());
      // Serial.print("Date width: ");
      // Serial.println(stringPixelWidth(datetime.c_str(), kFont));
    }
    FTImpl.Cmd_FGColor(kColourPrimary);
    FTImpl.ColorRGB(0xFF, 0xFF, 0xFF);

    if (topDisplayedExperiment - experimentsPerScreen > 0) {
      FTImpl.Tag(kNextPageTag);
      uint16_t options = currentPressedTag == kNextPageTag ? FT_OPT_FLAT : 0;
      FTImpl.Cmd_Button(423 - 47, 241 - 19, 94, 38, 26, options, "Next");
    }
    if (topDisplayedExperiment != numSavedExperiments) {
      FTImpl.Tag(kPrevPageTag);
      uint16_t options = currentPressedTag == kPrevPageTag ? FT_OPT_FLAT : 0;
      FTImpl.Cmd_Button(423 - 47 - 94 - 40, 241 - 19, 94, 38, 26, options,
                        "Previous");
    }
    drawBottomLeftButton(kBackButtonTag, "Back", currentPressedTag);
    FTImpl.DLEnd();

    uint8_t buttonPressTag = kpt.waitForChange(currentPressedTag);
    Serial.print("button pressed: ");
    Serial.print(buttonPressTag);
    Serial.print(" currentPressedTag: ");
    Serial.println(currentPressedTag);

    if (kBackButtonTag == buttonPressTag) {
      setNextScreen(DisplayScreen::kDisplayScreenHome);
      break;
    } else if (kNextPageTag == buttonPressTag) {
      if (topDisplayedExperiment - experimentsPerScreen > 0) {
        topDisplayedExperiment -= experimentsPerScreen;
      }
    } else if (kPrevPageTag == buttonPressTag) {
      if (topDisplayedExperiment + experimentsPerScreen <=
          numSavedExperiments) {
        topDisplayedExperiment += experimentsPerScreen;
      }
    } else if (0 < buttonPressTag && buttonPressTag <= experimentsToDisplay) {
      Serial.println("Setting next screen kDisplayScreenShowSavedExp");
      setNextScreen(DisplayScreen::kDisplayScreenShowSavedExp);
      const uint8_t selectedExp = buttonPressTag - 1;
      strcpy(currentExp.name, browseExperiments[selectedExp].name);
      for (int i = 0; i < 7; ++i) {
        currentExp.datetime[i] = browseExperiments[selectedExp].datetime[i];
      }
      currentExp.irradience = browseExperiments[selectedExp].irradience;
      currentExp.time = browseExperiments[selectedExp].time;
      currentExp.energy = currentExp.irradience * currentExp.time;
      // Set some magic global variable to the Saved Exp state
      browseExperimentsStartExpIdx = lastTopDisplayedExperiment;
      break;
    }
  };
}

void savedExperimentScreen() {
  Serial.println("TRACE: savedExperimentScreen");

  char labelBuffer[30];
  uint8_t currentPressedTag = 0;
  uint8_t buttonPressTag = 0;
  KeyPressTracker kpt(&FTImpl);

  do {
    FTImpl.Cmd_DLStart();
    FTImpl.ClearColorRGB(0xFFFFFF);
    FTImpl.Clear(1, 1, 1);
    showExpHeader(currentExp.name, currentExp.datetime);
    const int kSpacing = 40;
    const int kLeftColumnX = 30;
    const int kRightColumnX = 200;
    FTImpl.ColorRGB(0x00, 0x00, 0x00);
    FTImpl.Cmd_Text(kLeftColumnX, 60, kFont, 0, "Duration:");
    sprintf(labelBuffer, "%" PRId32 ":%02" PRId32, currentExp.time / 60,
            currentExp.time % 60);
    FTImpl.Cmd_Text(kRightColumnX, 60, kFont, 0, labelBuffer);
    FTImpl.Cmd_Text(kLeftColumnX, 60 + kSpacing, kFont, 0, "Irradience:");
    sprintf(labelBuffer, "%ld %%", currentExp.irradience);
    FTImpl.Cmd_Text(kRightColumnX, 60 + kSpacing, kFont, 0, labelBuffer);
    FTImpl.Cmd_Text(kLeftColumnX, 60 + 2 * kSpacing, kFont, 0,
                    "Energy Density:");
    sprintf(labelBuffer, "%ld mJ/mm2", currentExp.energy);
    FTImpl.Cmd_Text(kRightColumnX, 60 + 2 * kSpacing, kFont, 0, labelBuffer);

    const uint8_t kBackButtonTag = 10;
    const uint8_t kModifyButtonTag = 11;
    const uint8_t kRunButtonTag = 12;
    FTImpl.TagMask(1);
    FTImpl.ColorRGB(0xFF, 0xFF, 0xFF);
    drawBottomLeftButton(kBackButtonTag, "Back", currentPressedTag);
    drawBottomMiddleButton(kModifyButtonTag, "Modify", currentPressedTag);
    drawBottomRightButton(kRunButtonTag, "Run", currentPressedTag);
    FTImpl.DLEnd();

    do {
      buttonPressTag = kpt.getButtonPressTag(currentPressedTag);
    } while (buttonPressTag == 0 && currentPressedTag == 0);
    if (kBackButtonTag == buttonPressTag) {
      setNextScreen(DisplayScreen::kDisplayScreenBrowseExperiments);
    } else if (kModifyButtonTag == buttonPressTag) {
      setNextScreen(
          DisplayScreen::kDisplayScreenNewExp);  // TODO Need to keep the
                                                 // current experiment settings
                                                 // and pass in the name
    } else if (kRunButtonTag == buttonPressTag) {
      ReadTimeDate(currentExp.datetime);
      setNextScreen(DisplayScreen::kDisplayScreenRun);
    }
  } while (buttonPressTag == 0);
}

void optionScreen() {
  constexpr uint8_t kBackButtonTag = 13;
  constexpr uint8_t kSetDateButtonTag = 15;
  constexpr uint8_t kExportLogsButtonTag = 16;
  constexpr uint8_t kSettingsButtonTag = 17;
  constexpr uint8_t kAboutButtonTag = 18;

  uint8_t currentTag = 0;
  KeyPressTracker kpt(&FTImpl);
  bool done = false;
  do {
    FTImpl.Cmd_DLStart();
    FTImpl.ClearColorRGB(0xFFFFFF);
    FTImpl.Clear(1, 1, 1);
    FTImpl.TagMask(1);
    drawBottomLeftButton(kBackButtonTag, "Back", currentTag);
    drawTaggedButton(kSetDateButtonTag, 17, 54, 431, 33, "Set date and time",
                     currentTag);
    drawTaggedButton(kExportLogsButtonTag, 17, 10, 431, 33,
                     "Copy logs to flash drive", currentTag);
    drawTaggedButton(kSettingsButtonTag, 17, 98, 431, 33,
                     "Settings (Administrator password required)", currentTag);
    drawTaggedButton(kAboutButtonTag, 17, 142, 431, 33, "About", currentTag);
    FTImpl.DLEnd();

    const uint8_t buttonPressTag = kpt.waitForChange(currentTag);
    switch (buttonPressTag) {
      case kBackButtonTag:
        setNextScreen(DisplayScreen::kDisplayScreenHome);
        done = true;
        break;
      case kSetDateButtonTag:
        Screen = 2;
        done = true;
        break;
      case kExportLogsButtonTag:
        Screen = 9;
        done = true;
        break;
      case kSettingsButtonTag:
        setNextScreen(DisplayScreen::kDisplayScreenHome);  // TODO Fixme
        done = true;
        break;
      case kAboutButtonTag:
        Screen = 10;
        done = true;
        break;
    }
  } while (!done);
}

void loop() {
  // newscreen
  sTagXY sTagxy;
  int32_t tagval, tagoption = 0, datetag = 25;
  // for sliders
  uint32_t TrackRegisterVal = 0;
  int32_t tmpNewEnergyDensity,
      tmpNewCurrent = 100;  //, tmpintensity = 100
  char strNewEnergyDensity[15];
  char TmpDate[3];
  // end sliders

  File LogFile2;
  float Float;
  char sdlogbuffer[80], sdlog;
  int logpointer = 0, logcopycount = 0;
  char FlashFile[27] = {'$', 'W', 'R', 'I', 'T', 'E', ' ', 'L', 'O', 'G',
                        '0', '0', '0', '0', '0', '.', 'C', 'S', 'V'};
  bool DateRead = false;

  Screen = 0;  // 5;
  setNextScreen(DisplayScreen::kDisplayScreenHome);

  while (1) {
    FTImpl.TagMask(1);
    FTImpl.GetTagXY(sTagxy);
    tagval = sTagxy.tag;
    FTImpl.ClearColorRGB(64, 64, 64);  // text colour?

    if (DisplayScreen::kDisplayScreenHome == currentScreen) {
      homeScreen();
      tagval = 0;  // TODO remove, avoids below code detecting a press
    } else if (DisplayScreen::kDisplayScreenNewExp == currentScreen) {
      newExpScreen(tagval);
      tagval = 0;  // TODO remove, avoids below code detecting a press
    } else if (DisplayScreen::kDisplayScreenExpSettings == currentScreen) {
      experimentSettingsScreen();
      tagval = 0;  // TODO remove
    } else if (DisplayScreen::kDisplayScreenRun == currentScreen) {
      runScreen(tagval);
      tagval = 0;  // TODO remove
    } else if (DisplayScreen::kDisplayScreenBrowseExperiments ==
               currentScreen) {
      browseExperimentsScreen(tagval);
      tagval = 0;
    } else if (DisplayScreen::kDisplayScreenShowSavedExp == currentScreen) {
      savedExperimentScreen();
      tagval = 0;
    }
    else if (DisplayScreen::OptionsScreen == currentScreen) {
      optionScreen();
      tagval = 0;
    } else {
      FTImpl.DLStart();
      if (Screen !=
          0)  // On all other screen bottom left button is
              // quit///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      {
        // assign tag value 13 to the quit button
        tagoption =
            0;  // no touch is default 3d effect and touch is flat effect
        if (13 == tagval) tagoption = FT_OPT_FLAT;
        FTImpl.Tag(13);
        FTImpl.Cmd_Button(63 - 47, 241 - 19, 94, 38, 26, tagoption, "Quit");
      }

      if (Screen ==
          1)  // Options///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      {
        tagoption =
            0;  // no touch is default 3d effect and touch is flat effect
        if (15 == tagval) tagoption = FT_OPT_FLAT;
        FTImpl.Tag(15);
        FTImpl.Cmd_Button(17, 54, 431, 33, 26, tagoption, "Set date and time");

        tagoption =
            0;  // no touch is default 3d effect and touch is flat effect
        if (16 == tagval) tagoption = FT_OPT_FLAT;
        FTImpl.Tag(16);
        FTImpl.Cmd_Button(17, 10, 431, 33, 26, tagoption,
                          "Copy logs to flash drive");

        tagoption =
            0;  // no touch is default 3d effect and touch is flat effect
        if (18 == tagval) tagoption = FT_OPT_FLAT;
        FTImpl.Tag(18);
        FTImpl.Cmd_Button(17, 98, 431, 33, 26, tagoption,
                          "Settings (Administrator password required)");

        tagoption =
            0;  // no touch is default 3d effect and touch is flat effect
        if (17 == tagval) tagoption = FT_OPT_FLAT;
        FTImpl.Tag(17);
        FTImpl.Cmd_Button(17, 142, 431, 33, 26, tagoption,
                          "Product information");
      }

      if (Screen ==
          2)  // Date///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      {
        tagoption =
            0;  // no touch is default 3d effect and touch is flat effect
        if (14 == tagval) tagoption = FT_OPT_FLAT;
        FTImpl.Tag(14);
        FTImpl.Cmd_Button(423 - 47, 241 - 19, 94, 38, 26, tagoption, "Save");
        if (!DateRead) {
          ReadTimeDate(TimeAndDate);
          DateRead = true;
        }
        if ((25 <= tagval) && (tagval <= 30)) {
          datetag = tagval;  // tagoption = FT_OPT_FLAT;
        }
        FTImpl.Tag(25);
        TmpDate[0] = char(48 + int(TimeAndDate[4] / 10));
        TmpDate[1] = char(48 + TimeAndDate[4] - 10 * int(TimeAndDate[4] / 10));
        TmpDate[2] = '\0';
        FTImpl.Cmd_Button(40, 40, 60, 38, 30, (datetag == 25) ? FT_OPT_FLAT : 0,
                          TmpDate);  // Day
        FTImpl.Tag(26);
        TmpDate[0] = char(48 + int(TimeAndDate[5] / 10));
        TmpDate[1] = char(48 + TimeAndDate[5] - 10 * int(TimeAndDate[5] / 10));
        FTImpl.Cmd_Button(110, 40, 60, 38, 30,
                          (datetag == 26) ? FT_OPT_FLAT : 0,
                          TmpDate);  // Month
        FTImpl.Tag(27);
        TmpDate[0] = char(48 + int(TimeAndDate[6] / 10));
        TmpDate[1] = char(48 + TimeAndDate[6] - 10 * int(TimeAndDate[6] / 10));
        FTImpl.Cmd_Button(180, 40, 60, 38, 30,
                          (datetag == 27) ? FT_OPT_FLAT : 0,
                          TmpDate);  // Year
        FTImpl.Tag(28);
        TmpDate[0] = char(48 + int(TimeAndDate[2] / 10));
        TmpDate[1] = char(48 + TimeAndDate[2] - 10 * int(TimeAndDate[2] / 10));
        FTImpl.Cmd_Button(250, 40, 60, 38, 30,
                          (datetag == 28) ? FT_OPT_FLAT : 0,
                          TmpDate);  // Hour
        FTImpl.Tag(29);
        TmpDate[0] = char(48 + int(TimeAndDate[1] / 10));
        TmpDate[1] = char(48 + TimeAndDate[1] - 10 * int(TimeAndDate[1] / 10));
        FTImpl.Cmd_Button(320, 40, 60, 38, 30,
                          (datetag == 29) ? FT_OPT_FLAT : 0,
                          TmpDate);  // Minute
        FTImpl.Tag(30);
        TmpDate[0] = char(48 + int(TimeAndDate[0] / 10));
        TmpDate[1] = char(48 + TimeAndDate[0] - 10 * int(TimeAndDate[0] / 10));
        FTImpl.Cmd_Button(390, 40, 60, 38, 30,
                          (datetag == 30) ? FT_OPT_FLAT : 0,
                          TmpDate);  // Second
        // Up/down buttons
        tagoption =
            0;  // no touch is default 3d effect and touch is flat effect
        if (31 == tagval) tagoption = FT_OPT_FLAT;
        FTImpl.Tag(31);
        FTImpl.Cmd_Button(185, 120, 60, 38, 31, tagoption, "^");
        tagoption =
            0;  // no touch is default 3d effect and touch is flat effect
        if (32 == tagval) tagoption = FT_OPT_FLAT;
        FTImpl.Tag(32);
        FTImpl.Cmd_Button(255, 120, 60, 38, 30, tagoption, "v");
        // FTImpl.TagMask(0);
        // FTImpl.ColorRGB(0xff,0xff,0xff);
        // FTImpl.Cmd_Text(60, 20, 26, FT_OPT_CENTER, "Duration");
        FTImpl.Cmd_Text(250, 25, 16, FT_OPT_CENTER,
                        "Day    Month     Year     Hour   Minute  Second");

        if (tagval == 31) {
          TimeDigit = datetag - 25;
          TimeAndDate[TimePointer[TimeDigit]] += 1;
          if (TimeAndDate[TimePointer[TimeDigit]] > TimeMax[TimeDigit])
            TimeAndDate[TimePointer[TimeDigit]] = TimeMin[TimeDigit];
          delay(200);
        }

        if (tagval == 32) {
          TimeDigit = datetag - 25;
          TimeAndDate[TimePointer[TimeDigit]] -= 1;
          if (TimeAndDate[TimePointer[TimeDigit]] < TimeMin[TimeDigit])
            TimeAndDate[TimePointer[TimeDigit]] = TimeMax[TimeDigit];
          delay(200);
        }
      }

      if (Screen ==
          5)  // Current & power
              // density///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      {
        // assign tag value 14 to the save button
        tagoption =
            0;  // no touch is default 3d effect and touch is flat effect
        if (14 == tagval) tagoption = FT_OPT_FLAT;
        FTImpl.Tag(14);
        FTImpl.Cmd_Button(423 - 47, 241 - 19, 94, 38, 26, tagoption, "Save");

        // Slider definition and operation
        /* Set the tracker for 2 sliders */
        FTImpl.Cmd_Track(40, 100, 400, 8, 19);
        FTImpl.Cmd_Track(40, 200, 400, 8, 20);

        tagval = 0;
        TrackRegisterVal = FTImpl.Read32(REG_TRACKER);
        tagval = TrackRegisterVal & 0xff;
        if (0 != tagval) {
          if (19 == tagval) {
            NewCurrent = TrackRegisterVal >> 16;  // Value is 0 - 65535
          }
          if (20 == tagval) {
            NewEnergyDensity = TrackRegisterVal >> 16;  // Value is 0 - 65535
          }
        }

        // Float = (float)NewCurrent*100/65535;
        tmpNewCurrent =
            (float)NewCurrent * 100 / 65535 + 0.5;  // value is 0 - 100%
        Float = (float)NewEnergyDensity * 300 / 65535 +
                100;  // Value is 100 - 400 uW/mm2
        tmpNewEnergyDensity = Float;

        /* Draw slider with 3d effect */
        FTImpl.ColorRGB(0, 255, 0);
        if (NC != NewCurrent) FTImpl.ColorRGB(255, 0, 0);
        // FTImpl.Cmd_FGColor(0x00a000);
        FTImpl.Cmd_BGColor(0x000000);
        FTImpl.Tag(19);
        FTImpl.Cmd_Slider(40, 40, 400, 20, 0, NewCurrent, 65535);
        FTImpl.ColorRGB(0, 255, 0);
        if (NED != NewEnergyDensity) FTImpl.ColorRGB(255, 0, 0);
        FTImpl.Tag(20);
        FTImpl.Cmd_Slider(40, 100, 400, 20, 0, NewEnergyDensity, 65535);

        FTImpl.TagMask(0);
        FTImpl.ColorRGB(0xff, 0xff, 0xff);
        FTImpl.Cmd_Text(60, 20, 26, FT_OPT_CENTER, "Current");
        // strNewCurrent[0] = '\0';
        // Dec2Ascii(strNewCurrent,tmpNewCurrent);
        // strcat(strNewCurrent,"%");
        sprintf(OutputValue, "%03" PRId32, tmpNewCurrent);
        OutputValue[3] = '%';
        OutputValue[4] = '\0';
        // FTImpl.Cmd_Text(200, 20, 26, FT_OPT_CENTER, strNewCurrent);
        FTImpl.Cmd_Text(200, 20, 26, FT_OPT_CENTER, OutputValue);
        FTImpl.Cmd_Text(60, 80, 26, FT_OPT_CENTER, "Energy Density");
        strNewEnergyDensity[0] = '\0';
        Dec2Ascii(strNewEnergyDensity, tmpNewEnergyDensity);
        strcat(strNewEnergyDensity, "uW/mm2");
        FTImpl.Cmd_Text(200, 80, 26, FT_OPT_CENTER, strNewEnergyDensity);
        // FTImpl.DLEnd();
        // FTImpl.Finish();
        delay(10);
      }

      if (Screen == 10)  // Info
      {
        FTImpl.Cmd_Text(230, 20, 28, FT_OPT_CENTER, "PRODUCT INFORMATION PAGE");
        FTImpl.Cmd_Text(230, 60, 26, FT_OPT_CENTER,
                        "405nm Luminus SST-10-UV LEDs");
        FTImpl.Cmd_Text(230, 90, 26, FT_OPT_CENTER, "ST LED6001 controller");
        FTImpl.Cmd_Text(230, 120, 26, FT_OPT_CENTER,
                        "Drive current: 100% = 0.5A, 0% = OFF");
        FTImpl.Cmd_Text(230, 150, 26, FT_OPT_CENTER, "Software version 8.0");
        FTImpl.Cmd_Text(230, 200, 26, FT_OPT_CENTER,
                        "Designed & built by BNC for Lightox");
      }

      if (sd_present) {
        FTImpl.Cmd_Text(40, FT_DISPLAYHEIGHT / 3, 28, FT_OPT_CENTERY,
                        "Storage Device not Found");
      }

      FTImpl.DLEnd();
    }

    // End of display
    // plotting/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Plot requests below here will nt get implemented.
    if (currentScreen == DisplayScreen::kDisplayScreenNone) {
      if (tagval == 12)  // Options
      {
        Screen = 1;
      }

      if (tagval == 13)  // quit
      {
        setNextScreen(DisplayScreen::kDisplayScreenHome);
      }

      if (tagval == 14)  // Run / Save.............run option not coded yet
      {
        FTImpl.Cmd_DLStart();
        FTImpl.ClearColorRGB(64, 64, 64);
        FTImpl.Clear(1, 1, 1);
        FTImpl.ColorRGB(0xff, 0xff, 0xff);
        FTImpl.Display();
        FTImpl.Cmd_Swap();
        FTImpl.Finish();

        if (Screen == 5)  //=More
        {
          Float = (float)NewEnergyDensity / 65535 * 300 + 100;
          EnergyDensity = Float;
          EEPROM.put(eeAddress, EnergyDensity);
          // Float = (float)NewCurrent/65535*100;
          // Current = Float;
          Current = tmpNewCurrent;
          SetCurrent = (0.26 + 0.94 * (float)Current / 100.0) / 5.0 * 255;
          Serial.print("Current set to: ");
          Serial.println(SetCurrent);
          FirstPass = true;
          tagval = 18;
        } else if (Screen == 4)  //= TIE
        {
          Serial.println("Run");
          Screen = 6;
        } else if (Screen == 2)  //=Date
        {
          Serial.println("Save date");
          SetTimeDate(TimeAndDate[TimePointer[0]], TimeAndDate[TimePointer[1]],
                      TimeAndDate[TimePointer[2]], TimeAndDate[TimePointer[3]],
                      TimeAndDate[TimePointer[4]], TimeAndDate[TimePointer[5]]);
          DateRead = false;
          setNextScreen(DisplayScreen::kDisplayScreenHome);
        }
      }

      if (tagval == 15)  // Set date and time
      {
        Screen = 2;
      }

      if (tagval == 16)  // Copy files
      {
        Screen = 9;
        LogRef = 0;
        logcopycount = 0;

        // do
        //{
        //  if(mySerial.available()>0) checkfileexist[0] = mySerial.read();
        //}while(!mySerial.available());  //make sure the buffer is empty

        // sprintf(checkfileexist,"$size %s line\r","LOG00001.CSV");
        // mySerial.write(checkfileexist);
        // i = 0;
        // do
        //{
        //  i++;
        //}while((!mySerial.available()) && (i < 32000));        // Wait for
        // data
        // to be returned

        if (1 == 2) {
          Serial.println("File exists");
          FTImpl
              .Cmd_DLStart();  // start new display list - ends with
                               // DL_swap///////////////////////////////////////////////////////////////////////////////////////
          FTImpl.Begin(FT_BITMAPS);      // Start a new graphics primitive
          FTImpl.Vertex2ii(0, 0, 0, 0);  // Draw primitive
          FTImpl.BitmapHandle(0);
          FTImpl.BitmapSource(0);
          FTImpl.BitmapLayout(FT_RGB565, 480L * 2, 272);
          FTImpl.BitmapSize(FT_NEAREST, FT_BORDER, FT_BORDER, 480, 272);
          FTImpl.ColorRGB(0xff, 0xff, 0xff);
          FTImpl.Cmd_Text(230, 80, 31, FT_OPT_CENTER, "Flash drive not empty");
          FTImpl.Display();
          FTImpl.Cmd_Swap();
          FTImpl.Finish();
          delay(5000);
        } else {
          do {
            LogRef++;
            LogFileName[11] =
                char(48 + int(LogRef / 100));  // can cope with up to 999 files
            LogFileName[12] =
                char(48 + int((LogRef - int(LogRef / 100) * 100) / 10));
            LogFileName[13] = char(48 + int(LogRef - int(LogRef / 10) * 10));
            Serial.println(LogFileName);
            if (sd.exists(LogFileName)) {
              // copy file
              if (!(LogFile2 = sd.open(LogFileName))) {
                Serial.print(F("Log file not found"));
              } else {
                if (!digitalRead(
                        FLASH_IN))  // make sure flash drive is inserted
                {
                  Serial.print(F("No flash drive found"));
                } else {
                  for (int8_t i = 11; i < 14; i++)
                    FlashFile[i + 1] = LogFileName[i];
                  flash_data(FlashFile, true);  // open file for write
                  delay(1000);  // Needs extra time to create the file.
                                // Depends on flash drive used
                  logcopycount++;
                  LineCount = 0;

                  // clear the log buffer
                  memset(sdlogbuffer, '\0',
                         sizeof(sdlogbuffer) / sizeof(*sdlogbuffer));
                  while (LogFile2.available() > 0) {
                    sdlog = LogFile2.read();
                    if (sdlog != char(13)) {
                      sdlogbuffer[logpointer++] = sdlog;
                    } else {
                      LineCount++;
                      FTImpl
                          .Cmd_DLStart();  // start new display list - ends with
                                           // DL_swap///////////////////////////////////////////////////////////////////////////////////////
                      FTImpl.Begin(
                          FT_BITMAPS);  // Start a new graphics primitive
                      FTImpl.ClearColorRGB(100, 100, 100);
                      FTImpl.Clear(1, 1, 1);
                      FTImpl.Vertex2ii(0, 0, 0, 0);  // Draw primitive
                      FTImpl.BitmapHandle(0);
                      FTImpl.BitmapSource(0);
                      FTImpl.BitmapLayout(FT_RGB565, 480L * 2, 272);
                      FTImpl.BitmapSize(FT_NEAREST, FT_BORDER, FT_BORDER, 480,
                                        272);
                      FTImpl.ColorRGB(0xff, 0xff, 0xff);
                      FTImpl.Cmd_Text(230, 50, 31, FT_OPT_CENTER,
                                      "Copying....");
                      FTImpl.Cmd_Text(230, 100, 29, FT_OPT_CENTER, "File:");
                      FTImpl.Cmd_Text(230, 130, 29, FT_OPT_CENTER, LogFileName);
                      LineCountString[0] = char(
                          48 + int(LineCount /
                                   1000));  // can cope with up to 9999 lines
                      LineCountString[1] = char(
                          48 + int((LineCount - int(LineCount / 1000) * 1000) /
                                   100));
                      LineCountString[2] = char(
                          48 +
                          int((LineCount - int(LineCount / 100) * 100) / 10));
                      LineCountString[3] =
                          char(48 + int(LineCount - int(LineCount / 10) * 10));
                      LineCountString[4] = '\0';
                      FTImpl.Cmd_Text(230, 160, 29, FT_OPT_CENTER, "Line:");
                      FTImpl.Cmd_Text(230, 190, 29, FT_OPT_CENTER,
                                      LineCountString);
                      // Serial.println(LineCountString);
                      FTImpl.Display();
                      FTImpl.Cmd_Swap();
                      FTImpl.Finish();

                      flash_data(sdlogbuffer, false);
                      memset(sdlogbuffer, '\0', logpointer);
                      logpointer = 0;
                    }
                  }
                  // Close the file by sending Control-Z
                  mySerial.write(26);  // 26 is Control-Z character
                  LogFile2.close();
                  sd.remove(LogFileName);  // delete the logfile
                }
              }
            } else {
              LogRef = 0;
              delay(2000);
              Screen = 1;
            }
          } while (LogRef > 0);

          FTImpl
              .Cmd_DLStart();  // start new display list - ends with
                               // DL_swap///////////////////////////////////////////////////////////////////////////////////////
          FTImpl.Begin(FT_BITMAPS);  // Start a new graphics primitive
          FTImpl.ClearColorRGB(100, 100, 100);
          FTImpl.Clear(1, 1, 1);
          FTImpl.Vertex2ii(0, 0, 0, 0);  // Draw primitive
          FTImpl.BitmapHandle(0);
          FTImpl.BitmapSource(0);
          FTImpl.BitmapLayout(FT_RGB565, 480L * 2, 272);
          FTImpl.BitmapSize(FT_NEAREST, FT_BORDER, FT_BORDER, 480, 272);
          FTImpl.ColorRGB(0xff, 0xff, 0xff);
          LineCountString[0] = char(
              48 + int(logcopycount / 100));  // can cope with up to 999 lines
          LineCountString[1] = char(
              48 + int((logcopycount - int(logcopycount / 100) * 100) / 10));
          LineCountString[2] =
              char(48 + int(logcopycount - int(logcopycount / 10) * 10));
          LineCountString[3] = '\0';
          FTImpl.Cmd_Text(230, 160, 29, FT_OPT_CENTER, LineCountString);
          FTImpl.Cmd_Text(230, 190, 29, FT_OPT_CENTER, "Files copied.");
          FTImpl.Display();
          FTImpl.Cmd_Swap();
          FTImpl.Finish();
          delay(2000);
        }
      }

      if (tagval == 17)  // Product info
      {
        Screen = 10;
      }

      if (tagval == 18)  // More
      {
        FTImpl.DLStart();
        FTImpl.ClearColorRGB(64, 64, 64);
        FTImpl.Clear(1, 1, 1);
        FTImpl.Cmd_Swap();
        Float = ((float)EnergyDensity - 100) / 300 * 65535;
        NewEnergyDensity = Float;
        NED = NewEnergyDensity;
        Float = (float)Current / 100 * 65535;
        NewCurrent = Float;
        NC = NewCurrent;
        Screen = 5;
        if (FirstPass) {
          FirstPass = false;
        } else {
          FirstPass = true;
          Notepad();
        }
      }
    }
  }
}

/* Helper API to convert decimal to ascii - pSrc shall contain NULL terminated
 * string */
int32_t Dec2Ascii(char *pSrc, int32_t value) {
  int16_t Length;
  char *pdst, charval;
  int32_t CurrVal = value, tmpval, i;
  int8_t idx = 0;
  char tmparray[16];  // assumed that output string will not exceed 16
                      // characters including null terminated character

  // get the length of the string
  Length = strlen(pSrc);
  pdst = pSrc + Length;

  // cross check whether 0 is sent
  if (0 == value) {
    *pdst++ = '0';
    *pdst++ = '\0';
    return 0;
  }

  // handling of -ve number
  if (CurrVal < 0) {
    *pdst++ = '-';
    CurrVal = -CurrVal;
  }
  /* insert the digits */
  while (CurrVal > 0) {
    tmpval = CurrVal;
    CurrVal /= 10;
    tmpval = tmpval - CurrVal * 10;
    charval = '0' + tmpval;
    tmparray[idx++] = charval;
  }

  // flip the digits for the normal order
  for (i = 0; i < idx; i++) {
    *pdst++ = tmparray[idx - i - 1];
  }
  *pdst++ = '\0';

  return 0;
}  // end of Dec2Ascii

/* API for calibration on ft801 */
void Calibrate() {
  /*************************************************************************/
  /* Below code demonstrates the usage of calibrate function. Calibrate    */
  /* function will wait untill user presses all the three dots. Only way to*/
  /* come out of this api is to reset the coprocessor bit.                 */
  /*************************************************************************/

  /* Construct the display list with grey as background color, informative
   * string "Please Tap on the dot" followed by inbuilt calibration command */
  FTImpl.DLStart();
  FTImpl.ClearColorRGB(64, 64, 64);
  FTImpl.Clear(1, 1, 1);
  FTImpl.ColorRGB(0xff, 0xff, 0xff);
  FTImpl.Cmd_Text((FT_DISPLAYWIDTH / 2), (FT_DISPLAYHEIGHT / 2) - 40, 27,
                  FT_OPT_CENTER, "Power-up screen calibration.");
  FTImpl.Cmd_Text((FT_DISPLAYWIDTH / 2), (FT_DISPLAYHEIGHT / 2), 27,
                  FT_OPT_CENTER, "Please tap carefully on the dot.");
  // FTImpl.Cmd_Text((FT_DISPLAYWIDTH/2), (FT_DISPLAYHEIGHT/2)+40, 27,
  // FT_OPT_CENTER, "with a pointer");

  FTImpl.Cmd_Calibrate(0);

  /* Wait for the completion of calibration - either finish can be used for
   * flush and check can be used */
  FTImpl.Finish();
}

/* Apis used to upload the image to GRAM from SD card*/
void Load_Jpeg(
    FT_SDFile &fname)  //&fname appears to be the address of the Imagefile
{
  uint8_t imbuff[512];
  while (fname.Offset < fname.Size) {
    uint16_t n = min(512, fname.Size - fname.Offset);
    n = (n + 3) & ~3;  // force 32-bit alignment
    fname.ReadSector(imbuff);
    FTImpl.WriteCmd(imbuff, n); /* copy data continuously into command memory */
  }
}

void Loadimage2ram() {
  FtSd.OpenFile(Imagefile, imagename[Screen]);
  FTImpl.WriteCmd(CMD_LOADIMAGE);
  FTImpl.WriteCmd(
      0);  // pointer - was 500: 100  262144L instead of 500 gives blank pages
  FTImpl.WriteCmd(0);
  Load_Jpeg(Imagefile);
  return;
}

/* Api to bootup ft801, verify FT801 hardware and configure display/audio pins
 */
/* Returns 0 in case of success and 1 in case of failure */
int16_t BootupConfigure() {
  uint32_t chipid = 0;
  FTImpl.Init(FT_DISPLAY_RESOLUTION);  // configure the display to the WQVGA

  delay(20);  // for safer side
  chipid = FTImpl.Read32(FT_ROM_CHIPID);

  /* Identify the chip */
  if (FT801_CHIPID != chipid) {
    Serial.print("Error in chip id read ");
    Serial.println(chipid, HEX);
    return 1;
  }

  /*Platform pressure sensitivity adjustment*/
  //  FTImpl.Write16(REG_TOUCH_RZTHRESH,1200);

  /* Set the Display & audio pins */
  FTImpl.SetDisplayEnablePin(FT_DISPENABLE_PIN);
  FTImpl.SetAudioEnablePin(FT_AUDIOENABLE_PIN);
  FTImpl.DisplayOn();
  FTImpl.AudioOn();
  return 0;
}

/* Display an on-screen message if the storage device is not found */
void CheckStorageDevicePresence() {
  while (sd_present) {
    FTImpl.DLStart();
    FTImpl.Clear(1, 1, 1);
    FTImpl.ColorRGB(255, 255, 255);
    FTImpl.Cmd_Text(FT_DISPLAYWIDTH >> 1, FT_DISPLAYHEIGHT >> 1, 29,
                    FT_OPT_CENTER, "STORAGE DEVICE NOT FOUND");
    FTImpl.DLEnd();
    FTImpl.Finish();
    delay(1000);
  }
}

// Create a printing function which has a built-in delay
void flash_data(char *pstring, boolean Print) {
  if (Print) Serial.println(pstring);
  mySerial.println(pstring);
  delay(50);
}

void RTC_init() {
  Serial.println("RTC_init_start");

  // start the SPI library:
  Serial.print("1");
  // SPI.begin();  already done in sd_present = FtSd.Init();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  Serial.print("2");
  // set control register
  digitalWrite(RTC_CS, LOW);
  Serial.print("3");
  SPI.transfer(0x8E);
  Serial.print("4");
  SPI.transfer(0x60);  // 60= disable Osciallator and Battery SQ wave @1hz, temp
                       // compensation, Alarms disabled
  Serial.print("5");
  digitalWrite(RTC_CS, HIGH);
  Serial.print("6");
  SPI.setDataMode(SPI_MODE0);
  delay(10);
  Serial.println("RTC_init_end");
}

//=====================================
void SetTimeDate(int d, int mo, int y, int h, int mi, int s) {
  int TimeDate[7] = {s, mi, h, 0, d, mo, y};
  SPI.setDataMode(SPI_MODE3);
  for (int i = 0; i <= 6; i++) {
    if (i == 3) i++;
    int b = TimeDate[i] / 10;
    int a = TimeDate[i] - b * 10;
    if (i == 2) {
      if (b == 2)
        b = B00000010;
      else if (b == 1)
        b = B00000001;
    }
    TimeDate[i] = a + (b << 4);

    digitalWrite(RTC_CS, LOW);
    SPI.transfer(i + 0x80);
    SPI.transfer(TimeDate[i]);
    digitalWrite(RTC_CS, HIGH);
  }
  SPI.setDataMode(SPI_MODE0);
}

//=====================================
void ReadTimeDate(int TimeDate[]) {
  // int TimeDate [7]; //second,minute,hour,null,day,month,year
  SPI.setDataMode(SPI_MODE3);
  for (int j = 0; j <= 6; j++) {
    if (j == 3) j++;
    digitalWrite(RTC_CS, LOW);
    SPI.transfer(j + 0x00);
    unsigned int n = SPI.transfer(0x00);
    digitalWrite(RTC_CS, HIGH);
    int a = n & B00001111;
    if (j == 2) {
      int b = (n & B00110000) >> 4;  // 24 hour mode
      if (b == B00000010)
        b = 20;
      else if (b == B00000001)
        b = 10;
      TimeDate[j] = a + b;
    } else if (j == 4) {
      int b = (n & B00110000) >> 4;
      TimeDate[j] = a + b * 10;
    } else if (j == 5) {
      int b = (n & B00010000) >> 4;
      TimeDate[j] = a + b * 10;
    } else if (j == 6) {
      int b = (n & B11110000) >> 4;
      TimeDate[j] = a + b * 10;
    } else {
      int b = (n & B01110000) >> 4;
      TimeDate[j] = a + b * 10;
    }
  }
  SPI.setDataMode(SPI_MODE0);
}

//=====================================
String ConvertTimeDate(int TimeDate[]) {
  char buffer[20];
  if (TimeDate[4] < 1 || TimeDate[4] > 31 || TimeDate[5] < 1 ||
      TimeDate[5] > 12 || TimeDate[6] < 0 || TimeDate[6] > 3000 ||
      TimeDate[2] < 0 || TimeDate[2] > 24 || TimeDate[1] < 0 ||
      TimeDate[1] > 60 || TimeDate[0] < 0 || TimeDate[0] > 60) {
    return String("00/00/0000 00:00:00");
  }
  sprintf(buffer, "%02d/%02d/%4d %02d:%02d:%02d", TimeDate[4], TimeDate[5],
          TimeDate[6], TimeDate[2], TimeDate[1], TimeDate[0]);

  String temp(buffer);

  return temp;
}

#define ON 1
#define OFF 0
#define Font 27  // Font Size

#define SPECIAL_FUN 251
#define BACK_SPACE 251   // Back space
#define CAPS_LOCK 252    // Caps Lock
#define NUMBER_LOCK 253  // Number Lock
#define BACK 254         // Exit

#define LINE_STARTPOS FT_DISPLAYWIDTH / 50  // Start of Line
#define LINE_ENDPOS FT_DISPLAYWIDTH         // max length of the line

struct {
  uint8_t Key_Detect : 1;
  uint8_t Caps : 1;
  uint8_t Numeric : 1;
  uint8_t Exit : 1;
} Flag;

// Notepad buffer
NotepadResult Notepad(const char *initialText) {
  /*local variables*/
  uint8_t Line = 0;
  uint16_t Disp_pos = 0, But_opt;
  uint8_t tval;
  uint16_t noofchars = 0, line2disp = 0, nextline = 0;
  uint8_t font = 26, offset = 50;

  // Clear then set Linebuffer
  for (tval = 0; tval < MAX_FT_LINES; tval++)
    memset(&Buffer.notepad[tval], '\0',
           sizeof(Buffer.notepad[tval]));  // set all of buffer to be null

  for (noofchars = 0; initialText[noofchars] != '\0';
       ++noofchars)  // load in the Project Description
  {
    Buffer.notepad[0][noofchars] = initialText[noofchars];
  }

  Serial.print("buffer: ");
  Serial.println(Buffer.notepad[Line]);

  /*intial setup*/
  Line = 0;                  // Starting line
  Disp_pos = LINE_STARTPOS;  // starting pos
  Flag.Numeric = OFF;        // Disable the numbers and special charaters
  memset((Buffer.notepad[Line] + noofchars), '_',
         1);  // For Cursor //noofchars was 0
  Disp_pos +=
      Ft_Gpu_Rom_Font_WH(Buffer.notepad[Line][0], Font);  // Update the Disp_Pos
  Flag.Exit = 0;

  KeyPressTracker kpt(&FTImpl);

  // Serial.println(Read_sfk);
  // Serial.println(Line);
  // Serial.println(Disp_pos);
  Serial.print("buffer: ");
  Serial.println(Buffer.notepad[Line]);

  uint8_t buttonPressTag = 0;
  uint8_t currentPressedTag = 0;

  do {
    if (13 == buttonPressTag)  // quit
    {
      FTImpl.DLStart();
      FTImpl.ClearColorRGB(64, 64, 64);
      FTImpl.Clear(1, 1, 1);
      FTImpl.Cmd_Swap();
      setNextScreen(DisplayScreen::kDisplayScreenHome);
      goto Letsgetoutofhere;
    }

    if (21 == buttonPressTag && noofchars > 1)  // save
    {
      FTImpl.DLStart();
      FTImpl.ClearColorRGB(64, 64, 64);
      FTImpl.Clear(1, 1, 1);
      FTImpl.Cmd_Swap();
      if (DisplayScreen::kDisplayScreenNewExp == currentScreen) {
        // Remove the _ cursor
        Buffer.notepad[Line][noofchars] = '\0';
        return kNotepadResultSave;
      } else {
        if ((Buffer.notepad[0][0] == '3') && (Buffer.notepad[0][1] == '3') &&
            (Buffer.notepad[0][2] == '3') && (Buffer.notepad[0][3] == '3')) {
          Screen = 5;  // Goto setings screen
        } else {
          setNextScreen(DisplayScreen::kDisplayScreenHome);
        }
        goto Letsgetoutofhere;
      }
    }

    if (buttonPressTag >=
        SPECIAL_FUN) {  // check any special function keys are pressed
      switch (buttonPressTag) {
        case BACK_SPACE:
          if (noofchars > 0)  // check in the line there is any characters are
                              // present,cursor not include
          {
            noofchars -= 1;  // clear the character in the buffer
            Disp_pos -=
                Ft_Gpu_Rom_Font_WH(*(Buffer.notepad[Line] + noofchars - 1),
                                   Font);  // Update the Disp_Pos
          } else if (Line != 0) {
            Line--;
            noofchars =
                strlen(Buffer.notepad[Line]);  // Read the len of the line
            for (tval = 0; tval < noofchars;
                 tval++)  // Compute the length of the Line
              Disp_pos += Ft_Gpu_Rom_Font_WH(Buffer.notepad[Line][tval],
                                             Font);  // Update the Disp_Pos
          }
          Buffer.temp =
              (Buffer.notepad[Line] + noofchars);  // load into temporary buffer
          Buffer.temp[0] = '_';                    // update the string
          Buffer.temp[1] = '\0';
          Serial.print("BACKSPACE buffer: ");
          Serial.println(Buffer.notepad[Line]);
          break;

        case CAPS_LOCK:
          Flag.Caps ^= 1;  // toggle the caps lock on when the key detect
          break;

        case NUMBER_LOCK:
          Flag.Numeric ^= 1;  // toggle the number lock on when the key detect
          break;

        case BACK:
          for (tval = 0; tval < MAX_FT_LINES; tval++)
            memset(&Buffer.notepad[tval], '\0', sizeof(Buffer.notepad[tval]));
          Line = 0;                                    // Starting line
          Disp_pos = LINE_STARTPOS;                    // starting pos
          memset((Buffer.notepad[Line] + 0), '_', 1);  // For Cursor
          Disp_pos += Ft_Gpu_Rom_Font_WH(Buffer.notepad[Line][0],
                                         Font);  // Update the Disp_Pos
          noofchars = 0;
          break;
      }
    } else if (buttonPressTag >= 32)  // it is a standard character
    {
      // String and Line Termination

      Disp_pos += Ft_Gpu_Rom_Font_WH(buttonPressTag, Font);  // update dis_pos
      if (strlen(Buffer.notepad[0]) <
          40)  // attempt to limit the character count
      {
        noofchars += 1;
        Buffer.notepad[Line][noofchars - 1] = buttonPressTag;
        Buffer.notepad[Line][noofchars] = '_';
        Buffer.notepad[Line][noofchars + 1] = '\0';

        Serial.print("Add char ");
        Serial.print(buttonPressTag);
        Serial.print(" buffer: ");
        Serial.println(Buffer.notepad[Line]);
        // Serial.print(noofchars);
        /*if(Disp_pos > LINE_ENDPOS)                                        //
      check if there is place to put a character in a specific line
      {
        Buffer.temp = Buffer.notepad[Line]+(strlen(Buffer.notepad[Line])-1);
        Buffer.temp[0] = '\0';
        noofchars-=1;
        Disp_pos = LINE_STARTPOS;
        Line++; if(Line >= MAX_FT_LINES)  Line = 0;
        memset((Buffer.notepad[Line]),'\0',sizeof(Buffer.notepad[Line])); //
      Clear the line buffer for(;noofchars>=1;noofchars--,tval++)
        {
          if(Buffer.notepad[Line-1][noofchars] == ' '
      ||Buffer.notepad[Line-1][noofchars] =='.')  // In case of space(New
      String) or end of statement(.)
            {
              memset(Buffer.notepad[Line-1]+noofchars,'\0',1);
              noofchars+=1;             // Include the space
              memcpy(&Buffer.notepad[Line],(Buffer.notepad[Line-1]+noofchars),tval);
              break;
            }
        }
        noofchars = strlen(Buffer.notepad[Line]);
        Buffer.temp = Buffer.notepad[Line]+noofchars;
        Buffer.temp[0] = '_';
        Buffer.temp[1] = '\0';
        for(tval=0;tval<noofchars;tval++)
        {
          Disp_pos += Ft_Gpu_Rom_Font_WH(Buffer.notepad[Line][tval],Font);  //
      Update the Disp_Pos
        }
      }*/
      }
    }

    // Start the new Display list
    FTImpl.DLStart();
    FTImpl.ClearColorRGB(100, 100, 100);
    FTImpl.Clear(1, 1, 1);
    FTImpl.ColorRGB(255, 255, 255);
    FTImpl.TagMask(1);  // enable tag buffer updation
    FTImpl.Cmd_FGColor(kColourPrimary);
    FTImpl.Cmd_BGColor(0x19354B);
    But_opt = (currentPressedTag == BACK)
                  ? FT_OPT_FLAT
                  : 0;  // button color change if the button during press
    FTImpl.Tag(BACK);   // Back    Return to Home
    FTImpl.Cmd_Button((FT_DISPLAYWIDTH * 0.855),
                      (FT_DISPLAYHEIGHT * 0.83 - offset),
                      (FT_DISPLAYWIDTH * 0.146), (FT_DISPLAYHEIGHT * 0.112),
                      font, But_opt, "Clear");
    But_opt = (currentPressedTag == BACK_SPACE) ? FT_OPT_FLAT : 0;
    FTImpl.Tag(BACK_SPACE);  // BackSpace
    FTImpl.Cmd_Button((FT_DISPLAYWIDTH * 0.875),
                      (FT_DISPLAYHEIGHT * 0.70 - offset),
                      (FT_DISPLAYWIDTH * 0.125), (FT_DISPLAYHEIGHT * 0.112),
                      font, But_opt, "<-");
    But_opt = (currentPressedTag == ' ') ? FT_OPT_FLAT : 0;
    FTImpl.Tag(' ');  // Space
    FTImpl.Cmd_Button((FT_DISPLAYWIDTH * 0.115),
                      (FT_DISPLAYHEIGHT * 0.83 - offset),
                      (FT_DISPLAYWIDTH * 0.73), (FT_DISPLAYHEIGHT * 0.112),
                      font, But_opt, "Space");

    if (Flag.Numeric == OFF) {
      FTImpl.Cmd_Keys(0, (FT_DISPLAYHEIGHT * 0.442 - offset), FT_DISPLAYWIDTH,
                      (FT_DISPLAYHEIGHT * 0.112), font, currentPressedTag,
                      (Flag.Caps == ON ? "QWERTYUIOP" : "qwertyuiop"));
      FTImpl.Cmd_Keys(
          (FT_DISPLAYWIDTH * 0.042), (FT_DISPLAYHEIGHT * 0.57 - offset),
          (FT_DISPLAYWIDTH * 0.96), (FT_DISPLAYHEIGHT * 0.112), font,
          currentPressedTag, (Flag.Caps == ON ? "ASDFGHJKL" : "asdfghjkl"));
      FTImpl.Cmd_Keys(
          (FT_DISPLAYWIDTH * 0.125), (FT_DISPLAYHEIGHT * 0.70 - offset),
          (FT_DISPLAYWIDTH * 0.73), (FT_DISPLAYHEIGHT * 0.112), font,
          currentPressedTag, (Flag.Caps == ON ? "ZXCVBNM" : "zxcvbnm"));

      But_opt = (currentPressedTag == CAPS_LOCK) ? FT_OPT_FLAT : 0;
      FTImpl.Tag(CAPS_LOCK);  // Capslock
      FTImpl.Cmd_Button(0, (FT_DISPLAYHEIGHT * 0.70 - offset),
                        (FT_DISPLAYWIDTH * 0.10), (FT_DISPLAYHEIGHT * 0.112),
                        font, But_opt, "a^");
      But_opt = (currentPressedTag == NUMBER_LOCK) ? FT_OPT_FLAT : 0;
      FTImpl.Tag(NUMBER_LOCK);  // Numberlock
      FTImpl.Cmd_Button(0, (FT_DISPLAYHEIGHT * 0.83 - offset),
                        (FT_DISPLAYWIDTH * 0.10), (FT_DISPLAYHEIGHT * 0.112),
                        font, But_opt, "12*");
    }
    if (Flag.Numeric == ON) {
      FTImpl.Cmd_Keys(0, (FT_DISPLAYHEIGHT * 0.442 - offset), FT_DISPLAYWIDTH,
                      (FT_DISPLAYHEIGHT * 0.112), font, currentPressedTag,
                      "1234567890");
      FTImpl.Cmd_Keys((FT_DISPLAYWIDTH * 0.042),
                      (FT_DISPLAYHEIGHT * 0.57 - offset),
                      (FT_DISPLAYWIDTH * 0.96), (FT_DISPLAYHEIGHT * 0.112),
                      font, currentPressedTag, "-@#$%^&*(");
      FTImpl.Cmd_Keys((FT_DISPLAYWIDTH * 0.125),
                      (FT_DISPLAYHEIGHT * 0.70 - offset),
                      (FT_DISPLAYWIDTH * 0.73), (FT_DISPLAYHEIGHT * 0.112),
                      font, currentPressedTag, ")_+[]{}");
      But_opt = (currentPressedTag == NUMBER_LOCK) ? FT_OPT_FLAT : 0;
      FTImpl.Tag(253);  // Numberlock
      FTImpl.Cmd_Button(0, (FT_DISPLAYHEIGHT * 0.83 - offset),
                        (FT_DISPLAYWIDTH * 0.10), (FT_DISPLAYHEIGHT * 0.112),
                        font, But_opt, "AB*");
    }

    // assign tag value 21 to the save button
    drawBottomRightButton(
        21,
        DisplayScreen::kDisplayScreenNewExp == currentScreen ? "Save" : "Enter",
        currentPressedTag);
    // assign tag value 13 to the quit button
    drawBottomLeftButton(13, "Quit", currentPressedTag);

    FTImpl.TagMask(0);  // Disable the tag buffer updates
    FTImpl.ScissorXY(0, 0);
    FTImpl.ScissorSize(FT_DISPLAYWIDTH,
                       (uint16_t)(FT_DISPLAYHEIGHT * 0.405 - offset));
    FTImpl.ClearColorRGB(255, 255, 255);
    FTImpl.Clear(1, 1, 1);
    FTImpl.ColorRGB(0, 0, 0);  // Text Color
    line2disp = 0;
    while (line2disp <= Line) {
      nextline = 3 + (line2disp * (FT_DISPLAYHEIGHT * .073));
      FTImpl.Cmd_Text(line2disp, nextline, font, 0,
                      (const char *)&Buffer.notepad[line2disp]);
      line2disp++;
    }
    FTImpl.DLEnd();
    FTImpl.Finish();

    buttonPressTag = kpt.waitForChange(currentPressedTag);
    Serial.print("Notepad screen button pressed: ");
    Serial.print(buttonPressTag);
    Serial.print(" selectedTag: ");
    Serial.println(currentPressedTag);

  } while (1);

Letsgetoutofhere:
  delay(10);
  return NotepadResult::kNotepadResultQuit;
}
