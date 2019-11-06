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

#include <Arduino.h>

#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>  //SD card library
#include <SdFatConfig.h>
#include <SysCall.h>
#include <sdios.h>

#include <FT_NHD_43CTP_SHIELD.h>
//#include <FT_VM801B43.h>
//#include <FT_VM801P43_50.h>
//#include "FT_NHD_43RTP_SHIELD.h"  //new screen
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <OneWire.h>  //control devices (from Dallas Semiconductor) that use the One Wire protocol
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Wire.h>  //Allows communication with I2C devices via SDA (data line) and SCL (clock line)
#include <stdio.h>
#include "Adafruit_VEML6070.h"  //UV sensor

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
int Intensity = 100;
int32_t Current = 100, NewCurrent, NC;  // Current in %
float SetCurrent = 1.2 / 5.0 * 255;     // SetCurrent in 0-5V converted to 0-255
float Energy = 0;
int Time = 300, iTime, i, ds, OldiTime, it;
unsigned long msTime, msTimeLid;
int uv2 = 0;
uint16_t uvValue;
char uvPrintVal[8];
char uvPrint[15] = {'u', 'v', ' ', '=', ' ', '0', '0', '0', '0', '0', '\0'};
char tempPrintValA[3], tempPrintValB[3];
char tempPrint[15] = {'t', 'e', 'm', 'p', ' ', '=', ' ',
                      '0', '0', '0', '.', '0', '\0'};
char OutputValue[5];
char TimeString[6] = "00:00";
// int TimeStepUp = 1, TimeStepUpCount = 0, TimeStepDown = 1, TimeStepDownCount
// = 0;
char TimeDateString[18];
int TimeAndDate[7];
int TimePointer[] = {4, 5, 6, 2, 1, 0};
int TimeMax[] = {31, 12, 99, 23, 59, 59};
int TimeMin[] = {1, 1, 19, 0, 0, 0};
int TimeDigit = 2;
char ProjectString[150] = {'P', 'r', 'o', 'j', 'e', 'c', 't', ' ', 'd', 'e',
                           's', 'c', 'r', 'i', 'p', 't', 'i', 'o', 'n', ' ',
                           'i', 'n', ' ', 'u', 'p', ' ', 't', 'o', ' ', '4',
                           '0', ' ', 'c', 'h', 'a', 'r', 's', '\0'};
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
  for (i = 0; i < 8; i++) {
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
  Energy =
      (float)Time * (float)EnergyDensity / 1000.0 * (float)Intensity / 100.0;

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
  Serial.println("Setup complete.....");
}

const uint32_t kColourPrimary = 0x2A5673 + 0x001000;  // dark greeny
const uint32_t kColourSeconday = 0x6A8CA5;            // lighter
const uint32_t kColourLight = 0xD2D8E1;

static int32_t minutes = 5;
static int32_t seconds = 0;
static int32_t time = 60;
static int32_t irradience = 50;
static int32_t energy = 300;

enum HeldSlider { kHeldSliderTime, kHeldSliderIrradience, kHeldSliderEnergy };
static HeldSlider heldSlider = kHeldSliderIrradience;

enum DisplayScreen {
  kDisplayScreenHome = 0,
  kDisplayScreenNewExp,
  kDisplayScreenBrowseExperiments,
  kDisplayScreenExpSettings,
  kDisplayScreenRun,
};
bool screenJustSelected = true;
DisplayScreen currentScreen = kDisplayScreenHome;
const uint8_t kFont = 26;

static void setNextScreen(DisplayScreen screen) {
  screenJustSelected = true;
  currentScreen = screen;
}

void homeScreen(uint8_t selectedTag) {
  if (screenJustSelected) {
    Screen = 0;
    Loadimage2ram();
    screenJustSelected = false;
  }

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
  FTImpl.Tag(kNewExperimentTag);
  uint16_t options = kNewExperimentTag == selectedTag ? FT_OPT_FLAT : 0;
  FTImpl.Cmd_Button(FT_DISPLAY_HSIZE / 4 - buttonWidth / 2, 200, buttonWidth,
                    30, kFont, options, "New experiment");

  const uint8_t kPrevExperimentTag = 12;
  FTImpl.Tag(kPrevExperimentTag);
  options = kPrevExperimentTag == selectedTag ? FT_OPT_FLAT : 0;
  FTImpl.Cmd_Button(FT_DISPLAY_HSIZE * 3 / 4 - buttonWidth / 2, 200,
                    buttonWidth, 30, kFont, options, "Rerun experiment");

  if (kNewExperimentTag == selectedTag) {
    setNextScreen(kDisplayScreenNewExp);
    Screen = 3;  // TODO remove once new exp screen updated
  } else if (kPrevExperimentTag == selectedTag) {
    setNextScreen(kDisplayScreenBrowseExperiments);
  }
}

void newExpScreen(uint16_t currentScreen) {
  NotepadResult result = Notepad(ProjectString);
  if (kNotepadResultSave == result) {
    strncpy(ProjectString, Buffer.notepad[0],
            min(sizeof(ProjectString), sizeof(Buffer.notepad[0])));
    ProjectString[sizeof(ProjectString) - 1] = '\0';
    setNextScreen(kDisplayScreenExpSettings);
  } else {
    setNextScreen(kDisplayScreenHome);
  }
}

void drawHoldToggle(uint16_t x, uint16_t y, bool on) {
  const char *kHoldToggleText = "\xffhold";
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

void experimentSettingsScreen(uint8_t currentTag) {
  FTImpl.Cmd_DLStart();
  FTImpl.ClearColorRGB(64, 64, 64);
  FTImpl.Clear(1, 1, 1);
  FTImpl.TagMask(1);

  const uint8_t kRunTag = 14;
  uint16_t options = kRunTag == currentTag ? FT_OPT_FLAT : 0;
  FTImpl.Tag(kRunTag);
  FTImpl.Cmd_Button(423 - 47, 241 - 19, 94, 38, 26, options, "Run");

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
  const int16_t topSliderY = 80;
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
  FTImpl.Cmd_Track(sliderLeft, topSliderY + 2 * sliderYSpacing, sliderWidth, 8,
                   energySliderTag);

  int tagval = 0;
  uint32_t TrackRegisterVal = FTImpl.Read32(REG_TRACKER);
  tagval = TrackRegisterVal & 0xff;

  int32_t sliderTrackerVal;

  const int32_t kMaxIrradience = 100;  // needs units not percentage?

  const int32_t kMaxMinutes = 30;
  const int32_t kMaxSeconds = 59;
  if (timeSliderMinsTag == tagval || timeSliderSecsTag == tagval) {
    if (timeSliderMinsTag == tagval) {
      sliderTrackerVal = TrackRegisterVal >> 16;  // value is 0 - 65535
      minutes = kMaxMinutes * sliderTrackerVal / 65535;
    } else if (timeSliderSecsTag == tagval) {
      sliderTrackerVal = TrackRegisterVal >> 16;
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
        minutes = time / 60;
        seconds = time - 60 * minutes;
      } else if (irradience < 1) {
        irradience = 1;
        time = energy / irradience;
        minutes = time / 60;
        seconds = time - 60 * minutes;
      }
    } else {
      energy = irradience * time;
    }
  } else if (irradienceSliderTag == tagval) {
    sliderTrackerVal = TrackRegisterVal >> 16;  // value is 0 - 65535
    irradience = kMaxIrradience * sliderTrackerVal / 65535;
    irradience = max(irradience, 1);

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
      minutes = time / 60;
      seconds = time - 60 * minutes;
    } else {
      energy = irradience * time;
    }
  } else if (energySliderTag == tagval) {
    sliderTrackerVal = TrackRegisterVal >> 16;  // value is 0 - 65535
    energy =
        (kMaxMinutes * 60 * kMaxIrradience) / 16 * sliderTrackerVal /
        (65535 / 16);  // TODO check as max minutes and max irradience change
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
      minutes = time / 60;
      seconds = time - 60 * minutes;
    } else if (time != 0) {
      irradience = energy / time;
      if (irradience > kMaxIrradience) {
        irradience = kMaxIrradience;
        energy = irradience * time;
      } else if (irradience < 1) {
        irradience = 1;
        energy = irradience * time;
      }
    }
  }

  switch (currentTag) {
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
      setNextScreen(kDisplayScreenRun);
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
                       sliderWidth / 2 - 20, sliderHeight, seconds, kMaxSeconds,
                       heldSlider == kHeldSliderTime);
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
  drawSliderOrProgress(sliderLeft, topSliderY + 2 * sliderYSpacing, sliderWidth,
                       sliderHeight, energy / 60, kMaxMinutes * kMaxIrradience,
                       heldSlider == kHeldSliderEnergy);
  FTImpl.Tag(kEnergyHoldTag);
  drawHoldToggle(sliderLeft + sliderWidth + 60, topSliderY + 2 * sliderYSpacing,
                 heldSlider == kHeldSliderEnergy);

  FTImpl.TagMask(0);
  FTImpl.ColorRGB(0xff, 0xff, 0xff);
  FTImpl.Cmd_Text(60, 20, kFont, FT_OPT_CENTERY, ProjectString);

  char labelBuffer[30];
  sprintf(labelBuffer, "Duration: %2ld minutes", minutes);
  FTImpl.Cmd_Text(sliderLeft, topSliderY - 20, kFont, FT_OPT_CENTERY,
                  labelBuffer);
  sprintf(labelBuffer, "%2ld seconds", seconds);
  FTImpl.Cmd_Text(sliderLeft + sliderWidth / 2 + 20, topSliderY - 20, kFont,
                  FT_OPT_CENTERY, labelBuffer);

  sprintf(labelBuffer, "Irradiance: %3ld mW/cm^2", irradience);
  FTImpl.Cmd_Text(sliderLeft, topSliderY + sliderYSpacing - 20, kFont,
                  FT_OPT_CENTERY, labelBuffer);

  sprintf(labelBuffer, "Energy: %7ld mJ/cm^2", energy);
  FTImpl.Cmd_Text(sliderLeft, topSliderY + 2 * sliderYSpacing - 20, kFont,
                  FT_OPT_CENTERY, labelBuffer);
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

void startRunLog(File logFile)
{
  // TODO use correct variables and units
  logFile.println("Deliberately left blank");  // Sometimes lose first
                                                // line, so make it a dummy.
  logFile.print("Test ID: ");                  // Record project data
  logFile.println(ProjectString);
  logFile.print("Duration (s): ");
  logFile.print(Time);
  logFile.print(", ");
  logFile.print("Selected Intensity (%): ");
  logFile.print(Intensity);
  logFile.print(", ");
  logFile.print("Selected Current (%): ");
  logFile.println(Current);
  logFile.print("Calculation power density (uW/mm2): ");
  logFile.println(EnergyDensity);
  logFile.print("Energy applied (mW/cm2): ");
  logFile.println(Energy);
  logFile.print("Start Date/Time: ");
  logFile.println(ConvertTimeDate(TimeAndDate));
  logFile.println("Time (s), UV (relative), Temperature (Deg C)");
}

void runScreen(uint8_t currentTag) {
  FTImpl.Display();  // added as first go at getting screen 6 to work
  FTImpl.Cmd_Swap();
  FTImpl.Finish();
  Serial.println(SetCurrent);
  ReadTimeDate(TimeAndDate);
  Serial.println(ConvertTimeDate(TimeAndDate));

  File LogFile2 = tryOpenLogfile();
  // TODO error handling for logfile
  startRunLog(LogFile2);

  if (digitalRead(LID))  // Make sure lid is closed before start
  {
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
    FTImpl.Cmd_Text(230, 80, 31, FT_OPT_CENTER, "Close Lid");
    Serial.println("Close Lid");
    FTImpl.Cmd_Spinner(240, 150, 0, 0);
    FTImpl.Display();
    FTImpl.Cmd_Swap();
    FTImpl.Finish();
    do {
      delay(100);
    } while (digitalRead(LID));
  }
  analogWrite(ADM, 0);
  analogWrite(PWM, 5);  // LEDs power setting - see Mk6 - starting with
                        // PWM = 0 causes LED's to fail.
  delay(100);
  digitalWrite(FAN, HIGH);
  delay(1000);  // to make sure current setting voltage R-C circuit is
                // charged
  analogWrite(ADM, SetCurrent);  // turn on LEDs (slowly)
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
  FTImpl.Cmd_Text(230, 80, 28, FT_OPT_CENTER, "Starting, please wait....");
  FTImpl.Cmd_Spinner(240, 150, 0, 0);
  FTImpl.Display();
  FTImpl.Cmd_Swap();
  FTImpl.Finish();
  delay(10000);
  analogWrite(PWM, int((float)Intensity * 2.55));  // LEDs power setting

  msTime = millis();

  OldiTime = -1;
  uint8_t tagval = 42;
  LidOpen = false;
  uvPrintVal[0] = '\0';

  do {
    if (!LidOpen) iTime = Time - int((millis() - msTime) / 1000);
    TimeString[0] = char(48 + int(iTime / 600));
    TimeString[1] = char(48 + int(iTime / 60) - 10 * int(iTime / 600));
    TimeString[3] = char(48 + int((iTime - 60 * int(iTime / 60)) / 10));
    TimeString[4] = char(48 + iTime - 60 * int(iTime / 60) -
                         10 * int((iTime - 60 * int(iTime / 60)) / 10));
    // Serial.println(TimeString);
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
    FTImpl.Cmd_Text(230, 60, 31, FT_OPT_CENTER, TimeString);
    FTImpl.Cmd_Text(230, 10, 28, FT_OPT_CENTER, ProjectString);
    uint16_t tagoption = 0;  // no touch is default 3d effect and touch is flat
                             // effect
    if (13 == tagval) tagoption = FT_OPT_FLAT;
    FTImpl.Tag(13);
    FTImpl.Cmd_Button(63 - 47, 241 - 19, 94, 38, 26, tagoption, "Abort");

    if (!LidOpen) {
      sprintf(OutputValue, "%04i", Time);
      FTImpl.Cmd_Text(300, 90, 28, FT_OPT_CENTERX, "Duration (s):");
      FTImpl.Cmd_Text(450, 90, 28, FT_OPT_RIGHTX, OutputValue);
      sprintf(OutputValue, "%03i", Intensity);
      FTImpl.Cmd_Text(300, 120, 28, FT_OPT_CENTERX, "Intensity (%):");
      FTImpl.Cmd_Text(450, 120, 28, FT_OPT_RIGHTX, OutputValue);
      sprintf(OutputValue, "%03i", Current);
      FTImpl.Cmd_Text(300, 150, 28, FT_OPT_CENTERX, "Current (%):");
      FTImpl.Cmd_Text(450, 150, 28, FT_OPT_RIGHTX, OutputValue);
      sprintf(OutputValue, "%03i", EnergyDensity);
      FTImpl.Cmd_Text(300, 180, 28, FT_OPT_CENTERX, "Energy");
      FTImpl.Cmd_Text(300, 210, 28, FT_OPT_CENTERX, "Density (uW/mm2):");
      FTImpl.Cmd_Text(450, 210, 28, FT_OPT_RIGHTX, OutputValue);
    }

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
    if (LidOpen) {
      FTImpl.Cmd_Text(230, 120, 31, FT_OPT_CENTER, "Close Lid");
    }
    if (LidOpen && !digitalRead(LID))  // lid was open, but now closed.
    {
      analogWrite(PWM, int((float)Intensity * 2.55));
      msTime = msTime + (millis() - msTimeLid);  // correct the time
      LidOpen = false;
    }
    if (13 == tagval)  // abort pressed.
    {
      Serial.println("Abort hit");
      LogFile2.println("Abort hit");
      LidOpen = false;
      goto EscapeNestedLoops;
    }
    if (OldiTime != iTime)  // into a new second, so save results
    {
      OldiTime = iTime;
      Serial.print("Time = ");
      Serial.print(Time - iTime);
      LogFile2.print(iTime);
      Serial.print(", uv = ");
      LogFile2.print(", ");
      uvValue = uv.readUV();
      sprintf(uvPrintVal, "%05i",
              uvValue);  //%0 left pads the number with zeros, 5 = width,
                         // i = signed decimal integer
      for (i = 0; i < 5; i++) {
        uvPrint[i + 5] = uvPrintVal[i];
      }
      Serial.print(uvValue);
      LogFile2.print(uvValue);
      Serial.print(", t = ");
      LogFile2.print(", ");
      i = 0;
      do  // fiddle to sort out SPI clash
      {
        i++;
        SPI.end();
      } while ((SPCR & 64) == 64);
      // Serial.println(i);
      delay(10);
      sensors.requestTemperaturesByAddress(
          DallasAddress);  // can take up to 750ms to get temperature?
      float Temperature = sensors.getTempC(DallasAddress);
      if (Temperature != -127) {
        Serial.println(Temperature);
        sprintf(tempPrintValA, "%03i", int(Temperature));
        sprintf(tempPrintValB, "%01i",
                int((Temperature - int(Temperature)) * 10));
        for (it = 0; it < 3; it++) {
          tempPrint[it + 7] = tempPrintValA[it];
        }
        tempPrint[11] = tempPrintValB[0];
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
    if (!LidOpen) {
      FTImpl.Cmd_Text(100, 160, 29, FT_OPT_CENTER, tempPrint);
      FTImpl.Cmd_Text(100, 120, 29, FT_OPT_CENTER, uvPrint);
    }
    FTImpl.Display();
    FTImpl.Cmd_Swap();
    FTImpl.Finish();
    FTImpl.TagMask(1);
    sTagXY sTagxy;
    FTImpl.GetTagXY(sTagxy);
    tagval = sTagxy.tag;
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
  setNextScreen(kDisplayScreenHome);
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

  Screen = 0;
  setNextScreen(kDisplayScreenExpSettings);

  while (1) {
    FTImpl.TagMask(1);
    FTImpl.GetTagXY(sTagxy);
    tagval = sTagxy.tag;
    FTImpl.ClearColorRGB(64, 64, 64);  // text colour?

    if (kDisplayScreenHome == currentScreen) {
      homeScreen(tagval);
      tagval = 0;  // TODO remove, avoids below code detecting a press
    } else if (kDisplayScreenNewExp == currentScreen) {
      newExpScreen(tagval);
      tagval = 0;  // TODO remove, avoids below code detecting a press
    } else if (kDisplayScreenExpSettings == currentScreen) {
      experimentSettingsScreen(tagval);
      tagval = 0;  // TODO remove
    } else if (kDisplayScreenRun == currentScreen) {
      runScreen(tagval);
      tagval = 0;  // TODO remove
    } else {
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
        sprintf(OutputValue, "%03i", tmpNewCurrent);
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
    }
    FTImpl.Display();
    FTImpl.Cmd_Swap();
    FTImpl.Finish();
    // End of display
    // plotting/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Plot requests below here will nt get implemented.
    if (currentScreen != kDisplayScreenHome) {
      if (tagval == 11)  // Lightox main, run button picked
      {
        Screen = 3;  // select notepad before going to run screen
        Loadimage2ram();
        Notepad();
      }

      if (tagval == 12)  // Options
      {
        Screen = 1;
        Loadimage2ram();
      }

      if (tagval == 13)  // quit
      {
        FTImpl.Cmd_DLStart();
        FTImpl.ClearColorRGB(64, 64, 64);
        FTImpl.Clear(1, 1, 1);
        FTImpl.ColorRGB(0xff, 0xff, 0xff);
        FTImpl.Display();
        FTImpl.Cmd_Swap();
        FTImpl.Finish();
        setNextScreen(kDisplayScreenHome);
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
          Loadimage2ram();
        } else if (Screen == 2)  //=Date
        {
          Serial.println("Save date");
          SetTimeDate(TimeAndDate[TimePointer[0]], TimeAndDate[TimePointer[1]],
                      TimeAndDate[TimePointer[2]], TimeAndDate[TimePointer[3]],
                      TimeAndDate[TimePointer[4]], TimeAndDate[TimePointer[5]]);
          DateRead = false;
          setNextScreen(kDisplayScreenHome);
        }
      }

      if (tagval == 15)  // Set date and time
      {
        // FTImpl.DLStart();
        // FTImpl.ClearColorRGB(64,64,64);
        // FTImpl.Clear(1,1,1);
        // FTImpl.Cmd_Swap();
        Screen = 2;
        Loadimage2ram();
      }

      if (tagval == 16)  // Copy files
      {
        Screen = 9;
        Loadimage2ram();
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
              if ((LogFile2 = sd.open(LogFileName)) == NULL) {
                Serial.print(F("Log file not found"));
              } else {
                if (!digitalRead(
                        FLASH_IN))  // make sure flash drive is inserted
                {
                  Serial.print(F("No flash drive found"));
                } else {
                  for (i = 11; i < 14; i++) FlashFile[i + 1] = LogFileName[i];
                  flash_data(FlashFile, true);  // open file for write
                  delay(1000);  // Needs extra time to create the file.
                                // Depends on flash drive used
                  logcopycount++;
                  LineCount = 0;

                  for (i = 0; i < 80; i++)
                    sdlogbuffer[i] = '\0';  // clear the log buffer
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
                      for (i = 0; i < logpointer; i++) sdlogbuffer[i] = '\0';
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
              Loadimage2ram();
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
        // FTImpl.DLStart();
        // FTImpl.ClearColorRGB(64,64,64);
        // FTImpl.Clear(1,1,1);
        // FTImpl.Cmd_Swap();
        Screen = 10;
        Loadimage2ram();
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
        Loadimage2ram();
        if (FirstPass) {
          FirstPass = false;
        } else {
          FirstPass = true;
          Notepad();
        }
      }

      if (tagval == 22)  // Sam
      {
        // FTImpl.DLStart();
        // FTImpl.ClearColorRGB(64,64,64);
        // FTImpl.Clear(1,1,1);
        // FTImpl.Cmd_Swap();
        Screen = 7;
        Loadimage2ram();
      }

      if (tagval == 23)  // Carrie
      {
        // FTImpl.DLStart();
        // FTImpl.ClearColorRGB(64,64,64);
        // FTImpl.Clear(1,1,1);
        // FTImpl.Cmd_Swap();
        Screen = 8;
        Loadimage2ram();
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
  char tmparray[16], idx = 0;  // assumed that output string will not exceed 16
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
  static byte image_index = 0;  //,max_files=6;

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

void UpdateTime(uint16_t color) {
  TimeString[0] = char(48 + int(Time / 600));
  TimeString[1] = char(48 + int(Time / 60) - 10 * int(Time / 600));
  TimeString[3] = char(48 + int((Time - 60 * int(Time / 60)) / 10));
  TimeString[4] = char(48 + Time - 60 * int(Time / 60) -
                       10 * int((Time - 60 * int(Time / 60)) / 10));
  // tft.println(TimeString);
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
  String temp = "";

  temp.concat(TimeDate[4]);
  temp.concat("/");
  temp.concat(TimeDate[5]);
  temp.concat("/");
  temp.concat(TimeDate[6]);
  temp.concat(" ");
  temp.concat(TimeDate[2]);
  temp.concat(":");
  temp.concat(TimeDate[1]);
  temp.concat(":");
  temp.concat(TimeDate[0]);

  return (temp);
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

static uint8_t sk = 0;
/********API to return the assigned TAG value when penup,for the
 * primitives/widgets******/
uint8_t Read_keys() {
  static uint8_t Read_tag = 0, temp_tag = 0, ret_tag = 0;
  Read_tag = FTImpl.Read(REG_TOUCH_TAG);
  ret_tag = 0;
  if (Read_tag != 0 && temp_tag != Read_tag) {
    temp_tag = Read_tag;  // Load the Read tag to temp variable
    sk = Read_tag;
  }
  if (Read_tag == 0) {
    ret_tag = temp_tag;
    temp_tag = 0;
    sk = 0;
  }
  return ret_tag;
}

uint8_t Read_Keypad() {
  static uint8_t Read_tag = 0, temp_tag = 0, touch_detect = 1;  // ret_tag=0,

  Read_tag = FTImpl.Read(REG_TOUCH_TAG);
  // ret_tag = 0;
  if (FTImpl.IsPendown() == 0) touch_detect = 0;
  if (Read_tag != 0)  // Allow if the Key is released
  {
    if (temp_tag != Read_tag && touch_detect == 0) {
      temp_tag = Read_tag;  // Load the Read tag to temp variable
      // FTImpl.PlaySound(100,0x51);
      touch_detect = 1;
    }
  } else {
    if (temp_tag != 0) {
      Flag.Key_Detect = 1;  // global variable
      Read_tag = temp_tag;
    }
    temp_tag = 0;
  }
  return Read_tag;
}

// API used to calculate the width of the character
uint8_t Ft_Gpu_Rom_Font_WH(uint8_t Char, uint8_t font) {
  uint32_t ptr, Wptr;
  uint8_t Width = 0;
  ptr = FTImpl.Read32(0xffffc);
  // Read Width of the character
  Wptr = (ptr + (148 * (font - 16))) + Char;  // (table starts at font 16)
  Width = FTImpl.Read(Wptr);
  return Width;
}

// Notepad buffer
NotepadResult Notepad(const char *initialText) {
  /*local variables*/
  uint8_t Line = 0;
  uint16_t Disp_pos = 0, But_opt;
  uint8_t Read_sfk = 0, tval;
  uint16_t noofchars = 0, line2disp = 0, nextline = 0;
  uint8_t font = 27, offset = 50;
  int32_t tagoption;

  i = 0;
  // Clear then set Linebuffer
  for (tval = 0; tval < MAX_FT_LINES; tval++)
    memset(&Buffer.notepad[tval], '\0',
           sizeof(Buffer.notepad[tval]));  // set all of buffer to be null

  for (i = 0; i < strlen(initialText); i++)  // load in the Project Description
  {
    Buffer.notepad[0][i] = initialText[i];
  }
  noofchars = i;
  i = 0;

  /*intial setup*/
  Line = 0;                  // Starting line
  Disp_pos = LINE_STARTPOS;  // starting pos
  Flag.Numeric = OFF;        // Disable the numbers and special charaters
  memset((Buffer.notepad[Line] + noofchars), '_',
         1);  // For Cursor //noofchars was 0
  Disp_pos +=
      Ft_Gpu_Rom_Font_WH(Buffer.notepad[Line][0], Font);  // Update the Disp_Pos
  noofchars += 1;                                         // for cursor
                                                          /*enter*/
  Flag.Exit = 0;
  Read_sfk = Read_Keypad();  // extra read here to catch unwanted save/quit
  Read_sfk = Read_Keypad();  // extra read here to catch unwanted save/quit

  // Serial.println(Read_sfk);
  // Serial.println(Line);
  // Serial.println(Disp_pos);

  do {
    Read_sfk = Read_Keypad();  // read the keys
    // Serial.print(Read_sfk);
    if (FirstPass) {
      Read_sfk = BACK;
      FirstPass = false;
    }

    if (13 == Read_sfk)  // quit
    {
      FTImpl.DLStart();
      FTImpl.ClearColorRGB(64, 64, 64);
      FTImpl.Clear(1, 1, 1);
      FTImpl.Cmd_Swap();
      setNextScreen(kDisplayScreenHome);
      goto Letsgetoutofhere;
    }

    if (21 == Read_sfk)  // save
    {
      FTImpl.DLStart();
      FTImpl.ClearColorRGB(64, 64, 64);
      FTImpl.Clear(1, 1, 1);
      FTImpl.Cmd_Swap();
      if (Screen == 3) {
        return kNotepadResultSave;
      } else {
        if ((Buffer.notepad[0][0] == '3') && (Buffer.notepad[0][1] == '3') &&
            (Buffer.notepad[0][2] == '3') && (Buffer.notepad[0][3] == '3')) {
          Screen = 5;  // Goto setings screen
        } else {
          setNextScreen(kDisplayScreenHome);
        }
        Loadimage2ram();
        goto Letsgetoutofhere;
      }
    }

    if (Flag.Key_Detect) {  // check if key is pressed
      Flag.Key_Detect = 0;  // clear it
      if (Read_sfk >=
          SPECIAL_FUN) {  // check any special function keys are pressed
        switch (Read_sfk) {
          case BACK_SPACE:
            if (noofchars > 1)  // check in the line there is any characters are
                                // present,cursor not include
            {
              noofchars -= 1;  // clear the character in the buffer
              Disp_pos -=
                  Ft_Gpu_Rom_Font_WH(*(Buffer.notepad[Line] + noofchars - 1),
                                     Font);  // Update the Disp_Pos
            } else {
              if (Line >= (MAX_FT_LINES - 1))
                Line--;
              else
                Line = 0;  // check the FT_LINES
              noofchars =
                  strlen(Buffer.notepad[Line]);  // Read the len of the line
              for (tval = 0; tval < noofchars;
                   tval++)  // Compute the length of the Line
                Disp_pos += Ft_Gpu_Rom_Font_WH(Buffer.notepad[Line][tval],
                                               Font);  // Update the Disp_Pos
            }
            Buffer.temp = (Buffer.notepad[Line] +
                           noofchars);  // load into temporary buffer
            Buffer.temp[-1] = '_';      // update the string
            Buffer.temp[0] = '\0';
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
            noofchars += 1;
            break;
        }
      } else  // it is a standard character
      {
        // String and Line Termination

        Disp_pos += Ft_Gpu_Rom_Font_WH(Read_sfk, Font);  // update dis_pos
        Buffer.temp =
            Buffer.notepad[Line] +
            strlen(Buffer.notepad[Line]);  // load into temporary buffer
        if (strlen(Buffer.notepad[0]) <
            40)  // attempt to limit the character count
        {
          Buffer.temp[-1] = Read_sfk;  // update the string with new character
          Buffer.temp[0] = '_';        // add the cursor
          Buffer.temp[1] = '\0';       // add string terminator
          //}  moved down
          noofchars = strlen(Buffer.notepad[Line]);  // get the string len
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
    }

    // Start the new Display list
    FTImpl.DLStart();
    FTImpl.ClearColorRGB(100, 100, 100);
    FTImpl.Clear(1, 1, 1);
    FTImpl.ColorRGB(255, 255, 255);
    FTImpl.TagMask(1);  // enable tag buffer updation
    FTImpl.Cmd_FGColor(kColourPrimary);
    FTImpl.Cmd_BGColor(0x19354B);
    But_opt = (Read_sfk == BACK)
                  ? FT_OPT_FLAT
                  : 0;  // button color change if the button during press
    FTImpl.Tag(BACK);   // Back    Return to Home
    FTImpl.Cmd_Button((FT_DISPLAYWIDTH * 0.855),
                      (FT_DISPLAYHEIGHT * 0.83 - offset),
                      (FT_DISPLAYWIDTH * 0.146), (FT_DISPLAYHEIGHT * 0.112),
                      font, But_opt, "Clear");
    But_opt = (Read_sfk == BACK_SPACE) ? FT_OPT_FLAT : 0;
    FTImpl.Tag(BACK_SPACE);  // BackSpace
    FTImpl.Cmd_Button((FT_DISPLAYWIDTH * 0.875),
                      (FT_DISPLAYHEIGHT * 0.70 - offset),
                      (FT_DISPLAYWIDTH * 0.125), (FT_DISPLAYHEIGHT * 0.112),
                      font, But_opt, "<-");
    But_opt = (Read_sfk == ' ') ? FT_OPT_FLAT : 0;
    FTImpl.Tag(' ');  // Space
    FTImpl.Cmd_Button((FT_DISPLAYWIDTH * 0.115),
                      (FT_DISPLAYHEIGHT * 0.83 - offset),
                      (FT_DISPLAYWIDTH * 0.73), (FT_DISPLAYHEIGHT * 0.112),
                      font, But_opt, "Space");

    if (Flag.Numeric == OFF) {
      FTImpl.Cmd_Keys(0, (FT_DISPLAYHEIGHT * 0.442 - offset), FT_DISPLAYWIDTH,
                      (FT_DISPLAYHEIGHT * 0.112), font, Read_sfk,
                      (Flag.Caps == ON ? "QWERTYUIOP" : "qwertyuiop"));
      FTImpl.Cmd_Keys(
          (FT_DISPLAYWIDTH * 0.042), (FT_DISPLAYHEIGHT * 0.57 - offset),
          (FT_DISPLAYWIDTH * 0.96), (FT_DISPLAYHEIGHT * 0.112), font, Read_sfk,
          (Flag.Caps == ON ? "ASDFGHJKL" : "asdfghjkl"));
      FTImpl.Cmd_Keys(
          (FT_DISPLAYWIDTH * 0.125), (FT_DISPLAYHEIGHT * 0.70 - offset),
          (FT_DISPLAYWIDTH * 0.73), (FT_DISPLAYHEIGHT * 0.112), font, Read_sfk,
          (Flag.Caps == ON ? "ZXCVBNM" : "zxcvbnm"));

      But_opt = (Read_sfk == CAPS_LOCK) ? FT_OPT_FLAT : 0;
      FTImpl.Tag(CAPS_LOCK);  // Capslock
      FTImpl.Cmd_Button(0, (FT_DISPLAYHEIGHT * 0.70 - offset),
                        (FT_DISPLAYWIDTH * 0.10), (FT_DISPLAYHEIGHT * 0.112),
                        font, But_opt, "a^");
      But_opt = (Read_sfk == NUMBER_LOCK) ? FT_OPT_FLAT : 0;
      FTImpl.Tag(NUMBER_LOCK);  // Numberlock
      FTImpl.Cmd_Button(0, (FT_DISPLAYHEIGHT * 0.83 - offset),
                        (FT_DISPLAYWIDTH * 0.10), (FT_DISPLAYHEIGHT * 0.112),
                        font, But_opt, "12*");
    }
    if (Flag.Numeric == ON) {
      FTImpl.Cmd_Keys(0, (FT_DISPLAYHEIGHT * 0.442 - offset), FT_DISPLAYWIDTH,
                      (FT_DISPLAYHEIGHT * 0.112), font, Read_sfk, "1234567890");
      FTImpl.Cmd_Keys((FT_DISPLAYWIDTH * 0.042),
                      (FT_DISPLAYHEIGHT * 0.57 - offset),
                      (FT_DISPLAYWIDTH * 0.96), (FT_DISPLAYHEIGHT * 0.112),
                      font, Read_sfk, "-@#$%^&*(");
      FTImpl.Cmd_Keys((FT_DISPLAYWIDTH * 0.125),
                      (FT_DISPLAYHEIGHT * 0.70 - offset),
                      (FT_DISPLAYWIDTH * 0.73), (FT_DISPLAYHEIGHT * 0.112),
                      font, Read_sfk, ")_+[]{}");
      But_opt = (Read_sfk == NUMBER_LOCK) ? FT_OPT_FLAT : 0;
      FTImpl.Tag(253);  // Numberlock
      FTImpl.Cmd_Button(0, (FT_DISPLAYHEIGHT * 0.83 - offset),
                        (FT_DISPLAYWIDTH * 0.10), (FT_DISPLAYHEIGHT * 0.112),
                        font, But_opt, "AB*");
    }

    // assign tag value 21 to the save button
    tagoption = 0;  // no touch is default 3d effect and touch is flat effect
    if (21 == Read_sfk) tagoption = FT_OPT_FLAT;
    FTImpl.Tag(21);
    if (Screen == 3) {
      FTImpl.Cmd_Button(423 - 47, 241 - 19, 94, 38, 26, 0, "Save");
    } else {
      FTImpl.Cmd_Button(423 - 47, 241 - 19, 94, 38, 26, 0, "Enter");
    }

    // assign tag value 13 to the quit button
    tagoption = 0;  // no touch is default 3d effect and touch is flat effect
    if (13 == Read_sfk) tagoption = FT_OPT_FLAT;
    FTImpl.Tag(13);
    FTImpl.Cmd_Button(63 - 47, 241 - 19, 94, 38, 26, 0, "Quit");

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

  } while (1);

Letsgetoutofhere:
  delay(10);
}
