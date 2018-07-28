// *================================================================
//  emulate the Boolean type                               
//  From EE 472 WEB SITE                                   
// *================================================================
enum myBool { FALSE = 0, TRUE = 1 };
typedef enum myBool Bool;

// *================================================================
//  emulate the lock type                                                                  
// *================================================================
enum myLock { OFF = 0, ON = 1 };
typedef enum myLock Lock;

// *================================================================
//  TCB structure                                          
// *================================================================
struct taskSct
{
  void (*myTCB)(void*);
  void* taskDataPtr;
};

typedef struct taskSct TCB;

struct arrivingTrainDatax
{
  unsigned short* trainArriving;
  unsigned short* trainPresent;
  Bool* checkTrain;
  unsigned short* arrivalTime;
  unsigned short* arrivalDirection;
  unsigned int* globalCount;
  unsigned short* departureDirection;
  unsigned short* arrivingTrainDistance;
  Bool* departingTrainFlag;
  
  
};
typedef struct arrivingTrainDatax arrivingTrainData;

struct trainComDatax
{
  unsigned short* trainArriving;
  Bool* departingTrainFlag;
  unsigned short* trainSize;
  unsigned short* arrivalDirection;
  unsigned short* departureDirection;
  Bool* checkTrain;
  unsigned int* globalCount;
  unsigned short* trainDeparting;
  unsigned short* trainPresent; 
  unsigned short* traversalTime;
};
typedef struct trainComDatax trainComData;

struct departingTrainDatax
{
  Bool* departingTrainFlag;
  unsigned short* departureDirection;
  unsigned int* globalCount;
  unsigned short* trainDeparting;
  unsigned short* trainPresent; 
};
typedef struct departingTrainDatax departingTrainData;

struct switchControlDatax
{
  unsigned short* departureDirection;
  Lock* interLock; //intersection lock 
  Bool* gridlock;
  Bool* departingTrainFlag;
  unsigned short* trainSize;
  unsigned short* traversalTime;
  unsigned int* globalCount;
  unsigned int* temperatureBuf;
  unsigned short* trainDeparting;
  unsigned short* trainPresent; 
  
};
typedef struct switchControlDatax switchControlData;

struct lcdDisplayDatax
{
  Lock* interLock; //intersection lock
  Bool* departingTrainFlag;
  unsigned short* trainSize;
  unsigned short* traversalTime;
  unsigned short* departureDirection;
  unsigned int* globalCount;
  unsigned short* trainPresent; 
  Bool* gridlock;
};
typedef struct lcdDisplayDatax lcdDisplayData;

struct oledDisplayDatax
{
  Bool* departingTrainFlag;
  unsigned short* trainArriving;
  unsigned short* trainSize;
  unsigned short* traversalTime;
  unsigned short* arrivalDirection;
  Lock* interLock;  //intersection lock 
  unsigned int* globalCount;
  unsigned short* departureDirection;
  unsigned short* mode;
  unsigned short* scroll;
  unsigned short* select;
  unsigned short* statusSelection;
  Bool* gridlock;
  unsigned int* temperatureBuf;
  unsigned short* arrivingTrainDistance;
  unsigned short* trainDeparting;
  unsigned short* trainPresent; 
  
};
typedef struct oledDisplayDatax oledDisplayData;

struct localKeypadDatax
{
  unsigned short* mode;
  unsigned short* statusSelection;
  unsigned short* scroll;
  unsigned short* select;
  unsigned short* annunciation;
  unsigned int* globalCount;
};
typedef struct localKeypadDatax localKeypadData;

struct serialComDatax
{
  Lock* interLock;  //intersection lock
  Bool* departingTrainFlag;
  unsigned short* trainSize;
  unsigned short* traversalTime;
  unsigned short* departureDirection;
  unsigned short* arrivalDirection;
  unsigned short* trainArriving;
  unsigned short* arrivalTime;
  Bool* gridlock;
  unsigned int* globalCount;    
  unsigned int* temperatureBuf;
  unsigned short* trainDeparting;
  unsigned short* trainPresent; 
  unsigned int* noiseTransBuf;
  
};
typedef struct serialComDatax serialComData;

struct temperatureDatax
{
  Bool* departingTrainFlag;
  unsigned int* temperatureBuf;
  Bool* globalStartMeasure;
  
};
typedef struct temperatureDatax temperatureData;

struct noiseCaptureDatax
{
  signed int* noiseCaptureBuf;
  Bool* globalStartMeasure;
};
typedef struct noiseCaptureDatax noiseCaptureData;

struct noiseProcessingDatax
{
  signed int* noiseCaptureBuf;
  unsigned int* noiseTransBuf;  
  int* noiseFrequency;
};
typedef struct noiseProcessingDatax noiseProcessingData;

struct remoteCommunicationDatax
{  
  Bool* departingTrainFlag;
  unsigned short* trainSize;
  unsigned short* traversalTime;
  unsigned short* departureDirection;
  unsigned short* arrivalDirection;
  unsigned short* trainArriving;
  unsigned short* arrivalTime;
  Bool* gridlock;  
  unsigned int* temperatureBuf;
  unsigned short* trainPresent;
  unsigned short* arrivingTrainDistance;
  unsigned short* noiseTransBuf;
  unsigned short* showRecentData;
};
typedef struct remoteCommunicationDatax remoteCommunicationData;

struct commandDatax
{
  char* readCommand;
  Bool* globalStartMeasure;
  unsigned short* showRecentData;
};
typedef struct commandDatax commandData;
// ++++++++++++++++++++++++++++++++++++++++++++++++++
// initialize variables
// ++++++++++++++++++++++++++++++++++++++++++++++++++
//unsigned portBASE_TYPE task1;
//unsigned portBASE_TYPE task2;
//unsigned portBASE_TYPE task3;
//unsigned portBASE_TYPE task4;
//unsigned portBASE_TYPE task5;
//unsigned portBASE_TYPE task6;
//unsigned portBASE_TYPE task7;
//unsigned portBASE_TYPE task8;
//unsigned portBASE_TYPE task9;
//unsigned portBASE_TYPE task10;
//unsigned portBASE_TYPE task11;

Bool burstFail = TRUE;
unsigned short showRecentData = 0;
extern char readCommand[32];
Bool globalStartMeasure = FALSE;
Bool globalStartDisplay = TRUE;
int commandAccept = 0;
int noiseFrequency = 0;
Bool execute = FALSE;

signed int noiseCaptureBuf[256]; 
int noiseTransBuf[16];

Bool captureNoise = FALSE;
Bool processNoise = FALSE;

unsigned short test11 = 1;
Bool addingTrain = FALSE; //debounce sw2
Bool measuringTemp = FALSE;  //debouce sw3
Bool measure = FALSE;

Bool tcGo = FALSE;
Bool startSerial = FALSE;
Bool RunMinor = TRUE;

// command and control - direction
unsigned short arrivalDirection = 4;
unsigned short departureDirection = 4;

// command and control - annunciation
Bool checkTrain = FALSE;

// Status and annunciation management
Bool departingTrainFlag = FALSE;
unsigned short trainSize = 0;
unsigned short traversalTime = 0;
unsigned short arrivalTime = 0;
unsigned short trainArriving = 0; 
unsigned short arrivingWhistle = 0;
unsigned short departingWhistle = 0;

// Keypad
Bool OLEDChanged = TRUE;
unsigned short mode = 0;    //mode select = 0, train status = 1, annunciation = 2 
unsigned short statusSelection = 0;
unsigned short scroll = 0;
unsigned short select = 0;
unsigned short annunciation = 0;
unsigned short key0 = 1;                          //home--A4
unsigned short key1 = 1;                          //scroll--D4    
unsigned short key2 = 1;                          //select--D5
// Alarm
Bool gridlock = FALSE; 
unsigned int globalCount = 0;

//new
unsigned short arrivingTrainDistance[8];

unsigned long temperatureBuf[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Bool wheelTemperature = FALSE;

Lock interLock = OFF;   //intersection lock
xSemaphoreHandle intersectionLock;
unsigned int remainDistance = 1000;

// ADC
unsigned long ulValue3;

unsigned long ulValue0;
unsigned long ulValue1;
unsigned long ulValue2;
unsigned short trainDeparting[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //train departing queue
unsigned short trainPresent = 0;

arrivingTrainData myArrivingTrain;
trainComData myTrainCom;
departingTrainData myDepartingTrain;
switchControlData mySwitchControl;
lcdDisplayData myLcdDisplay;
oledDisplayData myOledDisplay;
localKeypadData myLocalKeypad;
serialComData mySerialCom;
temperatureData myTemperature;
noiseCaptureData myNoiseCapture;
noiseProcessingData myNoiseProcessing;
remoteCommunicationData myRemote;
commandData myCommand;

TCB arrivingTD;
TCB trainCD;
TCB departingTD;
TCB SWCD;
TCB LCDD;
TCB OLEDD;
TCB keypadD;
TCB serialCD;
TCB temperatureD;
TCB noiseCD;
TCB noisePD;
TCB ramoteCD;
TCB commandD;
// *================================================================
//  define methods                                         
// *================================================================

int randomInteger(int low, int high); 

/*
void arrivingTrain(void* train);
void trainCom(void* train);
void departingTrain(void* train);
void switchControl(void* train);
void LCDDisplay(void* train);
void OLEDDisplay(void* train);
void localKeypad(void* train);
void serialCom(void* train);
void temperatureMeasurement(void* train);
*/

void lcdSR();
void lcdClear();
void printOn();
void printOff();
void printSize();
void printTime();
void printNorth();
void printSouth();
void printWest();
void printEast();
void printSpace(); 
void printNumber(short number);
void IntTimer0(void);
void IntGPIOf(void);
void insert(TCB* node); 
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount);
void delay(unsigned long aValue);
void startUp();
void initialize();
void deleteFunction(TCB* node);
void modeSelect();
void trainStatus();
void scrollNselect();
void printGridLock();
void printinterLock();
signed int optfft(signed int real[256], signed int imag[256]);

void printO();
void printX();
void printBurst();
void printNuclear();


# define TIME_BASE 3846         // define 3846 delay in system as 0.5s delay
