// "Стенд".

// Библиотека для работы с портами ввода-вывода.
#include <CyberLib.h>
// Библиотека для работы с сервоприводом.
#include <ServoTimer2.h>
// Библиотека для работы с шаговыми двигателями.
#include <FastAccelStepper.h>
#include <AVRStepperPins.h>

// Структура данных, передаваемых на пульт.
struct Response {
  byte NoRun;       // режим РАБОТА             0 - НЕТ / 1 - ДА
  byte EndZero;     // режим движение к "0"     0 - НЕТ / 1 - ДА
  byte SQZ;         // концевой 0
  byte AlarmStend;  // режим авария на стенде  0 - НЕТ / 1 - ДА
  byte Link;        // режим LINK               0 - НЕТ / 1 - ДА
  byte Return;      // cчетчик для контроля за связью
  byte crc;         // контрольная сумма
};
Response response;

// Структура данных, принимаемая с пульта.
struct Request {
  word VRx;    // переменник джойстика X
  word VRy;    // переменник джойстика Y
  word VR;     // переменник SERVO
  byte Run;    // режим РАБОТА  0 - НЕТ / 1 - ДА
  byte Zero;   // режим движ к "0"  0- НЕТ / 1 - ДА
  byte Alarm;  // режим АВАРИЯ  0 - НЕТ / 1 - ДА
  byte PK;     // режим реле 5 ПК 0 - НЕТ / 1 - ДА
  byte P6;     // режим реле 6  0 - НЕТ / 1 - ДА    сброс питания с драйверов шд
  byte Link;   // режим LINK  0 - НЕТ / 1 - ДА
  byte Return; // cчетчик для контроля за связью
  byte crc;    // контрольная сумма
};
Request request;

// Пин для сервопривода поворота камеры.
int servoPin = 5;
// Объект для работы с сервоприводом поворота камеры.
ServoTimer2 Servo1;
// Длительность импульсов управления сервоприводом поворота камеры.
int valServo = 0;
// Предыдущее значение valServo.
int oldvalServo = 0;

// Объекты для управления шаговыми двигателями.
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

// Алиасы максимальной скорости двигателей в Гц.
#define STEPPER1_MAX_SPEED_HZ 800
#define STEPPER2_MAX_SPEED_HZ 8000

// Cкорость в режиме "Переместить в "0" для двигателей в Гц.
#define STEPPER1_MOVE_ZERO_SPEED_HZ STEPPER1_MAX_SPEED_HZ
#define STEPPER2_MOVE_ZERO_SPEED_HZ STEPPER2_MAX_SPEED_HZ

// Задать алиасы для пинов реле.
// Реле Зеленый свет спереди.
#define ReleGreenFront_OUT D14_Out // А0
#define ReleGreenFront_HI  D14_High
#define ReleGreenFront_ON  D14_High
#define ReleGreenFront_OFF D14_Low

// Реле Зеленый свет сзади.
#define ReleGreenBack_OUT  D15_Out // А1
#define ReleGreenBack_HI   D15_High
#define ReleGreenBack_ON   D15_High
#define ReleGreenBack_OFF  D15_Low

// Реле Белый свет.
#define ReleWhite_OUT      D16_Out // А2
#define ReleWhite_HI       D16_High
#define ReleWhite_ON       D16_High
#define ReleWhite_OFF      D16_Low

// Реле Красный свет.
#define ReleAlarm_OUT      D17_Out // А3
#define ReleAlarm_HIGH     D17_High
#define ReleAlarm_ON       D17_Low
#define ReleAlarm_OFF      D17_High

// Реле включения ПК.
#define RelePK_OUT         D18_Out // А4
#define RelePK_HI          D18_High
#define RelePK_LO          D18_Low

// Реле сброса драйверов двигателей.
#define ReleResetDrv_OUT   D19_Out // А5
#define ReleResetDrv_HI    D19_High
#define ReleResetDrv_LO    D19_Low

// Задать алиася для пина "Индикатор связи".
#define LedLink_Out D4_Out
#define LedLink_HI  D4_High
#define LedLink_LO  D4_Low
#define LedLink_Inv D4_Inv

// Пины для шагового двигателя 1 (вертикальное перемещение камеры).
// Пин импульсов перемещения. stepPinStepperA = 9.
#define stepPinStepper1 stepPinStepperA
// Пин направления перемещения.
#define dirPinStepper1 8
// Пин "Тревога".
#define Alm1_In     D7_In
#define Alm1_Read   D7_Read
#define Alm1_HI     D7_High

// Пины для шагового двигателя 2 (вращение платформы).
// Пин направления перемещения.
#define dirPinStepper2 11
// Пин импульсов перемещения. stepPinStepperB = 10.
#define stepPinStepper2 stepPinStepperB
// Пин "Тревога".
#define Alm2_In     D12_In
#define Alm2_Read   D12_Read
#define Alm2_HI     D12_High

// Пин "Крайнее верхнее положение камеры". SQ1
#define SQUp_In     D6_In
#define SQUp_Read   D6_Read
#define SQUp_HI     D6_High

// Пин "Крайнее нижнее положение камеры". SQ2
#define SQDown_In   D3_In
#define SQDown_Read D3_Read
#define SQDown_HI   D3_High

// Пин "Нулевое положение платформы". SQ3
#define SQZero_In   D2_In
#define SQZero_Read D2_Read
#define SQZero_HI   D2_High

// Концевой выключатель включен.
#define SQ_ON       1
// Концевой выключатель выключен.
#define SQ_OFF      0

byte RunStateStend = 0;   // статус Работа 0-нет/1-ДА
byte MoveToZeroState = 0; // статус Установка в нулевое положение 0-нет/1-ДА
byte EndZeroState = 0;    // статус Установка в нулевое положение завершена 0-нет/1-ДА
byte AlarmBtnState = 0;   // статус Аварийная остановка 0-нет/1-ДА

// Счетчик итераций отсутствия команд от пульта.
byte counter = 0;

// Признак необходимости отправить ответ на пульт.
boolean needSendResponse = false;
// Значение счетчика отправленных запросов с пульта.
byte requestReturn;
// Признак наличия связи с пультом.
boolean fLink = true;
boolean fRelePK = false;   // флаг реле ПК
boolean AlarmDRV = false;  // флаг авария на ШД

// Переменники джойстика
word RezistX, RezistY = 0;
// Переменная вычисления требуемой скорости двигателя.
word stepperSpeed;

// Время, когда был получен последний запрос.
unsigned long lastRequestTime = 0;

void setupRelaysPins()
{
  // Пины реле установить в режим вывода и записать HIGH.
  ReleGreenFront_OUT;
  ReleGreenFront_HI;
  ReleGreenBack_OUT;
  ReleGreenBack_HI;
  ReleWhite_OUT;
  ReleWhite_HI;
  ReleAlarm_OUT;
  ReleAlarm_HIGH;
  RelePK_OUT;
  RelePK_HI;
  ReleResetDrv_OUT;
  ReleResetDrv_HI;
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);
  
  Servo1.attach(servoPin);

  setupRelaysPins();

  // Пин "Связь" установить в режим вывода и записать HIGH.
  LedLink_Out;
  LedLink_HI;

  // Пин "Концевой выключатель "Верх"" установить в режим ввода и подтянуть к Vcc.
  SQUp_In;
  SQUp_HI;

  // Пин "Концевой выключатель "Низ"" установить в режим ввода и подтянуть к Vcc.
  SQDown_In;
  SQDown_HI;

  // Пин "Концевой выключатель "0"" установить в режим ввода и подтянуть к Vcc.
  SQZero_In;
  SQZero_HI;

  // Инициализировать библиотеку для работы с ШД.
  engine.init();
  // Создать объекты для работы с ШД.
  stepper1 = engine.stepperConnectToPin(stepPinStepper1);
  stepper1->setDirectionPin(dirPinStepper1);
  stepper1->setAcceleration(STEPPER1_MAX_SPEED_HZ*2);
  stepper2 = engine.stepperConnectToPin(stepPinStepper2);
  stepper2->setDirectionPin(dirPinStepper2);
  stepper2->setAcceleration(STEPPER2_MAX_SPEED_HZ*2);

  // Пин "Авария" ШД1 установить в режим ввода и подтянуть к Vcc.
  Alm1_In;
  Alm1_HI;
  // Пин "Авария" ШД2 установить в режим ввода и подтянуть к Vcc.
  Alm2_In;
  Alm2_HI;
}

// Вычислить CRC.
byte crc8_bytes(byte *buffer, byte size) {
  byte crc = 0;
  for (byte i = 0; i < size; i++) {
    byte data = buffer[i];
    for (int j = 8; j > 0; j--) {
      crc = ((crc ^ data) & 1) ? (crc >> 1) ^ 0x8C : (crc >> 1);
      data >>= 1;
    }
  }
  return crc;
}

void forceStopSteppers()
{
  stepper1->forceStopAndNewPosition(0);
  stepper2->forceStopAndNewPosition(0);
}

void processControls()
{
  // Если есть изменения в позиции СЕРВО, то крутим ее.
  if (valServo != oldvalServo)
  {
    Servo1.write(valServo);
    oldvalServo = valServo;
  }

  // Управление приводом вертикального перемещения камеры.
  // Если направление вверх.
  if (RezistX > 525)
  {
    // Если в крайнем верхнем положении.
    if (SQUp_Read == SQ_ON)
      stepper1->forceStopAndNewPosition(0);
    else
    {
      // Вычислить скорость.
      stepperSpeed = map(RezistX, 525, 1024, 0, STEPPER1_MAX_SPEED_HZ);

      // Задать скорость и направление.
      stepper1->setSpeedInHz(stepperSpeed);
      stepper1->runForward();
    }

  }
  // Если направление вниз.
  else if (RezistX < 480)
  {
    // Если в крайнем нижнем положении.
    if (SQDown_Read == SQ_ON)
      stepper1->forceStopAndNewPosition(0);
    else
    {
      // Вычислить скорость.
      stepperSpeed = map(RezistX, 480, 0, 0, STEPPER1_MAX_SPEED_HZ);

      // Задать скорость и направление.
      stepper1->setSpeedInHz(stepperSpeed);
      stepper1->runBackward();
    }
  }
  else
    stepper1->stopMove();

  // Управление приводом вращения платформы.
  // Если направление влево.
  if (RezistY < 480)
  {
    // Вычислить скорость.
    stepperSpeed = map(RezistY, 480, 0, 0, STEPPER2_MAX_SPEED_HZ);

    // Задать скорость и направление.
    stepper2->setSpeedInHz(stepperSpeed);
    stepper2->runForward();
  }
  // Если направление вправо.
  else if (RezistY > 525)
  {
    // Вычислить скорость.
    stepperSpeed = map(RezistY, 525, 1024, 0, STEPPER2_MAX_SPEED_HZ);

    // Задать скорость и направление.
    stepper2->setSpeedInHz(stepperSpeed);
    stepper2->runBackward();
  }
  else
    stepper2->stopMove();
}

void processMoveToZero()
{
  // Пока не сработал нижний концевик двигаться вниз.
  if (SQDown_Read == SQ_OFF)
  {
    stepper1->setSpeedInHz(STEPPER1_MOVE_ZERO_SPEED_HZ);
    stepper1->runBackward();
  }
  // Иначе остановиться.
  else
  {
    stepper1->forceStopAndNewPosition(0);
  }

  // Пока не сработал концевик "Нулевое положение" платформы двигаться назад.
  if (SQZero_Read == SQ_OFF)
  {
    stepper2->setSpeedInHz(STEPPER2_MOVE_ZERO_SPEED_HZ);
    stepper2->runBackward();
  }
  // Иначе остановиться.
  else
  {
    stepper2->setSpeedInHz(STEPPER2_MOVE_ZERO_SPEED_HZ / 10);
    stepper2->stopMove();
  }

  // Если сработали оба концевых выключателя, то выключить режим "Установка в 0"
  // и выставить признак того, что стенд находится в нулевом положении.
  if ((SQZero_Read == SQ_ON) and (SQDown_Read == SQ_ON))
  {
    MoveToZeroState = 0;
    EndZeroState = 1;
  }
}

// Управление реле цветных ламп.
void processRelays()
{
  //---- что делаем если АВАРИЯ или нет связи  -----//
  if (AlarmBtnState == 1 or fLink == false or AlarmDRV == true)
  {
    ReleGreenFront_OFF;
    ReleGreenBack_OFF;
    ReleWhite_OFF;
    ReleAlarm_ON;
  }
  else
    if (AlarmBtnState == 0 and fLink == true) // нет АВАРИЯ и есть связь
      ReleAlarm_OFF;


  //------ реж НЕ РАБОТА и нет аварии и есть связь -----//
  if ((RunStateStend == 0) and (AlarmBtnState == 0) and (fLink == true) and (AlarmDRV == false))
  {
    ReleGreenFront_ON;
    ReleGreenBack_ON;
  }


  if (RunStateStend == 1)
  {
    if (AlarmBtnState == 1 or fLink == false or AlarmDRV == true)
    {
      ReleAlarm_ON;
      ReleGreenFront_OFF;
      ReleGreenBack_OFF;
      ReleWhite_OFF;
    }
    else
    {
      ReleAlarm_OFF;
      ReleGreenFront_OFF;
      ReleGreenBack_ON;
      ReleWhite_ON;
    }
  }
  else
  {
    ReleWhite_OFF;
  }
}

void processLinkLed(bool linkExists)
{
  // Если связь с пультом есть
  if (linkExists)
    // То мигает.
    LedLink_Inv;
  else
    // Иначе постоянно горит.
    LedLink_HI;
}

void loop() {
  // Получение команды с пульта.
  if (Serial.readBytes((byte*)&request, sizeof(request)))
  {
    byte CRC = crc8_bytes((byte*)&request, sizeof(request));
    // Если контрольная сумма совпадает.
    if (CRC == 0)
    {
      fLink = true;
      lastRequestTime = millis();

      // для стенда
      RezistX = request.VRx;
      RezistY = request.VRy;
      valServo = map(request.VR, 0, 179, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
      RunStateStend = request.Run;
      MoveToZeroState = request.Zero;

      if (MoveToZeroState == 0)
        EndZeroState = 0;
      AlarmBtnState = request.Alarm;
      // вкл с реле ПК
      if (request.PK == 1)
        RelePK_LO;
      else
        RelePK_HI; // выкл с реле ПК
      // вкл с реле 6
      if (request.P6 == 1)
        ReleResetDrv_LO;
      else
        ReleResetDrv_HI; // выкл с реле 6

      if (AlarmBtnState == 1)
        EndZeroState = 0;
      requestReturn = request.Return;
      needSendResponse = true;
    }
  }
  else
  {
    // Если новой команды нет в течение 750 мс, то сбросить флаг наличия связи с пультом.
    if (millis() - lastRequestTime > 750)
      fLink = false;
  }

  if (needSendResponse == true)
  {
    response.NoRun = 0;
    response.EndZero = EndZeroState;

    // Если стенд находится в нулевом положении, то отправить соответствующий флаг.
    if ((SQZero_Read == SQ_ON) and (SQDown_Read == SQ_ON))
      response.SQZ = 1;
    else
      response.SQZ = 0;

    response.AlarmStend = AlarmDRV;
    response.Return = requestReturn; //счетчик отправлений обратно
    // последний байт - crc. Считаем crc всех байт кроме последнего, то есть кроме самого crc!!! (размер-1)
    response.crc = crc8_bytes((byte*)&response, sizeof(response) - 1);

    Serial.write((byte*)&response, sizeof(response));
    needSendResponse = false;
  }

  // --- сигналы АЛАРМ с драйверов ШД ------//
  if (Alm1_Read == 0 or Alm2_Read == 0)
    AlarmDRV = true;  // флаг сигнала АЛАРМ поднят
  else if (Alm1_Read == 1 and Alm2_Read == 1)
    AlarmDRV = false; // флаг сигнала АЛАРМ сброшен

  processLinkLed(fLink);
  processRelays();

  // Если (или):
  // - на пульте нажали кнопку "Авария";
  // - пришел сигнал тревоги от двигателей;
  // - нет связи с пультом;
  // - стенд выключен;
  // то остановить двигатели.
  if ((AlarmBtnState != 0) or AlarmDRV or !fLink or (RunStateStend == 0))
  {
    forceStopSteppers();
  }
  else
  {
    // Если включен режим "Установка в 0".
    if (MoveToZeroState == 1) {
      // Обработать команду перемещения в 0.
      processMoveToZero();
    }
    // Иначе обычный режим управления джойстиками.
    else
    {
      processControls();
    }
  }
}
