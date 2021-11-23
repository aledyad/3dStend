// "Стенд".

// Библиотека для работы с портами ввода-вывода.
#include <CyberLib.h>
// Библиотека для работы с сервоприводом.
#include <ServoTimer2.h>
// Библиотека для работы с шаговыми двигателями.
#include <FastAccelStepper.h>
#include <AVRStepperPins.h>

// Структура данных, передаваемых на пульт.
struct StrOtv {
  byte NoRun;       // режим РАБОТА             0 - НЕТ / 1 - ДА
  byte EndZero;     // режим движение к "0"     0 - НЕТ / 1 - ДА
  byte SQZ;         // концевой 0
  byte AlarmStend;  // режим ававрия на стенде  0 - НЕТ / 1 - ДА
  byte Link;        // режим LINK               0 - НЕТ / 1 - ДА
  byte Return;      // cчетчик для контроля за связью
  byte crc;         // контрольная сумма
};

// Структура данных, принимаемая с пульта.
struct Str {
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
Str buf;

// Пин для сервопривода поворота камеры.
int servoPin = 5;
// Объект для работы с сервоприводом поворота камеры.
ServoTimer2 Servo1;
// Угол отклонения сервопривода поворота камеры.
int valServo;
// Предыдущее значение отклонения сервопривода поворота камеры.
int oldvalServo;

// Объекты для управления шаговыми двигателями.
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

// Алиасы максимальной скорости двигателей в Гц.
#define STEPPER1_MAX_SPEED_HZ 200
#define STEPPER2_MAX_SPEED_HZ 1000

// Cкорость в режиме "Переместить в "0" для двигателей в Гц.
#define STEPPER1_MOVE_ZERO_SPEED_HZ 100
#define STEPPER2_MOVE_ZERO_SPEED_HZ 500

// Задать алиасы для пинов реле.
#define Rele1_Out  D14_Out   // А0
#define Rele1_HI   D14_High
#define Rele1_LO   D14_Low

#define Rele2_Out  D15_Out   // А1
#define Rele2_HI   D15_High
#define Rele2_LO   D15_Low

#define Rele3_Out  D16_Out   // А2
#define Rele3_HI   D16_High
#define Rele3_LO   D16_Low

#define Rele4_Out  D17_Out   // А3
#define Rele4_HI   D17_High
#define Rele4_LO   D17_Low

#define RelePK_Out D18_Out   // А4
#define RelePK_HI  D18_High
#define RelePK_LO  D18_Low

#define Rele6_Out  D19_Out   // А5
#define Rele6_HI   D19_High
#define Rele6_LO   D19_Low

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

byte RunStateStend = 0;   // статус Работа  0-нет/1-ДА
byte MoveToZeroState = 0; // статус Установка в нулевое положение 0-нет/1-ДА
byte EndZeroState = 0;    // статус Установка в нулевое положение завершена 0-нет/1-ДА
byte AlarmBtnState = 0;   // статус Аварийная остановка 0-нет/1-ДА
byte Return = 0;

// Счетчик итераций отсутствия команд от пульта.
byte counter = 0;

// Признак необходимости отправить ответ на пульт. 
boolean needSendResponse = false;
// Признак наличия связи с пультом.
boolean fLink = true;
boolean fRelePK = false;   // флаг реле ПК
boolean AlarmDRV = false;  // флаг авария на ШД

// Переменники джойстика
word RezistX, RezistY = 0;
// Переменная вычисления требуемой скорости двигателя.
word stepperSpeed;

// "Время", когда был отправлен на пульт последний ответ.
unsigned long tSend = 0;
unsigned long tRele = 0;



void setup() {
  Serial.begin(9600);
  Serial.setTimeout(3);
  Servo1.attach(servoPin);

  // Пины реле установить в режим вывода и записать HIGH.
  Rele1_Out;
  Rele1_HI;
  Rele2_Out;
  Rele2_HI;
  Rele3_Out;
  Rele3_HI;
  Rele4_Out;
  Rele4_HI;
  RelePK_Out;
  RelePK_HI;
  Rele6_Out;
  Rele6_HI;

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
  stepper1->setAcceleration(1000);
  stepper2 = engine.stepperConnectToPin(stepPinStepper2);
  stepper2->setDirectionPin(dirPinStepper2);
  stepper2->setAcceleration(200);

  // Пин "Авария" ШД1 установить в режим ввода и подтянуть к Vcc.
  Alm1_In;
  Alm1_HI;
  // Пин "Авария" ШД2 установить в режим ввода и подтянуть к Vcc.
  Alm2_In;
  Alm2_HI;

  tSend, tRele = millis();
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

void loop() {

  // Получение команды с пульта.
  if (Serial.readBytes((byte*)&buf, sizeof(buf))) {
    byte CRC = crc8_bytes((byte*)&buf, sizeof(buf));
    // Если контрольная сумма совпадает.
    if (CRC == 0) {
      counter = 0;

      // Выставить флаг "Связь".
      fLink = true;
      if (buf.Link == 1)
        LedLink_Inv; //мигаем LED  LINK  когда обмен данными
      else
        LedLink_HI; // LED  LINK  просто горит когда нет обмена данными

      // для стенда
      RezistX = buf.VRx;
      RezistY = buf.VRy;
      valServo = buf.VR;
      RunStateStend = buf.Run;
      MoveToZeroState = buf.Zero;

      if (MoveToZeroState == 0)
        EndZeroState = 0;
      AlarmBtnState = buf.Alarm;
      // вкл с реле ПК
      if (buf.PK == 1)
        RelePK_LO;
      else
        RelePK_HI; // выкл с реле ПК
      // вкл с реле 6
      if (buf.P6 == 1)
        Rele6_LO;
      else
        Rele6_HI; // выкл с реле 6

      Return = buf.Return;
      if (AlarmBtnState == 1)
        EndZeroState = 0;
      needSendResponse = true;
      tSend = millis();
    }
  } else {
    //  если нет обмена данными вкл.счетчик
    counter++;
    if (counter > 250 )
      // Выставить флаг отсутствия связи с пультом.
      fLink = false;
  }

  //--------- ОТПРАВКА ОБРАТНЫХ СООБЩЕНИЙ НА ПУЛЬТ ----------//
  if (needSendResponse == true)
    if (millis() > tSend + 10) // через 10 мсек
    { 
      // буфер на отправку
      StrOtv bufOtv;

      bufOtv.NoRun = 0;
      bufOtv.EndZero = EndZeroState;

      if (SQZero_Read == 1 and SQDown_Read == 1)
        bufOtv.SQZ = 1;
      else
        bufOtv.SQZ = 0;

      bufOtv.AlarmStend = AlarmDRV;
      bufOtv.Return = buf.Return; //счетчик отправлений обратно
      // последний байт - crc. Считаем crc всех байт кроме последнего, то есть кроме самого crc!!! (размер-1)
      bufOtv.crc = crc8_bytes((byte*)&bufOtv, sizeof(bufOtv) - 1);

      Serial.write((byte*)&bufOtv, sizeof(bufOtv));
      needSendResponse = false;
    }
  //---------КОНЕЦ ОТПРАВКА ОБРАТНЫХ СООБЩЕНИЙ НА ПУЛЬТ ----------//


  // --- сигналы АЛАРМ с драйверов ШД ------//
  if (Alm1_Read == 0 or  Alm2_Read == 0)
    AlarmDRV = true;  // флаг сигнала АЛАРМ поднят
  else if (Alm1_Read == 1 and Alm2_Read == 1)
    AlarmDRV = false; // флаг сигнала АЛАРМ сброшен


  //--------- УПРАВЛЯЕМ РЕЛЕ 1-4 ------------------//
  //---- что делаем если АВАРИЯ или нет связи  -----//
  if (AlarmBtnState == 1 or  fLink == false or AlarmDRV == true ) {
    Rele1_HI; // РЕЛЕ 1 ВЫКЛ
    Rele2_HI; // РЕЛЕ 2 ВЫКЛ
    Rele3_HI; // РЕЛЕ 3 ВЫКЛ
    Rele4_LO; // РЕЛЕ 4 ВКЛ
    if ( fLink == false) LedLink_HI; // LED LINK просто горит
  }
  else if (AlarmBtnState == 0 and  fLink == true) // нет АВАРИЯ и есть связь
    Rele4_HI;// РЕЛЕ 4 ВЫКЛ


  //------ реж НЕ РАБОТА и нет аварии и есть связь -----//
  if ((RunStateStend == 0) and (AlarmBtnState == 0) and  fLink == true and AlarmDRV == false) {
    Rele1_LO;// РЕЛЕ 1 ВКЛ
    Rele2_LO;// РЕЛЕ 2 ВКЛ
  }// end if (RunStateStend==0)


  // если реж РАБОТА
  if (RunStateStend == 1) {
    if (AlarmBtnState == 1 or  fLink == false or AlarmDRV == true ) {
      Rele4_LO; // РЕЛЕ 4 ВКЛ
      Rele1_HI;// РЕЛЕ 1 ВЫКЛ
      Rele2_HI;// РЕЛЕ 2 ВКЛ
      Rele3_HI;// РЕЛЕ 3 ВКЛ
    } else { 
      Rele4_HI;// РЕЛЕ 4 ВЫКЛ
      Rele1_HI;// РЕЛЕ 1 ВЫКЛ
      Rele2_LO;// РЕЛЕ 2 ВКЛ
      Rele3_LO;// РЕЛЕ 3 ВКЛ
    }
  } else
    if (RunStateStend == 0) {
      Rele3_HI;  // РЕЛЕ 3 ВЫКЛ
    }
  //---------КОНЕЦ УПРАВЛЯЕМ РЕЛЕ 1-4 ------------------//


  // Если включен режим "Установка в 0".
  if (MoveToZeroState == 1 and RunStateStend == 1) {
    // Пока не сработал нижний концевик двигаться вниз.
    if (SQDown_Read != 1) {
      stepper1->setSpeedInHz(STEPPER1_MOVE_ZERO_SPEED_HZ);
      stepper1->runBackward();
    // Иначе резко остановиться.
    } else {
      stepper1->forceStopAndNewPosition(0);
    }

    // Пока не сработал концевик "Нулевое положение" платформы двигаться назад.
    if (SQZero_Read != 1) {
      stepper2->setSpeedInHz(STEPPER2_MOVE_ZERO_SPEED_HZ);
      stepper2->runBackward();
    // Иначе плавно остановиться.
    } else {
      stepper2->stopMove();
    }

    // Если сработали оба концевых выключателя, то выключить режим "Установка в 0".
    if (SQZero_Read == 1 and SQDown_Read == 1) {
      MoveToZeroState = 0;
      EndZeroState = 1;
    }
  }


  // Режим "Работа от джойстиков".
  if ((RunStateStend == 1) and (MoveToZeroState != 1))
    // Если не нажата кнопка АВАРИЯ и нет аварии от ШД и есть связь.
    if ((AlarmBtnState != 1) and AlarmDRV == false and fLink == true) {
      // Если есть изменения в позиции СЕРВО, то крутим ее.
      if (valServo != oldvalServo) {
        Servo1.write(valServo);
        oldvalServo = valServo;
      }

      // Управление приводом вертикального перемещения камеры.
      // Если не в крайнем верхнем положении.
      if (SQUp_Read != 1) {
        // Если направление вверх.
        if (RezistX > (525)) {
          // Вычислить скорость.
          stepperSpeed = map(RezistX, 525, 1024, 0, STEPPER1_MAX_SPEED_HZ);

          // Задать скорость и направление.
          stepper1->setSpeedInHz(stepperSpeed);
          stepper1->runForward();
        }
      }

      // Если не в крайнем нижнем положении.
      if (SQDown_Read != 1)
      {
        // Если направление вниз.
        if (RezistX < (480)) {
          // Вычислить скорость.
          stepperSpeed = map(RezistX, 0, 480, 0, STEPPER1_MAX_SPEED_HZ);

          // Задать скорость и направление.
          stepper1->setSpeedInHz(stepperSpeed);
          stepper1->runBackward();
        }
      }

      // Управление приводом вращения платформы.
      // Если направление вправо.
      if (RezistY > (525)) {
        // Вычислить скорость.
        stepperSpeed = map(RezistY, 525, 1024, 0, STEPPER2_MAX_SPEED_HZ);

        // Задать скорость и направление.
        stepper2->setSpeedInHz(stepperSpeed);
        stepper2->runForward();
      }

      // Если направление влево.
      if (RezistY < (480)) {
        // Вычислить скорость.
        stepperSpeed = map(RezistY, 0, 480, 0, STEPPER2_MAX_SPEED_HZ);

        // Задать скорость и направление.
        stepper2->setSpeedInHz(stepperSpeed);
        stepper2->runBackward();
      }
    }
}
