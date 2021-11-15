// "Стенд".

// Библиотека для работы с портами ввода-вывода.
#include <CyberLib.h>
// Библиотека для работы с сервоприводом.
#include <ServoTimer2.h>
// Библиотека для работы с шаговыми двигателями.
#include <FastAccelStepper.h>

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

// Настройка управления шаговыми двигателями.
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;

// Пины для шагового двигателя 1 (вертикальное перемещение камеры).
#define dirPinStepper1 11
#define stepPinStepper1 12

// Задать алиасы для пинов реле.
#define Rele1_Out  D14_Out   // А0
#define Rele1_HI   D14_High
#define Rele1_LO   D14_Low
#define Rele1_Read D14_Read

#define Rele2_Out  D15_Out   // А1
#define Rele2_HI   D15_High
#define Rele2_LO   D15_Low
#define Rele2_Read D15_Read

#define Rele3_Out  D16_Out   // А2
#define Rele3_HI   D16_High
#define Rele3_LO   D16_Low
#define Rele3_Read D16_Read

#define Rele4_Out  D17_Out   // А3
#define Rele4_HI   D17_High
#define Rele4_LO   D17_Low
#define Rele4_Read D17_Read

#define RelePK_Out  D18_Out  // А4
#define RelePK_HI   D18_High
#define RelePK_LO   D18_Low
#define RelePK_Read D18_Read

#define Rele6_Out  D19_Out   // А5
#define Rele6_HI   D19_High
#define Rele6_LO   D19_Low
#define Rele6_Read D19_Read

// ----  определяем пины для LED ----//
#define LedLink_Out  D4_Out     //  led связи
#define LedLink_HI   D4_High
#define LedLink_LO   D4_Low
#define LedLink_Read D4_Read
#define  LedLink_Inv D4_Inv

// ----  пины драйверов ШД-1 (горизонтальное перемещение)
// пин STEP
#define Step1_Out  D12_Out
#define Step1_HI   D12_High
#define Step1_LO   D12_Low
#define Step1_Read D12_Read
// пин DIR
#define Dir1_Out  D11_Out
#define Dir1_HI   D11_High
#define Dir1_LO   D11_Low
#define Dir1_Read D11_Read
// пин ALARM
#define Alm1_In   D10_In
#define Alm1_Read D10_Read
#define Alm1_HI   D10_High

// пин концевой ВВЕРХ   SQ1
#define SQUp_In   D9_In
#define SQUp_Read D9_Read
#define SQUp_HI   D9_High

// пин концевой НИЗ   SQ2
#define SQDown_In   D8_In
#define SQDown_Read D8_Read
#define SQDown_HI   D8_High

//------------- пины драйверов ШД-2 (вертикальное перемещение)
// пин STEP
#define Step2_Out  D7_Out
#define Step2_HI   D7_High
#define Step2_LO   D7_Low
#define Step2_Read D7_Read

// пин DIR
#define Dir2_Out  D6_Out
#define Dir2_HI   D6_High
#define Dir2_LO   D6_Low
#define Dir2_Read D6_Read

// пин ALARM
#define Alm2_In   D3_In
#define Alm2_Read D3_Read
#define Alm2_HI   D3_High

// пин концевой SQ3
#define SQZero_In   D2_In
#define SQZero_Read D2_Read
#define SQZero_HI   D2_High


byte RunStateStend = 0;  // статус Работа  0-нет/1-ДА
byte NoRunState = 0;     // статус Работа  0-нет/1-ДА
byte ZeroStateStend = 0; // статус уст в "0"  0-нет/1-ДА
byte EndZeroState = 0;   // статус уст в "0"  0-нет/1-ДА
byte AlarmBtnState = 0;  // статус Авария в "0" 0-нет/1-ДА
byte Return = 0;

// byte stepState1 = 0;
// Время, когда был отправлен последний сигнал шага ШД1.
unsigned long previousMillisD1 = 0;

// byte stepState2 = 0;
// Время, когда был отправлен последний сигнал шага ШД2.
unsigned long previousMillisD2 = 0;

byte counter = 0; // счетчик нет связи

// Признак необходимости отправить ответ на пульт. 
boolean SendOtvet = false;
// Флаг LINK наличия связи с пультом.
boolean fLink = true;
boolean fRelePK = false;   // флаг реле ПК
boolean AlarmDRV = false;  // флаг авария на ШД


// переменники джойстика
word RezistX, RezistY = 0;
unsigned long MaxX = 100000;
word MinX = 1;
unsigned long MaxY = 100000;
word MinY = 1;
unsigned long interval1, interval2 = 0;
// Cкорость в режиме "Переместить в "0"" для  ШД-1.
word speedZero_1 = 50000;
// Cкорость в режиме "Переместить в "0"" для  ШД-2.
word speedZero_2 = 50000;
// Задержка в мкс на формирование импульса STEP управления ШД.
const byte delPuls = 10;

unsigned long currentMillis1, currentMillis2, timerZero = 0;
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

  engine.init();

  // Создать объект для работы с ШД1.
  stepper1 = engine.stepperConnectToPin(stepPinStepper1);
  // Пин "Авария" ШД1 установить в режим ввода и подтянуть к Vcc.
  Alm1_In;
  Alm1_HI;
  
  // ШД2.
  // Пин "Направление ШД2" установить в режим вывода и записать LO.
  Dir2_Out;
  Dir2_LO;
  // Пин "Шаг ШД2" установить в режим вывода и записать LO.
  Step2_Out;
  Step2_LO;
  // ?
  //pinMode(En2, OUTPUT);   digitalWrite(En2, LOW);         // пин ENA
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
  /* if (millis() >= (tRele + 10000)and fRelePK==false) // после вклчючения ждем 10 сек
    {RelePK_LO;   // вкл с реле ПК
    delay(1000); // через 1 c
    RelePK_HI;   // выкл с реле ПК
    fRelePK=true;
    }
  */

  if (Serial.readBytes((byte*)&buf, sizeof(buf)))
  {
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
      ZeroStateStend = buf.Zero;

      if (ZeroStateStend == 0)
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
      if (AlarmBtnState == 1) EndZeroState = 0;
      SendOtvet = true;
      tSend = millis();
    }
  }
  else
  { //  если нет обмена данными вкл.счетчик
    counter++;
    if (counter > 250 )
      // Выставить флаг отсутствия связи с пультом.
      fLink = false;
  }

  //--------- ОТПРАВКА ОБРАТНЫХ СООБЩЕНИЙ НА ПУЛЬТ ----------//
  if (SendOtvet == true)
    if (millis() > tSend + 10) // через 10 мсек
    { 
      // буфер на отправку
      StrOtv bufOtv;

      bufOtv.NoRun = NoRunState;
      bufOtv.EndZero = EndZeroState;

      if (SQZero_Read == 1 and SQDown_Read == 1)       // было  (SQZero_Read == 0 and SQDown_Read == 0)
        bufOtv.SQZ = 1;
      else
        bufOtv.SQZ = 0;

      bufOtv.AlarmStend = AlarmDRV;
      bufOtv.Return = buf.Return; //счетчик отправлений обратно
      // последний байт - crc. Считаем crc всех байт кроме последнего, то есть кроме самого crc!!! (размер-1)
      bufOtv.crc = crc8_bytes((byte*)&bufOtv, sizeof(bufOtv) - 1);

      //      mySerial.write((byte*)&bufOtv, sizeof(bufOtv));
      Serial.write((byte*)&bufOtv, sizeof(bufOtv));
      //tSend=millis();
      SendOtvet = false;
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
    if (AlarmBtnState == 1 or  fLink == false or AlarmDRV == true )
    { Rele4_LO; // РЕЛЕ 4 ВКЛ
      Rele1_HI;// РЕЛЕ 1 ВЫКЛ
      Rele2_HI;// РЕЛЕ 2 ВКЛ
      Rele3_HI;// РЕЛЕ 3 ВКЛ
    }
    else
    { Rele4_HI;// РЕЛЕ 4 ВЫКЛ

      Rele1_HI;// РЕЛЕ 1 ВЫКЛ
      Rele2_LO;// РЕЛЕ 2 ВКЛ
      Rele3_LO;// РЕЛЕ 3 ВКЛ
    }
  }
  else if (RunStateStend == 0) {
    Rele3_HI;  // РЕЛЕ 3 ВЫКЛ
  }

  //---------КОНЕЦ УПРАВЛЯЕМ РЕЛЕ 1-4 ------------------//



  // Если включен режим "Установка в 0".
  if (ZeroStateStend == 1 and RunStateStend == 1) {
    //движение ШД-1 к нижнему концевому
    if (SQDown_Read != 1)        // было  (SQDown_Read != 0)
    {
      //Serial.println("---- движение ШД-1 к нижнему концевому ----- ");
      currentMillis1 = micros();

      Dir1_LO; // направление движение
      // с постоянной скоростью движения
      if (currentMillis1 - previousMillisD1 >= speedZero_1) {
        previousMillisD1 = currentMillis1;
        Step1_HI;    // высокий уровень пина
        delayMicroseconds(delPuls);  // ждём X мкс
        Step1_LO;    // низкий уровень пина

        delayMicroseconds(delPuls);  // ждём X мкс
        Step1_HI;    // высокий уровень пина
        delayMicroseconds(delPuls);  // ждём X мкс
        Step1_LO;    // низкий уровень пина

        delayMicroseconds(delPuls);  // ждём X мкс
        Step1_HI;    // высокий уровень пина
        delayMicroseconds(delPuls);  // ждём X мкс
        Step1_LO;    // низкий уровень пина
      }
    }

    //--------движение ШД-2 к концевому  "0" ----------//
    if (SQZero_Read != 1) {
      currentMillis2 = micros();

      Dir2_LO; // направление движение           // было  Dir2_HI;
      // с постоянной скоростью движения
      if (currentMillis2 - previousMillisD2 >= speedZero_2) {
        previousMillisD2 = currentMillis2;

        Step2_HI;    // высокий уровень пина
        delayMicroseconds(delPuls);  // ждём X мкс
        Step2_LO;    // низкий уровень пина

        delayMicroseconds(delPuls);  // ждём X мкс
        Step2_HI;    // высокий уровень пина
        delayMicroseconds(delPuls);  // ждём X мкс
        Step2_LO;    // низкий уровень пина

        delayMicroseconds(delPuls);  // ждём X мкс
        Step2_HI;    // высокий уровень пина
        delayMicroseconds(delPuls);  // ждём X мкс
        Step2_LO;    // низкий уровень пина


        /*    // !было  раньше
          if (stepState2 == LOW) {stepState2 = HIGH; Step2_HI; }
          else
          { stepState2 = LOW; Step2_LO;  }
        */
      }// end if (currentMillis2

    }// end if (digitalRead(SQZero)!=1

    // когда оба КВ сработают
    if (SQZero_Read == 1 and SQDown_Read == 1)  {      // было  (SQZero_Read == 0 and SQDown_Read == 0)
      ZeroStateStend = 0;
      EndZeroState = 1;
    }

  } // end if (ZeroState==1)

  //------ конец режима установка в "0" ------//



  //------ включен режим РАБОТА  движение от джойстиков ------//
  if ((RunStateStend == 1) and (ZeroStateStend != 1))
    // Если не нажата кнопка АВАРИЯ и нет аварии от ШД и есть связь.
    if ((AlarmBtnState != 1) and AlarmDRV == false and fLink == true) {
      NoRunState = 0;

      // если есть изменения в позиции СЕРВО то крутим ее
      if ((valServo != oldvalServo))
        Servo1.write(valServo);
      oldvalServo = valServo;

      // ----- РАБОТА  ШД 1  ------------
      if (RezistX < 525 and RezistX > 480)
        Step1_LO;

      // Если не в крайнем верхнем положении.
      if (SQUp_Read != 1)
      {
        // Если направление вверх.
        if (RezistX > (525)) {
          currentMillis1 = micros();

          // Вычислить интервал, спустя который можно посылать повторный импульс.
          // Чем ближе к нулю, тем больше интервал.
          interval1 = map(RezistX, 525, 1023, MaxX, MinX);

          // Направление движения вверх.
          Dir1_HI;

          if (currentMillis1 - previousMillisD1 >= interval1) {
            previousMillisD1 = currentMillis1;

            Step1_HI;    // высокий уровень пина
            delayMicroseconds(delPuls);  // ждём X мкс
            Step1_LO;    // низкий уровень пина


            delayMicroseconds(delPuls);  // ждём X мкс
            Step1_HI;    // высокий уровень пина
            delayMicroseconds(delPuls);  // ждём X мкс
            Step1_LO;    // низкий уровень пина

            delayMicroseconds(delPuls);  // ждём X мкс
            Step1_HI;    // высокий уровень пина
            delayMicroseconds(delPuls);  // ждём X мкс
            Step1_LO;    // низкий уровень пина
          }
        }
      }

      // Если не в крайнем нижнем положении.
      if (SQDown_Read != 1)
      {
        // Если направление вниз.
        if (RezistX < (480)) {
          interval1 = map(RezistX, 0, 480, MinX, MaxX);
          currentMillis1 = micros();
          // Направление движения вниз.
          Dir1_LO;

          if (currentMillis1 - previousMillisD1 >= interval1) {
            previousMillisD1 = currentMillis1;

            Step1_HI;    // высокий уровень пина
            delayMicroseconds(delPuls);  // ждём X мкс
            Step1_LO;    // низкий уровень пина


            delayMicroseconds(delPuls);  // ждём X мкс
            Step1_HI;    // высокий уровень пина
            delayMicroseconds(delPuls);  // ждём X мкс
            Step1_LO;    // низкий уровень пина

            delayMicroseconds(delPuls);  // ждём X мкс
            Step1_HI;    // высокий уровень пина
            delayMicroseconds(delPuls);  // ждём X мкс
            Step1_LO;    // низкий уровень пина
          }
        }
      }


      // ----- РАБОТА  ШД 2  ------------
      if (RezistY < 525 and RezistY > 480)
        Step2_LO;

      // Движение вправо.
      if (RezistY > (525)) {
        currentMillis2 = micros();
        interval2 = map(RezistY, 525, 1023, MaxY, MinY);

        // Направление движения вправо.
        Dir2_LO;

        if (currentMillis2 - previousMillisD2 >= interval2) {
          previousMillisD2 = currentMillis2;
          Step2_HI;    // высокий уровень пина
          delayMicroseconds(delPuls);  // ждём X мкс
          Step2_LO;    // низкий уровень пина

          delayMicroseconds(delPuls);  // ждём X мкс
          Step2_HI;    // высокий уровень пина
          delayMicroseconds(delPuls);  // ждём X мкс
          Step2_LO;    // низкий уровень пина

          delayMicroseconds(delPuls);  // ждём X мкс
          Step2_HI;    // высокий уровень пина
          delayMicroseconds(delPuls);  // ждём X мкс
          Step2_LO;    // низкий уровень пина
        }
      }

      // Если направление движения влево.
      if (RezistY < (480)) {
        interval2 = map(RezistY, 0, 480, MinY, MaxY);
        currentMillis2 = micros();
        Dir2_HI;

        if (currentMillis2 - previousMillisD2 >= interval2) {
          previousMillisD2 = currentMillis2;
          Step2_HI;    // высокий уровень пина
          delayMicroseconds(delPuls);  // ждём X мкс
          Step2_LO;    // низкий уровень пина

          delayMicroseconds(delPuls);  // ждём X мкс
          Step2_HI;    // высокий уровень пина
          delayMicroseconds(delPuls);  // ждём X мкс
          Step2_LO;    // низкий уровень пина

          delayMicroseconds(delPuls);  // ждём X мкс
          Step2_HI;    // высокий уровень пина
          delayMicroseconds(delPuls);  // ждём X мкс
          Step2_LO;    // низкий уровень пина
        }
      }
    }
  //------ конец режима РАБОТА движение от джойстиков ------//
}
