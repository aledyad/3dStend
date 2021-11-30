// правка скетча под схему от 30,03,21

// Сторонняя библиотека для работы с портами ввода-вывода.
#include ".\libraries\CyberLib\CyberLib.cpp"
#include <SoftwareSerial.h>

// Номер пина для RX (TX на пульте).
#define RX 3
// Номер пина для TX (RX на пульте).
#define TX 2
// DEBUG
//#define RX 2
//#define TX 1

SoftwareSerial mySerial(RX, TX);

// Алиас команды чтения положения потенциометра поворота камеры.
#define potpin_Read A3_Read
int valServo;

// Алиас команды чтения положения потенциометра поворота платформы.
#define RezistX_Read A0_Read
// Алиас команды чтения положения потенциометра вертикального перемещения камеры.
#define RezistY_Read A1_Read

// кнопка авария pin D8
#define AlarmBtn_In   D8_In
#define AlarmBtn_Read D8_Read
#define AlarmBtn_HI   D8_High

// кнопка работа pin A2 кн1
#define RunBtn_In     D16_In
#define RunBtn_Read   D16_Read
#define RunBtn_HI     D16_High

// кнопка Zero  pin D7
#define ZeroBtn_In   D7_In
#define ZeroBtn_Read D7_Read
#define ZeroBtn_HI   D7_High

// кнопка ПК  pin D11   // 10
#define PK_Btn_In   D11_In
#define PK_Btn_Read D11_Read
#define PK_Btn_HI   D11_High

// кнопка вкл Р6  pin D12   // 10
//сброс питания с драйверов шд
#define BtnP6_In   D12_In
#define BtnP6_Read D12_Read
#define BtnP6_HI   D12_High

// LED  ZERO D6
#define LedZero_Out  D6_Out
#define LedZero_HI   D6_High
#define LedZero_LO   D6_Low
#define LedZero_Read D6_Read
#define  LedZero_Inv D6_Inv

// LED работа   D5
#define LedRun_Out  D5_Out
#define LedRun_HI   D5_High
#define LedRun_LO   D5_Low
#define LedRun_Read D5_Read

// LED  авария    D4
#define LedAlarm_Out  D4_Out
#define LedAlarm_HI   D4_High
#define LedAlarm_LO   D4_Low
#define LedAlarm_Read D4_Read

//  LED связи  D9
#define LedLink_Out  D9_Out
#define LedLink_HI   D9_High
#define LedLink_LO   D9_Low
#define LedLink_Read D9_Read
#define LedLink_Inv  D9_Inv

byte RunState = 0; // статус Работа  0-нет/1-ДА
byte ZeroState = 0; // статус уст в "0"  0-нет/1-ДА
byte SQZ = 0; // статус уст в "0"  0-нет/1-ДА
byte AlarmState = 0; // статус уст в "0"  0-нет/1-ДА
byte AlarmStend = 0; // статус уст в "0"  0-нет/1-ДА
// Количество отправленных пакетов на стенд. Сбрасывается в 0 при достижении значения 250.
byte count = 0;
boolean fLink = true;
boolean CRC = true;

unsigned long timerZero, tSend = 0;

// переменне пульта
word   VRx, VRy, VR = 0; //  переменник джойстика Х, У, SERVO
word   Run, Zero, Alarm = 0; //  режим РАБОТА, движ к "0",АВАРИЯ    0- НЕТ / 1 - ДА
word   CountReturn = 0;           //cчетчик для контроля за связью
word  countAlarm = 0;
//----------структура  данных для ПРД -----------//
struct Str {
  word VRx;  // переменник джойстика Y
  word VRy;  // переменник джойстика Y
  word VR;   // переменник SERVO
  byte Run; // режим РАБОТА  0- НЕТ / 1 - ДА
  byte Zero; // // режим движ к "0"  0- НЕТ / 1 - ДА
  byte Alarm; // // режим АВАРИЯ  0- НЕТ / 1 - ДА
  byte PK; // // режим вкл ПК (реле № 5)  0- НЕТ / 1 - ДА
  byte P6; // // режим вкл (реле № 6)  0- НЕТ / 1 - ДА   сброс питания с драйверов шд
  byte Link; // // режим LINK  0- НЕТ / 1 - ДА
  byte Return; //cчетчик для контроля за связью
  byte crc; // контрольная сумма
};

// Структура для отправки данных на пульт.
struct StrOtv {
  byte NoRun;       // режим РАБОТА  0- НЕТ / 1 - ДА
  byte EndZero;     // режим движ к "0"  0- НЕТ / 1 - ДА
  byte SQZ;
  byte AlarmStend;  // режим ававрия на стенде  0- НЕТ / 1 - ДА
  byte Link;        // режим LINK  0- НЕТ / 1 - ДА
  byte Return;      //cчетчик для контроля за связью
  byte crc;         // Контрольная сумма.
};

// создаём саму структуру
StrOtv bufOtv;


// Рассчитать контрольную сумму.
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

void setup()
{ //Serial.begin(9600);                   // Инициируем аппаратный последовательный порт
  mySerial.begin(9600);                  // Инициируем программный последовательный порт
  mySerial.setTimeout(3);                // таймаут влияет скорость обработки по умолчанию 1000 мс

  // настройка пинов
  AlarmBtn_In; AlarmBtn_HI;// // кнопка S1 авария A2
  RunBtn_In; RunBtn_HI;// // кнопка SW2 работа кн1
  BtnP6_In; BtnP6_HI;// // кнопка  вкл Р6   сброс питания с драйверов шд
  ZeroBtn_In; ZeroBtn_HI;// // кнопка SW3 0 кн2
  PK_Btn_In; PK_Btn_HI;// // кнопка вкл ПК
  LedZero_Out;  LedZero_LO; // LED  0
  LedLink_Out;  LedLink_LO; // LED  Link
  LedRun_Out;  LedRun_LO; // LED работа
  LedAlarm_Out;  LedAlarm_LO; // LED авария


  tSend = millis();
}// end setup

void loop()
{

  // читаем родным методом readBytes()
  // указываем ему буфер-структуру, но приводим тип к byte*
  // размер можно указать через sizeof()

  if (mySerial.readBytes((byte*)&bufOtv, sizeof(bufOtv))) {
    // считаем crc пакета:
    // передаём буфер, преобразовав его к (byte*)
    // а также его ПОЛНЫЙ размер, включая байт crc
    byte CRCOtv = crc8_bytes((byte*)&bufOtv, sizeof(bufOtv));
    // если crc равен 0, данные верны (такой у него алгоритм расчёта)

    if (CRCOtv == 0) {
      CRC = true;

      if (bufOtv.EndZero == 1)   ZeroState = 0; // пришел флаг установка в 0 закончена
      SQZ = bufOtv.SQZ;                   //  сост. КВ 0 (ZERO)
      AlarmStend = bufOtv.AlarmStend;     //  сигнал АВАРИЯ со стенда
      CountReturn = bufOtv.Return;        //  возврат счетчика контроля связи

    } // end  if (CRCOtv == 0)
    else CRC = false;

  }// end if (mySerial.readBytes

  // считаем сколько пришло обратно импульсов
  if ((count - CountReturn) > 110) countAlarm++;

  else countAlarm = 0;


  valServo =  potpin_Read;                   // читаем пин потенциометра СЕРВО привода (0-1023)
  valServo = map(valServo, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180)

  //---- обработчик нажатия кнопки РАБОТА -----//
  if (RunBtn_Read == LOW and RunState == 0)
  { LedRun_HI;// ЛЕД РАБОТА  ГОРИТ
    RunState = 1;

    delay(500);
  }
  else if (RunBtn_Read == LOW and RunState == 1)
  { LedRun_LO;// ЛЕД РАБОТА  НЕ ГОРИТ
    RunState = 0;
    ZeroState = 0;
    delay(500);
  }


  //---- обработчик нажатия кнопки ZERO -----//
  if (RunState == 1) // если режим РАБОТА
    if (ZeroBtn_Read == LOW and ZeroState == 0)
    { ZeroState = 1;
      delay(500);
    }
    else if (ZeroBtn_Read == LOW and ZeroState == 1)
    { ZeroState = 0;

      delay(500);
    }

  //---------- вкл. тумблер АВАРИЯ  ----------//

  if ((AlarmBtn_Read == LOW) or (fLink == false) or AlarmStend == 1)
  {
    LedAlarm_HI; // ЛЕД АВАРИЯ  ГОРИТ
    AlarmState = 1;
    RunState = 0;
    ZeroState = 0;
    LedRun_LO; // ЛЕД РАБОТА  НЕ ГОРИТ
    LedZero_LO;// ЛЕД 0  НЕ ГОРИТ

  }
  else
    //если нет ни одного аварийного события
    if ((AlarmBtn_Read == HIGH) and (fLink == true) and AlarmStend == 0)
    {
      LedAlarm_LO; // ЛЕД АВАРИЯ НЕ ГОРИТ
      AlarmState = 0;
    }
  // --------- ПЕРЕДАЕМ  ДАННЫЕ НА  СТЕНД  ------------ //
  if (millis() > tSend + 250) // каждую 250 mc
  { // буфер на отправку
    Str buf;

    if (countAlarm > 25) {
      fLink = false;
      LedLink_Inv;//  ЛЕД LINK  МИГАЕТ когда на связи
      //LedLink_HI;//  ЛЕД LINK  ГОРИТ когда нет связи
      buf.Link = 0;

    }
    else
    { buf.Link = 1;
      LedLink_HI; //  ЛЕД LINK  ГОРИТ когда нет связи
      //LedLink_Inv;//  ЛЕД LINK  МИГАЕТ когда на связи
      fLink = true;
    }

    if (count >= 250)count = 0; // обнуляем счетчик обратной связи

    // заполняем структуру данными
    buf.VRx = RezistX_Read;
    buf.VRy = RezistY_Read;
    buf.VR = valServo;
    buf.Run = RunState;
    buf.Zero = ZeroState;
    buf.Alarm = AlarmState;
    if (PK_Btn_Read == HIGH) buf.PK = 0;  else buf.PK = 1;
    if (BtnP6_Read == HIGH) buf.P6 = 0;  else buf.P6 = 1;
    buf.Return = count++; //увеличиваем счетчик отправлений
    // последний байт - crc. Считаем crc всех байт кроме последнего, то есть кроме самого crc!!! (размер-1)
    buf.crc = crc8_bytes((byte*)&buf, sizeof(buf) - 1);

    // отправляем родным write()
    // указываем ему буфер-структуру, но приводим тип к byte*
    // размер можно указать через sizeof()
    mySerial.write((byte*)&buf, sizeof(buf));

    // DEBUG
    //char hex[4];
    //for(int i=0; i < sizeof(buf); i++)
    //{
    //  sprintf(hex, "%02X", ((byte*)&buf)[i]);
    //  mySerial.write(hex);
    //}
    //mySerial.write("\n");
    
    tSend = millis(); // перезапускаем таймер ПРД
  }



  // включен режим установка в "0"
  if (ZeroState == 1) {
    // мигаем светиком
    if (millis() > timerZero + 1000) // каждую 1 сек
    { timerZero = millis();
      LedZero_Inv;
    }// end  if (millis()> timerZero +
  }
  else if (SQZ == 1) LedZero_HI; //если сработал КВ то горит постоянно
  else
    LedZero_LO;// иначе не горит

}
