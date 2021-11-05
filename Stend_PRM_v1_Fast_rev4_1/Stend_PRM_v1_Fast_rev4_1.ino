// 131021
//изменили функцию формирования импулься STEP
// ввели доп.настройку задержка мкс формирование импулься

// 121021
// изменяем концевые на нормально замкнутые
// устр ошибка по реле 4
// изм.направл. вращ.ШД-2

// стенд ПРМ
// отправки и приёма структуры через Serial
// с контролем целостности данных

#include <CyberLib.h>       // Подключаем шуструю библиотеку для работы с портами IO

#include <Servo.h>
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(2, 3); // RX, TX

// структура для передачи на пульт

struct StrOtv {
  byte NoRun;       // режим РАБОТА  0- НЕТ / 1 - ДА
  byte EndZero;     //  режим движ к "0"  0- НЕТ / 1 - ДА
  byte SQZ;         // концевой 0
  byte AlarmStend;  // режим ававрия на стенде  0- НЕТ / 1 - ДА
  byte Link;        // режим LINK  0- НЕТ / 1 - ДА
  byte Return;      //cчетчик для контроля за связью
  byte crc;         // контрольная сумма
};
// создаём саму структуру
//Str buf;


// структура для приёма
// должна соответствовать отправляемой

struct Str {
  word VRx;  // переменник джойстика Y
  word VRy;  // переменник джойстика Y
  word VR;   // переменник SERVO
  byte Run; // режим РАБОТА  0- НЕТ / 1 - ДА
  byte Zero; // // режим движ к "0"  0- НЕТ / 1 - ДА
  byte Alarm; // // режим АВАРИЯ  0- НЕТ / 1 - ДА
  byte PK; // // режим реле 5 ПК 0- НЕТ / 1 - ДА
  byte P6; // // режим реле 6  0- НЕТ / 1 - ДА    сброс питания с драйверов шд
  byte Link; // // режим LINK  0- НЕТ / 1 - ДА
  byte Return; //cчетчик для контроля за связью
  byte crc;    // контрольная сумма
};

// создаём саму структуру
Str buf;


// Пин для сервопривода
int servoPin = 5;
// Создаем объект
Servo Servo1;
int valServo, oldvalServo;   // угол отклонения серво/предыдущее значение

// ----  определяем пины для реле ----//
#define Rele1_Out  D14_Out   // А0
#define Rele1_HI   D14_High
#define Rele1_LO   D14_Low
#define Rele1_Read D14_Read

#define Rele2_Out  D15_Out   // А1
#define Rele2_HI   D15_High
#define Rele2_LO   D15_Low
#define Rele2_Read D15_Read

#define Rele3_Out  D16_Out  // А2
#define Rele3_HI   D16_High
#define Rele3_LO   D16_Low
#define Rele3_Read D16_Read

#define Rele4_Out  D17_Out // А3
#define Rele4_HI   D17_High
#define Rele4_LO   D17_Low
#define Rele4_Read D17_Read

#define RelePK_Out  D18_Out  // А4
#define RelePK_HI   D18_High
#define RelePK_LO   D18_Low
#define RelePK_Read D18_Read

#define Rele6_Out  D19_Out // А5
#define Rele6_HI   D19_High
#define Rele6_LO   D19_Low
#define Rele6_Read D19_Read

// ----  определяем пины для LED ----//

#define LedLink_Out  D4_Out     //  led связи
#define LedLink_HI   D4_High
#define LedLink_LO   D4_Low
#define LedLink_Read D4_Read
#define  LedLink_Inv D4_Inv

// ----  пины драйверов ШД-1 -------//
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

//------------- пины драйверов ШД-2 -----------//
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
byte AlarmBtnState = 0; // статус уст в "0"  0-нет/1-ДА

byte Return = 0;

byte stepState1 = 0;
unsigned long previousMillisD1 = 0;

byte stepState2 = 0;
unsigned long previousMillisD2 = 0;

byte counter = 0; // счетчик нет связи

boolean SendOtvet = false; // флаг передачи
boolean fLink = true;      // флаг LINK
boolean fRelePK = false;   // флаг реле ПК
boolean AlarmDRV = false;  // флаг авария на ШД


// переменники джойстика
word RezistX, RezistY = 0;
unsigned long MaxX= 100000; 
word MinX= 1;
unsigned long MaxY= 100000; 
word MinY= 1;
unsigned long interval1, interval2 = 0;
word speedZero_1 = 50000; // скорость в режиме к "0"  ШД-1
word speedZero_2 = 50000; // скорость в режиме к "0"  ШД-2
const byte delPuls = 10; // задержка в мкс на формирование импульса STEP упр-я ШД

unsigned long currentMillis1, currentMillis2, timerZero, tSend,tRele = 0;



void setup() {
  Serial.begin(9600);
 // mySerial.begin(9600);
//  mySerial.setTimeout(3);
  Serial.setTimeout(3);
   Servo1.attach(servoPin);              // attaches the servo on pin 2 to the servo object


  Rele1_Out;  Rele1_HI; 
  Rele2_Out;  Rele2_HI; 
  Rele3_Out;  Rele3_HI; 
  Rele4_Out;  Rele4_HI; 
  RelePK_Out; RelePK_HI; // выкл
   Rele6_Out;  Rele6_HI; 
  
  //pinMode( LedLink, OUTPUT); digitalWrite( LedLink, HIGH);
  LedLink_Out;  LedLink_HI; // LedLink
  //1 ШД
  // pinMode(Dir1, OUTPUT); digitalWrite(Dir1, LOW);        // пин DIR
  Dir1_Out;  Dir1_LO;
  //pinMode(Step1, OUTPUT); digitalWrite(Step1, LOW);      // пин STEP
  Step1_Out;  Step1_LO;
  //pinMode(En1, OUTPUT);   digitalWrite(En1, LOW);        // пин ENA
  //pinMode(Alm1, INPUT_PULLUP);                           // пин  ALARM
  Alm1_In; Alm1_HI;// // пин  ALARM INPUT_PULLUP

  // pinMode( SQUp, INPUT_PULLUP);                         // концевой верх
  SQUp_In; SQUp_HI; // концевой верх INPUT_PULLUP
  // pinMode( SQDown, INPUT_PULLUP);                       // концевой низ
  SQDown_In; SQDown_HI; // концевой низ INPUT_PULLUP

  //2  ШД
  //pinMode(Dir2, OUTPUT); digitalWrite(Dir2, LOW);         // пин DIR
  Dir2_Out;  Dir2_LO;
  //pinMode(Step2, OUTPUT); digitalWrite(Step2, LOW);       // пин STEP
  Step2_Out;  Step2_LO;

  //pinMode(En2, OUTPUT);   digitalWrite(En2, LOW);         // пин ENA
  // pinMode(Alm2, INPUT_PULLUP);                           // пин  ALARM
  Alm2_In; Alm2_HI;// // пин  ALARM INPUT_PULLUP

  // pinMode( SQZero, INPUT_PULLUP);                        // концевой 0
  SQZero_In; SQZero_HI; //  концевой 0 INPUT_PULLUP

  tSend,tRele = millis();
}// end setup

// функция для расчёта crc
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
}// end crc8_bytes




void loop() {
/* if (millis() >= (tRele + 10000)and fRelePK==false) // после вклчючения ждем 10 сек
 {RelePK_LO;   // вкл с реле ПК
 delay(1000); // через 1 c
 RelePK_HI;   // выкл с реле ПК
 fRelePK=true;
 }

*/

  // читаем родным методом readBytes()
  // указываем ему буфер-структуру, но приводим тип к byte*
  // размер можно указать через sizeof()
 
   if  (Serial.readBytes((byte*)&buf, sizeof(buf)))
  {

    // считаем crc пакета:
    // передаём буфер, преобразовав его к (byte*)
    // а также его ПОЛНЫЙ размер, включая байт crc
    byte CRC = crc8_bytes((byte*)&buf, sizeof(buf));
    // если crc равен 0, данные верны (такой у него алгоритм расчёта)
   
    if (CRC == 0) {
      counter = 0;
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
      if (ZeroStateStend == 0)EndZeroState = 0;
      AlarmBtnState = buf.Alarm;
      // вкл с реле ПК
      if (buf.PK==1) RelePK_LO;  else   RelePK_HI;   // выкл с реле ПК
    // вкл с реле 6
   if (buf.P6==1) Rele6_LO;  else   Rele6_HI;   // выкл с реле 6
 
      Return = buf.Return;
      if (AlarmBtnState == 1) EndZeroState = 0;
      SendOtvet = true;
      tSend = millis();
     

    } // end  if (CRC == 0
  
  }// end if (Serial.readBytes
  else
  {   //  если нет обмена данными вкл.счетчик
    counter++;
    if (counter > 250 ) {
      fLink = false;  // подняли флаг нет связи
    }
  }
//--------- ОТПРАВКА ОБРАТНЫХ СООБЩЕНИЙ НА ПУЛЬТ ----------//
  if (SendOtvet == true)
    if (millis() > tSend + 10) // через 10 мсек
    { // буфер на отправку
      StrOtv bufOtv;
      // заполняем
      bufOtv.NoRun = NoRunState;
      bufOtv.EndZero = EndZeroState;
      
      if (SQZero_Read == 1 and SQDown_Read == 1)       // было  (SQZero_Read == 0 and SQDown_Read == 0)
        bufOtv.SQZ = 1; else bufOtv.SQZ = 0;

      bufOtv.AlarmStend = AlarmDRV;
      // Serial.print("AlarmDRV =");Serial.println(AlarmDRV);
      bufOtv.Return = buf.Return; //счетчик отправлений обратно
      // последний байт - crc. Считаем crc всех байт кроме последнего, то есть кроме самого crc!!! (размер-1)
      bufOtv.crc = crc8_bytes((byte*)&bufOtv, sizeof(bufOtv) - 1);

      // отправляем родным write()
      // указываем ему буфер-структуру, но приводим тип к byte*
      // размер можно указать через sizeof()

//      mySerial.write((byte*)&bufOtv, sizeof(bufOtv));
        Serial.write((byte*)&bufOtv, sizeof(bufOtv));
      //tSend=millis();
      SendOtvet = false;
 
    }
//---------КОНЕЦ ОТПРАВКА ОБРАТНЫХ СООБЩЕНИЙ НА ПУЛЬТ ----------//

  
// --- сигналы АЛАРМ с драйверов ШД ------//
  if (Alm1_Read == 0 or  Alm2_Read == 0)
    AlarmDRV = true;  // флаг сигнала АЛАРМ поднят
  else
      if (Alm1_Read == 1 and Alm2_Read == 1)
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

 
//------ реж НЕ РАБОТА  и нет аварии и есть связь -----//
  if ((RunStateStend == 0) and (AlarmBtnState == 0) and  fLink == true and AlarmDRV == false) { 
    Rele1_LO;// РЕЛЕ 1 ВКЛ
    Rele2_LO;// РЕЛЕ 2 ВКЛ
  }// end if (RunStateStend==0)


// если реж РАБОТА
  if (RunStateStend == 1) {
    if (AlarmBtnState == 1 or  fLink == false or AlarmDRV == true )
     {Rele4_LO; // РЕЛЕ 4 ВКЛ
      Rele1_HI;// РЕЛЕ 1 ВЫКЛ
    Rele2_HI;// РЕЛЕ 2 ВКЛ
    Rele3_HI;// РЕЛЕ 3 ВКЛ
     }
     else
  {  Rele4_HI;// РЕЛЕ 4 ВЫКЛ
    
    Rele1_HI;// РЕЛЕ 1 ВЫКЛ
    Rele2_LO;// РЕЛЕ 2 ВКЛ
    Rele3_LO;// РЕЛЕ 3 ВКЛ
  }
  }
  else if (RunStateStend == 0) {
    Rele3_HI;  // РЕЛЕ 3 ВЫКЛ
  }

  //---------КОНЕЦ УПРАВЛЯЕМ РЕЛЕ 1-4 ------------------//



//------ включен режим установка в "0" ------//
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
      
      /*    // !было  раньше
       if (stepState1 == LOW) { stepState1 = HIGH; Step1_HI;} 
        else
        { stepState1 = LOW; Step1_LO;}
        */
        
      } // end  if (currentMillis1 -
   
    } // end if ((SQDown)!=1)
  


    //--------движение ШД-2 к концевому  "0" ----------//
 
    if (SQZero_Read != 1)                     //  было (SQZero_Read != 0)
    { //Serial.println("---- движение ШД-2 к концевому  0 ----- ");

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
    if ((AlarmBtnState != 1) and AlarmDRV == false and  fLink == true)     { // если не нажата кн.АВАРИЯ, нет аварии от ШД, есть связь
      NoRunState = 0;
    
     // если есть изменения в позиции СЕРВО то крутим ее
      if ((valServo != oldvalServo)) Servo1.write(valServo); 
          oldvalServo = valServo;
    
      // ----- РАБОТА  ШД 1  ------------
   if (RezistX < 525 and RezistX > 480)     Step1_LO;
   
      if (SQUp_Read != 1)    // было  (SQUp_Read != 0)
      {
        if (RezistX > (525))     // движение - >
         
        { currentMillis1 = micros();

          interval1  = map(RezistX, 525, 1023, MaxX, MinX); 
         
          Dir1_HI;  // направление движение
         
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
     
      
      /*    // !было  раньше
            if (stepState1 == LOW) {stepState1 = HIGH; Step1_HI; }
            else 
            { stepState1 = LOW;  Step1_LO; }
       */
          }// end   if (currentMillis1 - previousMillisD1 


        } // end   if (RezistX > (525)
      }// if (SQUp)!=1)

     
      if (SQDown_Read != 1)        // было (SQDown_Read != 0)
      {
        if (RezistX < (480)) { //  <-  движение
        
                 
          interval1 = map(RezistX, 0, 480,MinX, MaxX);
          currentMillis1 = micros();
          Dir1_LO;  // направление движение

         
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
      
      /*    // !было  раньше
            if (stepState1 == LOW) {stepState1 = HIGH;Step1_HI; } 
            else
            { stepState1 = LOW; Step1_LO;}
           */
          }// end if (currentMillis1


        }// end if (RezistX < (480)
      }// end if ((SQDown)!=1)

      
      // -----КОНЕЦ РАБОТА  ШД 1  ------------

      // ----- РАБОТА  ШД 2  ------------
      if (RezistY < 525 and RezistY > 480)     Step2_LO;
     
      if (RezistY > (525)) { // движение - >
       
        currentMillis2 = micros();
        interval2 = map(RezistY, 525, 1023, MaxY, MinY); // настройка шага движения

        Dir2_LO;//напрвление движения          //  было Dir2_HI;

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
      
      /*    // !было  раньше
          if (stepState2 == LOW) {stepState2 = HIGH; Step2_HI;} 
            else 
            {  stepState2 = LOW;  Step2_LO;  }
            */
          } // end  if (currentMillis2 - previousMillisD2 

      }// end if (RezistA1 > (521)

      if (RezistY < (480)) { // <-  движение
       
        interval2 = map(RezistY, 0, 480, MinY, MaxY); // настройка шага движения
        currentMillis2 = micros();
        Dir2_HI;    //напрвление движения         //  было Dir2_LO;

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
	 
	 
      
      /*    // !было  раньше
           if (stepState2 == LOW) {stepState2 = HIGH; Step2_HI;} 
            else 
            {  stepState2 = LOW;  Step2_LO;  }
     */
        }// end  if (currentMillis2 - previousMillisD2 

      } // end if (RezistA1 < (489)
      // -----КОНЕЦ РАБОТА  ШД 2  ------------

    }// end if (AlarmBtn)!=1)

  //------ конец режима РАБОТА  движение от джойстиков ------//


 

}// end loop
