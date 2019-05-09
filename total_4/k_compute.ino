////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*  Ficheiro que contém as funções para o calculo do K
    Troca de mensagens para sincronizar o ligar e desligar de cada nó:
        -Primeiro nós obtamos por o que tiver o endereço 4 começa por enviar o token O => ON com o valor medido do seu ldr e no fim N => fim
        -o outro nó recebe O endereço lux N, mede o valor de lux e calcula a influencia do outro nó (lux medido / lux recebido pelo I2C) e envia
  R valor medido de lux N.
        -O nó que enviou o O recebe o R, e ao receber desliga o seu led e manda o token F(OFF).
        -O nó ao receber o token F liga o seu led mede o seu lux e envia o token O, ou seja, dá se o mesmo processo descrito anteriormente.
  Esta troca de tokens acontece até que o se põe no ldr (0-255) seja 250.
  Como se sabe que a influencia não varia muito com o pwm implica que se faz a média dos valores calculados.
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>

#include "functions.h"

float K_lux = 0;
int K_pwm = 50;
bool other_already_receive = false;
int done = 0;
int K_val_read = 0;
float K_value[10];
int point_k_value = 0;

extern float offset_LUX;

/////ADDRESS//////
extern const int own_address; //this dev address

extern const int other_add; //other dev address

extern byte _aux[4];

extern const int LEDpin;
extern const int LDRpin;
extern const int button;

extern float Kij;

byte K_byte_buffer[4];


extern volatile byte buffer_guardar[30];
extern volatile int point_buffer_guardar;

extern volatile int guardar;
extern volatile int pronto_processar;
extern int address_viz;
bool address_viz_known = false;

/**********APLICACAO**********/
// Responde após receber o token O
void I2C_response()
{
  K_lux = 0;
  for (int a = 0; a < 5; a++)
    K_lux = K_lux + convert_LUX(analogRead(LDRpin)) / 5;

  //Serial.print("Calculo "); Serial.print(K_lux); Serial.print(" / "); Serial.print(K_val_read); Serial.print(" = ");
  K_value[point_k_value] = (K_lux - offset_LUX) / K_val_read ;//Kij = luxread/pwm [lux/pwm]
  // Serial.println(K_value[point_k_value]);
  point_k_value ++;

  /* Serial.print("Responding ");
    Serial.println(K_lux);*/

  Wire.beginTransmission(other_add);//get BUS
  Wire.write('R');
  Wire.write(own_address);
  Wire.write('N');
  Wire.endTransmission(); //release BUS
}

//Resposta ao token R
void I2C_OFF()
{
  //Serial.println("Send OFF");
  analogWrite(LEDpin, 0);
  delay(200);
  Wire.beginTransmission(other_add);//get BUS
  Wire.write('F');
  Wire.write(own_address);
  Wire.write('N');
  Wire.endTransmission(); //release BUS
}

//Quando já recebeu uma mensagem é ativada uma flag que ativa esta função, que processa a mensagem
void leitura_Proc_buffer()
{
  //Serial.println("leitura_proc_buffe");
  if (buffer_guardar[0] == 'O')
  {
    /*Serial.print("Buffer: O ");

      Serial.print(buffer_guardar[1]);  /// leitura do endereço do arduino que enviou informaçao
      Serial.print(" ");*/

    for (int a = 0; a < 2; a++)
    {
      K_byte_buffer[a] = buffer_guardar[2 + a];
    }

    K_val_read = *((int*)K_byte_buffer);

    // Serial.println(K_val_read);

    I2C_response();
  }

  if (buffer_guardar[0] == 'F')
  {
    if (address_viz_known == false)
    {
      address_viz = buffer_guardar[1];
      address_viz_known = true;
    }

    //Serial.print("Buffer: F ");
    done = 1;
    if (K_pwm == 255)
    {
      Serial.println("mandei o off assincrono");
      Wire.beginTransmission(other_add);//get BUS
      Wire.write('F');
      Wire.write(own_address);
      Wire.write('N');
      Wire.endTransmission(); //release BUS
    }

  }

  if (buffer_guardar[0] == 'R')
  {
    //Serial.print("Buffer: R ");
    //Serial.print(buffer_guardar[1]);  /// leitura do endereço do arduino que enviou informaçao
    I2C_OFF();
  }
}


//É a função que envia o token O
void I2C_arduino_send() {

  K_lux = 0;
  for (int b = 0; b < 5; b++)
    K_lux = K_lux + convert_LUX(analogRead(LDRpin)) / 5;
  //Serial.print("sending ");
  //Serial.println(K_lux);

  Wire.beginTransmission(other_add);//get BUS
  Wire.write('O');

  Wire.write(own_address);

  *((int*)_aux) = K_pwm;
  //Serial.println(*((int*)_aux));
  Wire.write(_aux[0]);
  Wire.write(_aux[1]);

  Wire.write('N');
  Wire.endTransmission(); //release BUS
}


//Função que controla o processo todo, para o cálculo do K
void k_compute()
{
  analogWrite(LEDpin, 0);
  delay(500);
  int aux = 0;
  K_pwm = 50;
  Serial.println("K_compute");


  while (aux == 0)
  {
    if (own_address == 4)
    {
      if ( other_already_receive == false)
      {
        delay(2000);
        other_already_receive = true;
        //Serial.println("own_address");
        done = 1;
      }
    }

    if (done == 1)
    {
      if ( K_pwm == 50 )
      {
        init_FEEDFORWARD(); ///Não esta no ficheiro .h
      }
      /*      Serial.println(" ");
            Serial.print("o valor de k é ");
            Serial.println(K_pwm); Serial.println(" ");*/
      done = 0;
      if (K_pwm == 255)
        done = 2;
      if (done != 2)
      {
        analogWrite(LEDpin, K_pwm);
        delay(500);
        I2C_arduino_send();
      }
      K_pwm += 50;
      if (K_pwm > 250)
        K_pwm = 255;
    }


    if (pronto_processar == 1)
    {
      leitura_Proc_buffer();
      pronto_processar = 0;
    }



    if (done == 2 )
    {
      Kij = 0;
      for (int i = 0; i < point_k_value; i++)
      {
        Kij = Kij + K_value[i];
      }

      Kij = Kij / point_k_value;

      aux = 1;
    }
  }
  K_pwm = -255;
}
