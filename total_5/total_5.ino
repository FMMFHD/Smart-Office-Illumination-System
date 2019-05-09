////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  Things to do:
  -Por uma nova condição para ele funcionar
  -testar o sistema
  -Fazer em classes
  -beber cerveza
  -rave party!!!!!!!
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include "consesus.h"

// definação de pins
extern const int LEDpin;
extern const int LDRpin;
extern const int button;

volatile int lux_mediana0 = 0;
volatile int lux_mediana1 = 0;
volatile int lux_mediana2 = 0;
volatile int number = 0;
volatile int butao = 0;

// CONSTANTES QUE DEFINEM OS PARAMETROS DE CADA UMA DAS PLACAS
// constantes da placa com etiqueta bonita
const float LDR_offset = 5.2665;
const float LDR_slope = -0.7196;

//constantes da placa com etiqueta feia
//const float LDR_offset = 5.3027;
//const float LDR_slope = -0.7184;

// controlo de frequencia do PWM
const byte mask = B11111000; //mask bits that are not prescale
int prescale = 1;  // fastest possible

// Constantes
const int debounceDelay = 1000;
const float max_LUX = 533;
const float min_LUX = 190;
#define BAUD_RATE  2000000

///////////////////////////// VARIAVEIS ///////////////////
extern float offset_LUX;
extern float G0;
float aux1, aux2;
volatile float forward_volt = 0.00;
extern volatile float forward_lux;
extern volatile float lux_boundary;
volatile int state = 0;

//////////////////////////// SIMULADOR  ///////////////////
float ti = 0;
float vi = 0.00;
float vf = 0.00;
float tau = 11.500;
volatile float v[20];
volatile int count = 0;
volatile int limite = 0;
float sim_volt = 0;
volatile float error_volt = 0.00;
volatile float lux = 0.00;
volatile float lux_ant = 0.00;
volatile float lux_antigo, lux_novo;
volatile int TIMER_flag = 0;

////////////////////////////// PI ////////////////////////
volatile float PI_volt = 0.00;
volatile float PI_volt_ant = 0.00;
volatile float PI_p = 0.00;
volatile float PI_i = 0.00;
volatile float PI_i_ant = 0.00;
volatile float error_volt_ant = 0.00;
float Kp = 0.7;
float Ki = 0.055;
float T = 9.984;
float K2 = Kp * Ki * T / 2;
volatile float anti_wind = 0;
float Ka = 1;
volatile int pwm = 0;

volatile int pi_finish = 0;
extern byte _aux[4];

///////////////////////ADDRESS//////////////////////////////
extern const int own_address; //this dev address
extern const int other_add; //other dev address
extern int address_viz;

/////////////////////I2C buffer/////////////////////////////
extern volatile int guardar;
extern volatile int pronto_processar;
extern volatile byte buffer_guardar[30];
extern volatile int point_buffer_guardar;

//////////////////////////////Consesus////////////////////////
extern int number_nodes;
extern volatile double d_rec[8];
extern volatile int point_d_rec;
extern bool consesus_runing;
extern bool new_parameters;
volatile int next_consesus = 0;
extern int d_final;
int inicio = 0;
bool first_time = true;
int help = 0;
int consesus_iterration = 0;

///////////////////////////////K compute///////////////////////
extern float Kij;

float tempo = 0.00;

///////////////////////////////////////////////////////////
///////////////////////////// FUNCOES /////////////////////
///////////////////////////////////////////////////////////

/* Debounce retorna true se o botao esta premido e estavel */
boolean debounce(int pin) {
  boolean state;
  boolean previousState;
  previousState = digitalRead(pin);
  for (int counter = 0; counter < debounceDelay; counter++) {
    state = digitalRead(pin);
    if (state != previousState) {
      counter = 0;
      previousState = state;
    }
  }
  return state;
}

//-------------------------------------------------------------------------------------

/*Calcular a resistência de LDR*/
float LDR_res( int val) {
  float L = val * 5.0 / 1023;
  L = ((5.0 - L) / (L / 10000.0));
  return L;
}

//---------------------------------------------------------------------------------------

/* Conversao de tensao para unidades de LUX */
float convert_LUX( int val) {
  float L = pow(10.0, ((log10(LDR_res(val)) - LDR_offset) / LDR_slope));
  return L;
}

//-------------------------------------------------------------------------------------

/*Expressão retiradas da interpolação entre vários pontos*/
void compute_tau (void )
{
  /*Lux esta a descer*/
  if (lux_antigo > lux_novo)
  {
    if (lux_novo > 190)
    {
      tau = -0.141 * lux_novo + 38.288;
    }
    else
    {
      tau = pow(-7 * 10, -8) * pow(lux_novo, 3);
      tau = tau + 0.0001 * pow(lux_novo, 2);
      tau = tau - 0.0779 * lux_novo;
      tau = tau + 22.081;
    }
  }
  else if (lux_antigo < lux_novo)
  {
    tau = pow( -8 * 10, -8) * pow(lux_novo, 3);
    tau = tau + 0.0001 * pow(lux_novo, 2);
    tau = tau - 0.0919 * lux_novo;
    tau = tau + 26.836;
  }
}

//---------------------------------------------------------------------------------------

/* Determinçao dos parametros de feedforward */
void init_FEEDFORWARD() {
  // Determinaçao do valor de offset das mediçoes de lux
  analogWrite(LEDpin, 0);
  delay(100);
  for (int i = 0; i < 100; i++)
    offset_LUX = offset_LUX + convert_LUX(analogRead(LDRpin)) / 100;

  Serial.print("LUX offset is "); Serial.println(offset_LUX);

  // Determinaçao do ganho dos LDR
  aux2 = 0;
  analogWrite(LEDpin, 255);
  delay(500);
  for (int i = 0; i < 100; i++) {
    aux1 = analogRead(LDRpin);
    aux2 = aux2 + aux1 / 100;
    G0 = G0 + convert_LUX(aux1) / 100;
  }
  aux2 = aux2 * 5 / 1023;

  G0 = (G0 - offset_LUX) / aux2;
  Serial.print("LDR gain is "); Serial.println(G0);
  Serial.println();
}

//--------------------------------------------------------------------------------------

///Processa -se o buffer na 2 parte do I2C ou seja após se ter calculado o K
void Proc_buffer()
{
  byte byte_buffer[4];
  if (buffer_guardar[0] == 'C')
  {
    //Serial.print("Proc buff "); Serial.println(point_d_rec);
    d_rec[point_d_rec] = 0;
    d_rec[point_d_rec + 1] = 0;

    byte_buffer[0] = buffer_guardar[2];
    byte_buffer[1] = buffer_guardar[3];
    byte_buffer[2] = buffer_guardar[4];
    byte_buffer[3] = buffer_guardar[5];
    d_rec[point_d_rec] = *((double*) byte_buffer);

    byte_buffer[0] = buffer_guardar[6];
    byte_buffer[1] = buffer_guardar[7];
    byte_buffer[2] = buffer_guardar[8];
    byte_buffer[3] = buffer_guardar[9];
    d_rec[point_d_rec + 1] = *((double*) byte_buffer);

    Serial.print(point_d_rec);
    Serial.print("  ");
    Serial.print(d_rec[point_d_rec]);
    Serial.print("  ");
    Serial.println(d_rec[point_d_rec + 1]);

    point_d_rec += 2;

    /*Serial.print("pbuf ");
      Serial.println(point_d_rec);*/

    if (point_d_rec == 8)
      point_d_rec = 0;

    next_consesus = 1;
  }
}

//--------------------------------------------------------------------------------------

/*Função que retorna a mediana de 3 valores*/
void what_is_number ( )
{
  if (lux_mediana2 > lux_mediana0)
  {
    if (lux_mediana2 < lux_mediana1)
    {
      error_volt = lux_mediana2;
    }
    else if (lux_mediana1 > lux_mediana0)
    {
      error_volt = lux_mediana1;
    }
    else
    {
      error_volt = lux_mediana0;
    }
  }
  else
  {
    if (lux_mediana0 < lux_mediana1)
    {
      error_volt = lux_mediana0;
    }
    else if (lux_mediana1 > lux_mediana2)
    {
      error_volt = lux_mediana1;
    }
    else
    {
      error_volt = lux_mediana2;
    }

  }
}

//--------------------------------------------------------------------------------------

void Calculus()
{
  TIMER_flag = 0;

  forward_volt = d_final * 5 / 255; // conversao de pwm para volt

  /* determinação de +arametros do simulador */
  vi = analogRead(LDRpin);
  Serial.print("forward");
  Serial.println(forward_volt);
  vi = vi / 2013 * 5;
  vf = log10(forward_lux);   // log(R) = slope*log(Y) + offset
  vf = vf * LDR_slope + LDR_offset;   // log(lux) -> log(R)
  vf = pow(10, vf);
  vf = 10000 + vf;
  vf = 50000 / vf;  // R -> V

  ti = 0;
  Serial.print("v[i]");
  limite = 0;
  for ( int i = 0; i < 20; i++, limite++)
  {
    v[i] = exp(-1 * ti / tau);
    v[i] = (vf - vi) * v[i];
    v[i] = vf - v[i];   // sim_volt esta em V

    ti = ti + T;

    if ( v[i] == vf)
    {
      i = 20;
    }
    Serial.print("  ");
    Serial.print(v[i]);
  }
  if (limite == 20)
    limite = 19;

  v[limite] = vf;

  Serial.println();
  Serial.print("vf   "); Serial.println(vf);
  Serial.print("limite    "); Serial.println(limite);
  count = 0;

  TIMER_flag = 1;
}

///////////////////////////////////////////////////////////
////////////////////////// INTERRUPTORES //////////////////
///////////////////////////////////////////////////////////

/* defeniçao de interruptor que define o estado da presença de uma pessoa na secretaria */
void deskSTATE() {

  TIMER_flag = 0;
  butao = 1;

  lux_antigo = lux_boundary;
  if (debounce(button) == HIGH) {  // nao existe pessoa na secretaria
    lux_boundary = min_LUX;
    state = 0;
  }

  if (debounce(button) == LOW) {  // ha uma pessoa na secretaria
    lux_boundary = max_LUX;
    state = 1;
  }
  lux_novo = lux_boundary;
  compute_tau();

  TIMER_flag = 1;
}

//------------------------------------------------------------------------------------

/* Calibração do sistema de iluminação */
void receiveEvent_1_parte(int howMany)
{
  int nread = 0;
  while (Wire.available() > 0) { //check data on BUS
    byte aux = Wire.read();
    if ((aux == 'O' || aux == 'F' || aux == 'R') && pronto_processar == 0 )
    {
      if (aux == 'O')
        nread = 5;
      else
        nread = 3;

      point_buffer_guardar = 1;
      buffer_guardar[0] = aux;
      guardar = 1;
      continue;
    }
    else if ((aux == 'N') && pronto_processar == 0 && point_buffer_guardar == nread - 1)
    {
      guardar = 0;
      pronto_processar = 1;

    }
    if (guardar == 1 && pronto_processar == 0 )
    {
      buffer_guardar[point_buffer_guardar] = aux;
      point_buffer_guardar++;
    }
  }
}

//--------------------------------------------------------------------------------------

/* Receção de dados do consesus (I2C) */
void receiveEvent_2_parte(int howMany)
{
  while (Wire.available() > 0) { //check data on BUS
    byte aux = Wire.read();
    //Serial.println(aux);

    if ( aux == 'C' && pronto_processar == 0  && inicio == 0)
    {
      aux = Wire.read();
      if (aux != address_viz) {
        point_buffer_guardar = 0;
        guardar = 0;
        inicio = 0;
        continue;
      }

      point_buffer_guardar = 0;
      guardar = 1;
      inicio = 1;
      buffer_guardar[point_buffer_guardar] = 'C';
      point_buffer_guardar++;
      buffer_guardar[point_buffer_guardar] = aux;
      point_buffer_guardar++;
      continue;
    }
    else if (point_buffer_guardar == 10)
    {
      guardar = 0;
      inicio = 0;
      if ((aux == 'N') && pronto_processar == 0 )
      {
        pronto_processar = 1;
        buffer_guardar[point_buffer_guardar] = aux;
        point_buffer_guardar++;
      }
      else
      {
        point_buffer_guardar = 0;
      }
    }
    if (guardar == 1 && pronto_processar == 0 )
    {
      buffer_guardar[point_buffer_guardar] = aux;
      point_buffer_guardar++;
    }
  }
  if (pronto_processar == 1)
  {
    Proc_buffer();
    pronto_processar = 0;
    next_consesus = 1;
  }
}

//-------------------------------------------------------------------------------------

/* Controlodor Proporcional e Integrador */
void PID()
{
  if (count == limite)
  {
    sim_volt = v[limite];
  }
  else
  {
    sim_volt = v[count];
    count++;
  }

  //Serial.print("Pdil "); Serial.println(error_volt * 5 / 1023);
  //Serial.print("sim_volt  "); Serial.println(sim_volt);
  //Serial.print("sim count   ");Serial.println(count);
  lux = convert_LUX(error_volt);
  error_volt = error_volt * 5 / 1023;   // conversao de valor analogico para voltagem
  error_volt = sim_volt - error_volt;

  // caso o valor medido é igual ao valor da amostra anterior, mantem-se o mesmo valor de OUTPUT
  if ( error_volt != error_volt_ant)//  && abs(lux - lux_ant) > 2 )
  {
    PI_p = Kp * error_volt;
    PI_i = PI_i_ant + K2 * (error_volt + error_volt_ant ) + Ka * anti_wind;
    PI_volt = PI_p + PI_i;
    PI_volt_ant = forward_volt + PI_volt;

    // limitaçao do valor de OUTPUT entre 0 e 5 V
    if (PI_volt_ant < 0 )
    {
      anti_wind = 0 - PI_volt_ant;
      PI_volt_ant = 0;
    }
    if (PI_volt_ant > 5)
    {
      anti_wind = 5 - PI_volt_ant;
      PI_volt_ant = 5;
    }

    analogWrite(LEDpin, PI_volt_ant * 255 / 5);
    //Serial.print("Pdill "); Serial.println(PI_volt_ant * 255 / 5);
    error_volt_ant = error_volt;
    PI_i_ant = PI_i;
    lux_ant = lux;
  }
}

//--------------------------------------------------------------------------------------

/*Rotina de interrupção da amostragem do sinal*/
ISR(TIMER2_COMPA_vect) {

  if ( number == 0)
  {
    lux_mediana0 = analogRead(LDRpin);
    number = 1;
  }
  else if ( number == 1)
  {
    lux_mediana1 = analogRead(LDRpin);
    number = 2;
  }
  else if (number == 2)
  {
    lux_mediana2 = analogRead(LDRpin);
    pi_finish = 1;
    number = 0;
    what_is_number();
    PID();
  }
}

//--------------------------------------------------------------------------------------

/* pré definição do interruptor do timer 2 */
void initTIMER2INTERRUPT() {
  cli(); //DISABLE INTERRUPTS
  // SET TIMER2 - 8 BITS
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0; // inicializa o contador a 0
  OCR2A = 51; //   16e6/ (f*Prescaler) - 1
  TCCR2B |= (1 << WGM12); // enable the CTC mode
  TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22); //set prescaler to 1024
  TIMSK2 |= (1 << OCIE2A); //enable the interrupt
  sei(); // ENABLE INTERRUPTS
}

////////////////////////////////////////////////////////////
//////////////////////////// SETUP  ////////////////////////
////////////////////////////////////////////////////////////

void setup() {
  analogWrite(LEDpin, 0);
  delay(1000);
  Serial.begin(BAUD_RATE);
  Serial.println("setup");

  pinMode(LEDpin, OUTPUT);
  pinMode(LDRpin, INPUT);
  pinMode(button, INPUT);
  digitalWrite(button, HIGH);

  Wire.begin(own_address);

  Wire.onReceive(receiveEvent_1_parte); //event handler
  //----------------------------------
  TCCR1B = (TCCR1B & mask) | prescale;
  TWAR = (own_address << 1) | 1;
  //----------------------------------

  k_compute();
  pronto_processar = 0;

  Serial.print("O valor de Kij é "); Serial.println(Kij);

  Serial.println("Computed k");

  analogWrite(LEDpin, 0);
  delay(500);  // garantir que o led esta desligado

  consesus_runing = true;

  if (own_address == 4)
    address_viz = 5;

  Wire.onReceive(receiveEvent_2_parte);
  deskSTATE();

  butao = 0;

  if (own_address == 4)
  {
    Serial.println("wait");
    delay(1000);  // como o endereço 4 e o "mais" mestre que os outro, espera de forma a que os outros estejam bloqueados para receber mensagem
    tempo = micros();
    consesus_iterration = Algorithm_consesus_begin();
    pronto_processar = 0;
  }
}

///////////////////////////////////////////////////////////////
/////////////////////////////  LOOP  //////////////////////////
///////////////////////////////////////////////////////////////

void loop() {

  /* Quando há mudança de estado da ocupação do sistema, reinicia-se o controlador PI */
  if ( butao == 1)
  {
    consesus_runing = true;

    consesus_iterration = Algorithm_consesus_begin();
    tempo = micros();

    butao = 0;
    Serial.println("change occupancy");
  }

  if (consesus_runing == false && new_parameters == true)
  {
    Calculus();
    new_parameters = false;
    analogWrite(LEDpin, d_final);
    Serial.println("finish");
    Serial.print("  d_final:  ");
    Serial.println(d_final);
    Serial.print("Forward lux: ");
    Serial.println(forward_lux);
    Serial.println("parametesr");
    Serial.println(new_parameters);
    Serial.println("new goal");

    if (first_time == true)
    {
      attachInterrupt(digitalPinToInterrupt(button), deskSTATE, CHANGE);
      butao = 0;
      initTIMER2INTERRUPT();
      Serial.println("Init interrupt and timer");
      first_time = false;
    }

    *((double*)_aux) = forward_lux;
    // enviar mensagem para rapsberry com informação do estado da secretaria
    Wire.beginTransmission(other_add);//get BUS
    Wire.write('S');
    Wire.write(own_address);
    Wire.write(state);  // occupation state
    Wire.write(_aux[0]);
    Wire.write(_aux[1]);
    Wire.write(_aux[2]);
    Wire.write(_aux[3]);
    Wire.write('N');
    Wire.endTransmission();

    point_d_rec = 0;
    for (int ij = 0; ij < 8; ij++)
      d_rec[ij] = 0.00;

    Serial.print("Tempo: "); Serial.println(micros() - tempo);
  }

  /* Pode se entrar dentro da condição após ter o valor retirado à saida do sistema */
  if (pi_finish == 1 && consesus_runing == false)
  {
    //Serial.print("L:  ");
    //Serial.println(lux_ant);
    pi_finish = 0;
    *((double*)_aux) = lux_ant;
    Serial.println(lux_ant);
    // enviar informação sobre o nível de luminosidade lido na secretaria
    Wire.beginTransmission(other_add);//get BUS
    Wire.write('T');
    Wire.write(own_address);
    Wire.write(d_final);
    Wire.write(_aux[0]);
    Wire.write(_aux[1]);
    Wire.write(_aux[2]);
    Wire.write(_aux[3]);
    Wire.write('N');
    Wire.endTransmission();
  }

  if (next_consesus == 1)
  {
    next_consesus = 0;
    ti = -1;
    help = 0;
    consesus_iterration = Algorithm_consesus();
  }
  else if (consesus_iterration == 51 && consesus_runing == true)
    consesus_iterration = Algorithm_consesus();


}
