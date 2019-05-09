////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  Ficheiro que contém as variáveis globais do sistema
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Wire.h>

const int LEDpin = 9;
const int LDRpin = 1;
const int button = 2;

float offset_LUX = 0.0;
float G0 = 0.0;


/////ADDRESS//////
const int own_address = 5; //this dev address

const int other_add = 0; //other dev address

int address_viz;

byte _aux[4];

volatile byte buffer_guardar[30];
volatile int point_buffer_guardar = 0;

volatile int guardar = 0;
volatile int pronto_processar = 0;

int number_nodes = 2;
volatile double d_rec[8];
volatile int point_d_rec = 0;


int d_final = 0;


volatile float forward_lux = 0.00;
volatile float lux_boundary = 0.00;

float Kij;


bool consesus_runing = false;
bool new_parameters = false;
