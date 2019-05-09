#include "classes.h"
#include <string.h>
#include <algorithm>
#include <iostream>
#include <stdlib.h>
#define period 9.9
using namespace std;

void node::insertDATA(float rec_lux, int rec_pwm)
{
  printf("buffer_point: %d iterations: %d rec_lux: %f rec_pwm: %d\n", buffer_point, iterations, rec_lux, rec_pwm);
  lux[buffer_point] = rec_lux;
  pwm[buffer_point] = rec_pwm;

  energy += (pwm[buffer_point] / 255) * 0.01;
  aux_error += max(get_lux_lower() - get_meas_lux(), (float)0);

  if (buffer_point > 2 && ((lux[buffer_point] - lux[buffer_point - 1]) * (lux[buffer_point] - lux[buffer_point - 1]) < 0))
    aux_flicker += (abs(lux[buffer_point]) + abs(lux[buffer_point - 1])) / (2 * 0.001 * period);

  buffer_point++;
  iterations++;

  if (buffer_point == 600)
    buffer_point = 0;

}

void node::change_ref(float f_lux)
{
    control_ref = f_lux;
}

void node::change_occup(char arg)
{
  if (arg == '0')
    occup == false;
  else if (arg == '1')
    occup == true;
}

node::node()
{
  cout << "Node criada" << endl;
  iterations = 0;
  control_ref = 0;
  buffer_point = 0;
  energy = 0;
  aux_error = 0;
  aux_flicker = 0;
  occup = false;
  pwm[0] = 0;
  pwm[599] = 0;
  lux[598] = 0;
  lux[599] = 0;
  lux[0] = 0;
}

node::~node()
{
  cout << "Node destruída" << endl;
}

int node::get_buffer_point()
{
  if(buffer_point - 1 < 0)
    return 599;
  else return buffer_point - 1;
}

int node::get_iterations()
{
  if(iterations-1 < 0)
    return 0;
  else return iterations - 1;
}

int* node::get_pwm()
{
  return pwm;
}

float node::get_energy()
{
  return energy;
}

float node::get_aux_error()
{
  return aux_error;
}

float node::get_aux_flicker()
{
  return aux_flicker;
}

float node::get_meas_lux()
{
  return lux[get_buffer_point()];
}

float node::get_lux_i(int i)
{
    return lux[i];
}

float node::get_dutycycle()
{
  return pwm[get_buffer_point()];
}

float node::get_pwm_i(int i)
{
    return pwm[i];
}

bool node::get_occupancy()
{
  return occup;
}

float node::get_lux_lower()
{
  if (occup) {
      return 500;
  }
  else {
      return 200;
  }
}

float node::get_ext_lux()
{
  if (occup)
  {
    if (lux[get_buffer_point()] > 500)
      return (lux[get_buffer_point()] - 500);
    else return 0;
  }
  else
  {
    if (lux[get_buffer_point()] > 200)
      return (lux[get_buffer_point()] - 200);
    else return 0;
  }
}

float node::get_lux_contr_ref()
{
  return control_ref;
}

float node::get_inst_power()
{
  return pwm[get_buffer_point()] / 255;
}

float node::get_time()
{
  return (get_iterations() * 0.001 * period);
}

float node::get_accu_energy()
{
  return energy;
}

float node::get_accu_error()
{
  if(get_iterations() == 0)
    return 0;
  else return aux_error / get_iterations();
}

float node::get_accu_flicker()
{
  if(get_iterations() == 0)
    return 0;
  return aux_flicker / get_iterations();
}

void node::get_buffer(char arg, char * data)
{
  int i, j;
  char aux[10];

  if (arg == 'l')
  {

    if (iterations < 600)
    {
        for (i = 0; i < iterations; i+=6) {
        sprintf(aux, "%0.2f,", lux[i]);
        strcat(data, aux);
      }
    }

    else{
      for (i = buffer_point + 6; i != buffer_point; i+=6) {
        if(i > 599)
          i = i - 600;

        sprintf(aux, "%0.2f,", lux[i]);
        strcat(data, aux);
      }
    }
  }

  if (arg == 'd')
  {
      if (iterations < 600)
      {
          for (i = 0; i < iterations; i+=6) {
              sprintf(aux, "%d,", pwm[i]);
              strcat(data, aux);
          }
        }
        else
        {
            for (i = buffer_point + 6; i != buffer_point; i+=6) {
                if(i > 599)
                    i = i - 600;

                sprintf(aux, "%d,", pwm[i]);
                strcat(data, aux);
            }
        }
  }

  strcat(data,"\n");
}

nodes::nodes()
{
  cout << "Nodes criada" << endl;
}

nodes::~nodes()
{
  cout << "Nodes destruida" << endl;
}

float nodes::get_inst_total_power()
{
  int power = 0, _point, * _pwm;

  for (int i = 0; i < 2; i++)
  {
    _point = node_i[i].get_buffer_point();
    _pwm = node_i[i].get_pwm();
    power += (_pwm[_point] / 255);
  }

  return power;
}

float nodes::get_accu_total_energy()
{
  float _energy = 0;

  for (int i = 0; i < 2; i++)
  {
    _energy += node_i[i].get_energy();
  }

  return _energy;
}

float nodes::get_total_error()
{
  float _error = 0;
  for (int i = 0; i < 2; i++)
  {
    if (node_i[i].get_iterations() == 0)
      return 0;

    _error += (node_i[i].get_aux_error() / node_i[i].get_iterations());
  }

  return _error;
}

float nodes::get_total_flicker()
{
  float _flicker = 0;
  for (int i = 0; i < 2; i++)
  {
    if (node_i[i].get_iterations() == 0)
      return 0;

    _flicker += (node_i[i].get_aux_flicker() / node_i[i].get_iterations());
  }

  return _flicker;
}

void select_function(char * data, size_t * size, nodes *all_nodes, session *s)
{
  char arg3[5];
  char arg0, arg1, arg2;
  bool aux_bool = true;
  float aux_float = 0;
  int arg_int = 0, aux = 0;

  aux = sscanf( data, "%c %c %c\n", &arg0, &arg1, &arg2);
  if (aux != 3 )
  {
    strcpy(data, "Comando errado!1\n");
    (*size) = strlen(data);
    return;
  }
  if (arg2 != 'T')
  {
    aux = sscanf(data, "%c %c %s\n", &arg0, &arg1, arg3);
    arg_int = atoi(arg3);
    if(arg_int != 0 && aux == 3)
      arg_int = arg_int - 4;
    else
    {
      strcpy(data, "Comando errado!2\n");
      (*size) = strlen(data);
      return;
    }
  }

  if (arg0 == 'g')
  {
    switch (arg1)
    {
      case 'l': {
          aux_float = (*all_nodes).node_i[arg_int].get_meas_lux();
          break;
        }
      case 'd': {
          aux_float = (*all_nodes).node_i[arg_int].get_dutycycle();
          break;
        }
      case 'L': {
          aux_float = (*all_nodes).node_i[arg_int].get_lux_lower();
          break;
        }
      case 'o': {
          aux_float = (*all_nodes).node_i[arg_int].get_ext_lux();
          break;
        }
      case 'r': {
          aux_float = (*all_nodes).node_i[arg_int].get_lux_contr_ref();
          break;
        }
      case 't': {
          aux_float = (*all_nodes).node_i[arg_int].get_time();
          break;
        }
      case 'p': {
          if (arg2 == 'T')
            aux_float = (*all_nodes).get_inst_total_power();
          else aux_float = (*all_nodes).node_i[arg_int].get_inst_power();
          break;
        }
      case 'e': {
          if (arg2 == 'T')
            aux_float = (*all_nodes).get_accu_total_energy();
          else aux_float = (*all_nodes).node_i[arg_int].get_accu_energy();
          break;
        }
      case 'c': {
          if (arg2 == 'T')
            aux_float = (*all_nodes).get_total_error();
          else aux_float = (*all_nodes).node_i[arg_int].get_accu_error();
          break;
        }
      case 'v': {
          if (arg2 == 'T')
            aux_float = (*all_nodes).get_total_flicker();
          else aux_float = (*all_nodes).node_i[arg_int].get_accu_flicker();
          break;
        }
      case 's': {
          if (aux_bool)
            sprintf(data, "%c %c true\n", arg0, arg1);
          else sprintf(data, "%c %c false\n", arg0, arg1);
          (*size) = strlen(data);
          return;
        }
      default: {
          strcpy(data, "Comando errado!3\n");
          (*size) = strlen(data);
          return;
        }
    }

    sprintf(data, "%c %c %f\n", arg0, arg1, aux_float);
    (*size) = strlen(data);
    return;
  }
  else if (arg0 == 'b')
  {
      if(arg1 != 'l' && arg1 != 'd')
      {
          strcpy(data, "Comando errado!4\n");
          (*size) = strlen(data);
          return;
      }
      sprintf(data, "%c %c %d \0", arg0, arg1, (arg_int+4));
      (*all_nodes).node_i[arg_int].get_buffer(arg1, data);
      (*size) = strlen(data);
    return;
  }
  else if (arg0 == 's')
  {
    if(arg1 == 'l' && arg_int == 0)
    {
        if(s->stream == 1)
            s->stream = 0;
        else s->stream = 1;
    }
      if(arg1 == 'l' && arg_int == 1)
      {
          if(s->stream == 3)
              s->stream = 0;
          else s->stream = 3;
      }
      if(arg1 == 'd' && arg_int == 0)
      {
          if(s->stream == 2)
              s->stream = 0;
          else s->stream = 2;
      }
      if(arg1 == 'd' && arg_int == 1)
      {
          if(s->stream == 4)
              s->stream = 0;
          else s->stream = 4;
      }

      if(s->stream != 0)
      {
          if(arg_int == 0)
              s->read_buffer4 = (*all_nodes).node_i[arg_int].get_iterations();
          if(arg_int == 1)
              s->read_buffer5 = (*all_nodes).node_i[arg_int].get_iterations();
      }
  }
  else {
    strcpy(data, "Comando errado!5\n");
    (*size) = strlen(data);
    return;
  }
}