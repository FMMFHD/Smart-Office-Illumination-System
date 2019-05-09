#ifndef SERVER_CLASSES_H
#define SERVER_CLASSES_H

#include <list>
#include <cstddef>
#include "servidor.h"

class node {
  private:
    float lux[600];
    int pwm[600];
    float control_ref;
    int buffer_point;
    int iterations;
    float energy;
    float aux_error;
    float aux_flicker;
    bool occup;

  public:
    node();
    ~node();
    void insertDATA(float, int);
    void change_ref(float);
    void change_occup(char);
    int get_buffer_point();
    int get_iterations();
    int* get_pwm();
    float get_energy();
    float get_aux_error();
    float get_aux_flicker();
    float get_meas_lux();
    float get_lux_i(int);
    float get_dutycycle();
    float get_pwm_i(int);
    bool get_occupancy();
    float get_lux_lower();
    float get_ext_lux();
    float get_lux_contr_ref();
    float get_inst_power();
    float get_time();
    float get_accu_energy();
    float get_accu_error();
    float get_accu_flicker();
    void get_buffer(char, char *);
};

class nodes {
  public:
    //auto hash = [=] (int a ) { return a - 4; };
    node node_i[2];
    nodes();
    ~nodes();
    float get_inst_total_power();
    float get_accu_total_energy();
    float get_total_error();
    float get_total_flicker();
};

void select_function(char * data, std::size_t * size, nodes *all_nodes, session *);

#endif //SERVER_CLASSES_H
