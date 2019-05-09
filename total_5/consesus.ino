////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
    Ficheiro que contém as funções do algoritmo consesus
    1º Começa se por preencher a estrutura abaixo indicada
    2º Envia-se ao outro nó a mensagem de inicialização para ele saber que se vai começar a realizar o algoritmo consesus
    3º A cada itereção do algorimto calcula-se o melhor valor da percentagem do pwm para cada nó e envia -se ao outro nó esse resultado.
    4º Começa-se uma nova iteração após receber os valores do outro nó.
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include "consesus.h"


//Definição da estrutura do nó
typedef struct {
  int index;
  double d[2], d_avg[2], lagrange_multi[2], k[2];
  double norm;
  double m;
  int c; //cost
  double o; //external illuminant
  double L; // lower bound illuminant
} node;

//Inicialização de variáveis
node node1;

int number_iterations = 0;

extern float G0;

int first_cons = 1;

extern bool consesus_runing;

extern bool new_parameters;

extern volatile float forward_lux;
extern volatile float lux_boundary;

double d[2], d_aux[2], cost, rho = 0.07;

extern volatile double d_rec[8];
extern volatile int point_d_rec;
int point_d_rec_cons = 0;

extern int number_nodes;

extern float Kij;

extern byte _aux[4];

extern int address_viz;
extern const int own_address;

extern int d_final;

int already_sent = 0;

/////////////////////////////////////
///////////////APLICAÇÃO/////////////
/////////////////////////////////////

void compute_norm_and_m ()
{
  node1.norm = pow(node1.k[0], 2) + pow(node1.k[1], 2);

  node1.m = node1.norm - pow(node1.k[node1.index - 1], 2);
}


void Send_solution()
{
  //Serial.println("\nDELAY");
  for (int i = 0; i < 1000; i++);

  //Serial.println("Send Solutionfunction ");

  /*Serial.print("d[0] ");
  Serial.println(d[0]);
  Serial.print("d[1] ");
  Serial.println(d[1]);*/

  Wire.beginTransmission(other_add);//get BUS
  Wire.write('C');
  Wire.write(own_address);
  //Wire.write(number_nodes);
  //Serial.println("sending bytes");
  for (int i = 0; i < number_nodes; i++)
  {
    *((double*)_aux) = d[i];
    Wire.write(_aux[0]);
    Wire.write(_aux[1]);
    Wire.write(_aux[2]);
    Wire.write(_aux[3]);
    /*Serial.println(_aux[0]);
    Serial.println(_aux[1]);
    Serial.println(_aux[2]);
    Serial.println(_aux[3]);*/

  }
  Wire.write('N');
  Wire.endTransmission();

  /*Serial.println("Sending");
  Serial.print(d[0]); Serial.print("    "); Serial.println(d[1]);*/
}

//Função que inicializa os parametros do nó
void begin_alg()
{
  //Serial.println("Begin alg");
  consesus_runing = true;

  if (first_cons == 1)
  {
    first_cons = 0;
    //Serial.println("Begin alg start == False");
    double k[number_nodes];

    if (address_viz > own_address)
    {
      node1.index = 1;
      k[0] = (double) G0 * 5 / 255;
      k[1] = (double) Kij;

    }
    else
    {
      node1.index = 2;
      k[0] = (double) Kij;
      k[1] = (double) G0 * 5 / 255;
    }
    node1.c = 1;

    for (int i = 0; i < number_nodes; ++i)
    {
      node1.k[i] = k[i];
    }
  }

  node1.o = offset_LUX;
  node1.L = lux_boundary;

  for (int i = 0; i < number_nodes; ++i)
  {
    node1.d[i] = 0;
    node1.d_avg[i] = 0;
    node1.lagrange_multi[i] = 0;
  }

  Serial.print("node:");
  Serial.print("index: ");
  Serial.print(node1.index);
  if ( node1.index == 1)
  {
    Serial.print(" kii: ");
    Serial.print(node1.k[0], 5);
    Serial.print(" kij: ");
    Serial.print(node1.k[1], 5);
  }
  else
  {
    Serial.print(" kii: ");
    Serial.print(node1.k[1], 5);
    Serial.print(" kij: ");
    Serial.print(node1.k[0], 5);
  }

  Serial.print(" o: ");
  Serial.print(node1.o, 5);
  Serial.print(" L: ");
  Serial.print(node1.L);

  compute_norm_and_m();

  Serial.print(" norm: ");
  Serial.print(node1.norm);
  Serial.print(" m: ");
  Serial.println(node1.m, 5);

  for (int i = 0; i < number_nodes; i++)
  {
    d[i] = 0;
  }

  number_iterations = 1;
}

int check_feasibility( )
{
  double tol = 0.001;


  double aux1 = (d_aux[0] * node1.k[0] + d_aux[1] * node1.k[1]);


  double aux2 = (node1.L - node1.o - tol);

  if (d_aux[node1.index - 1] < 0 - tol)
  {
    return 0;
  }
  else if (d_aux[node1.index - 1] > 255 + tol)
  {
    return 0;
  }
  else if (aux1 < aux2)
  {
    return 0;
  }
  else
    return 1;

}

double evaluate_cost()
{
  double cost_aux, aux = 0;

  cost_aux = node1.c * d_aux[node1.index - 1];
  for ( int i = 0; i < 2; i++)
    cost_aux = cost_aux + node1.lagrange_multi[i] * (d_aux[i] - node1.d_avg[i]);

  for (int i = 0; i < 2; i++)
  {
    double aux1;
    aux1 = d_aux[i] - node1.d_avg[i];
    aux = aux + pow(aux1, 2);
  }

  cost_aux = cost_aux + rho * aux / 2;

  return cost_aux;
}

void primal_solve(void)
{
  //Serial.println("primal solve:");
  int sol_unconstrained = 1, sol_boundary_linear = 1, sol_boundary_0 = 1;
  int sol_boundary_255 = 1, sol_linear_0 = 1, sol_linear_255 = 1;
  double cost_best = 1000000;
  double d_best[2];;
  double z[2], d_u[2];
  int solve_unconstrained = 0;
  d_best[0] = -1;
  d_best[1] = -1;


  z[0] = rho * node1.d_avg[0] - node1.lagrange_multi[0];
  z[1] = rho * node1.d_avg[1] - node1.lagrange_multi[1];

  //Serial.print("Z: ");
  //Serial.print(z[0]);
  //Serial.print(" ");
  //Serial.println(z[1]);

  /*se soubermos o custo do outro pode se alterar aqui*/

  z[node1.index - 1] = z[node1.index - 1] - node1.c;

  Serial.print("Z: ");
  Serial.print(z[0]);
  Serial.print(" ");
  Serial.println(z[1]);

  //unconstrained minimum
  d_u[0] = (1 / rho) * z[0];
  d_u[1] = (1 / rho) * z[1];

  Serial.print("d_u[0] ");
  Serial.print(d_u[0]);
  Serial.print(" ");
  Serial.println(d_u[1]);

  d_aux[0] = d_u[0];

  d_aux[1] = d_u[1];

  sol_unconstrained = check_feasibility();

  //Serial.println("sol_unconstrained: ");
  //Serial.println(sol_unconstrained);

  if ( sol_unconstrained == 1)
  {

    double cost_unconstrained = evaluate_cost( );
    //Serial.println("cost_unconstrained: ");
    //Serial.println(cost_unconstrained);
    if (cost_unconstrained < cost_best)
    {
      d[0] = d_u[0];
      d[1] = d_u[1];
      cost = cost_unconstrained;

      //if unconstrained solution exits, then it is optimal
      //Serial.println("Fim primal solve unc");
      solve_unconstrained = 1;
    }
  }

  if (solve_unconstrained == 0)
  {
    double d_b1[2];


    double aux;

    //compute minimum constrained to linear boundary
    for ( int i = 0; i < 2; i++)
    {
      double aux1;

      aux = 1 / rho;

      d_b1[i] = aux * z[i];
      aux1 = z[1] * node1.k[1];
      aux1 = z[0] * node1.k[0] + aux1;
      aux1 = aux * aux1;
      aux1 = aux1 + node1.o;
      aux = aux1 - node1.L;
      aux1 = node1.k[i] / node1.norm;
      d_b1[i] = d_b1[i] - aux1 * aux;
    }

    Serial.print("d_b1[0]: ");
    Serial.print(d_b1[0]);
    Serial.print(" ");
    Serial.println(d_b1[1]);

    d_aux[0] = d_b1[0];
    d_aux[1] = d_b1[1];

    //check feasibility of minimum constrained to linear boundary
    sol_boundary_linear = check_feasibility();

    //Serial.println("sol_boundary_linear: ");
    //Serial.println(sol_boundary_linear);



    //compute cost and if best store new optimum
    if (sol_boundary_linear == 1)
    {
      double cost_boundary_linear = evaluate_cost();
      //Serial.println("cost_boundary_linear: ");
      //Serial.println(cost_boundary_linear);
      if (cost_boundary_linear < cost_best)
      {
        d_best[0] = d_b1[0];
        d_best[1] = d_b1[1];
        cost_best = cost_boundary_linear;
      }
    }

    //compute minimum constrained to 0 boundary
    double d_b0[2];
    aux = (1 / rho);
    for (int i = 0; i < 2; i++)
    {
      d_b0[i] = aux * z[i];
    }
    d_b0[node1.index - 1] = 0;

    Serial.print("d_b0[0]: ");
    Serial.print(d_b0[0]);
    Serial.print(" ");
    Serial.println(d_b0[1]);

    d_aux[0] = d_b0[0];
    d_aux[1] = d_b0[1];

    //check feasibility of minimum constrained to 0 boundary
    sol_boundary_0 = check_feasibility();

    //Serial.println("sol_boundary_0: ");
    //Serial.println(sol_boundary_0);

    //compute cost and if best store new optimum
    if (sol_boundary_0 == 1)
    {
      double cost_boundary_0 = evaluate_cost();

      //Serial.println("cost_boundary_0: ");
      //Serial.println(cost_boundary_0);

      if (cost_boundary_0 < cost_best)
      {
        d_best[0] = d_b0[0];
        d_best[1] = d_b0[1];
        cost_best = cost_boundary_0;
      }
    }

    // compute minimum constrained to 255 boundary
    for (int i = 0; i < 2; i++)
    {
      d_b1[i] = aux * z[i];
    }
    d_b1[node1.index - 1] = 255;

    Serial.print("d_b1[0]: ");
    Serial.print(d_b1[0]);
    Serial.print(" ");
    Serial.println(d_b1[1]);


    d_aux[0] = d_b1[0];
    d_aux[1] = d_b1[1];

    //check feasibility of minimum constrained to 255 boundary
    sol_boundary_255 = check_feasibility();

    //Serial.println("sol_boundary_255: ");
    //Serial.println(sol_boundary_255);


    //compute cost and if best store new optimum
    if (sol_boundary_255 == 1)
    {
      double cost_boundary_255 = evaluate_cost();
      //Serial.println("cost_boundary_255: ");
      //Serial.println(cost_boundary_255);
      if (cost_boundary_255 < cost_best)
      {
        d_best[0] = d_b1[0];
        d_best[1] = d_b1[1];
        cost_best = cost_boundary_255;
      }
    }

    // compute minimum constrained to linear and to 0 boundary
    double d_l0[2];
    for (int i = 0; i < 2; i++)
    {
      int aux1;
      double aux2;
      aux1 = node1.index - 1;
      aux = 1 / rho;
      d_l0[i] = aux * z[i];
      aux = node1.o - node1.L;
      aux = aux * (1 / node1.m);
      aux = aux * node1.k[i];
      d_l0[i] = d_l0[i] - aux;
      aux = node1.k[aux1] * z[aux1];
      aux2 = z[0] * node1.k[0];
      aux2 = aux2 + z[1] * node1.k[1];
      aux = aux - aux2;
      aux2 = 1 / rho;
      aux2 = aux2 / node1.m;
      aux2 = aux2 * node1.k[i];
      aux =  aux2 * aux;
      d_l0[i] = d_l0[i] + aux;
    }
    d_l0[node1.index - 1] = 0;

    Serial.print("d_l0[0]: ");
    Serial.print(d_l0[0]);
    Serial.print(" ");
    Serial.println(d_l0[1]);

    d_aux[0] = d_l0[0];
    d_aux[1] = d_l0[1];

    //check feasibility of minimum constrained to linear and to 0 boundary
    sol_linear_0 = check_feasibility();

    //Serial.println("sol_linear_0: ");
    //Serial.println(sol_linear_0);

    //compute cost and if best store new optimum
    if (sol_linear_0 == 1)
    {
      double cost_linear_0 = evaluate_cost();
      //Serial.println("cost_linear_0: ");
      //Serial.println(cost_linear_0);

      if (cost_linear_0 < cost_best)
      {
        d_best[0] = d_l0[0];
        d_best[1] = d_l0[1];
        cost_best = cost_linear_0;
      }
    }

    // compute minimum constrained to linear and to 255 boundary
    double d_l1[2];
    for (int i = 0; i < 2; i++)
    {
      double aux1;
      int index = node1.index - 1;
      aux = 1 / rho;
      d_l1[i] = aux * z[i];
      aux = node1.o - node1.L;
      aux1 = 255 * node1.k[node1.index - 1];
      aux = aux + aux1;
      aux = aux * (1 / node1.m);
      aux = aux * node1.k[i];
      d_l1[i] = d_l1[i] - aux;
      aux1 = z[index];
      aux = node1.k[index] * aux1;
      aux1 = z[0] * node1.k[0];
      aux1 = aux1 + z[1] * node1.k[1];
      aux = aux - aux1;
      aux1 = 1 / rho;
      aux1 = aux1 / node1.m;
      aux1 = node1.k[i] * aux1;
      aux = aux1 * aux;
      d_l1[i] = d_l1[i] + aux;
    }
    d_l1[node1.index - 1] = 255;

    Serial.print("d_l1[0]: ");
    Serial.print(d_l1[0]);
    Serial.print(" ");
    Serial.println(d_l1[1]);

    d_aux[0] = d_l1[0];
    d_aux[1] = d_l1[1];

    //check feasibility of minimum constrained to linear and to 255 boundary
    sol_linear_255 = check_feasibility();
    //Serial.println("sol_linear_255: ");
    //Serial.println(sol_linear_255);


    //compute cost and if best store new optimum
    if (sol_linear_255 == 1)
    {
      double cost_linear_255 = evaluate_cost();
      //Serial.println("cost_linear_255: ");
      //Serial.println(cost_linear_255);

      if (cost_linear_255 < cost_best)
      {
        d_best[0] = d_l1[0];
        d_best[1] = d_l1[1];
        cost_best = cost_linear_255;
      }
    }

    d[0] = d_best [0];
    d[1] = d_best [1];

    cost = cost_best;

    //Serial.print("d0: ");
    //Serial.println(d[0]);
    //Serial.print(" d1: ");
    //Serial.println(d[1]);

    //Serial.println("Fim primal solve");
  }
}



//Quando recebe uma mensagem do I2C com o token C implica que recebeu os valores do outro nó logo pode começar a próxima iteração
int Algorithm_consesus()
{
  while (point_d_rec_cons != point_d_rec)
  {
    if (d_rec[point_d_rec_cons] == -111 && d_rec[point_d_rec_cons + 1] == -111)
    {
      Serial.println("consesus begin");
      begin_alg();

      already_sent = 0;
     // point_d_rec = 2;
      point_d_rec_cons = 0;
      d_rec[0] = 0;
      d_rec[1] = 0;
      for(int i = 0; i<10001;i++);
    }

    /* if((d_rec[0] == node1.d[0]) && (d_rec[1] == node1.d[1]) && d_rec[0] != 0 && d_rec[1] != 0)
      {
        number_iterations = 50;
      }*/

    for (int j = 0; j < number_nodes; j++)
    {
      double aux;
      node1.d_avg[j] = (node1.d[j] + d_rec[point_d_rec_cons + j]) / 2;
      aux = node1.d[j] - node1.d_avg[j];
      aux = rho * aux;
      node1.lagrange_multi[j] = node1.lagrange_multi[j] + aux;
    }

    Serial.print("number iterations: ");
    Serial.println(number_iterations);

    Serial.print("d_rec[0] ");
    Serial.println(d_rec[point_d_rec_cons]);
    Serial.print("d_rec[1] ");
    Serial.println(d_rec[point_d_rec_cons + 1]);

    Serial.print("d[0]: ");
    Serial.println(node1.d[0]);
    Serial.print("d[1]: ");
    Serial.println(node1.d[1]);

    if (number_iterations == 50)
    {
      d_final = (int) node1.d_avg[node1.index - 1];

      forward_lux = node1.k[0] * node1.d_avg[0] + node1.k[1] * node1.d_avg[1] + node1.o;

      Serial.println("-----------------------------------------------------------");
      Serial.print(new_parameters); Serial.print("    ");
      if ( consesus_runing == true && new_parameters == false)
      {
        Serial.println("Already true");
        already_sent = 1;
        Send_solution();
        consesus_runing = false;
        new_parameters = true;
        number_iterations++;
      }
    }
    else if (number_iterations < 50)
    {
      //node1

      primal_solve();

      for (int j = 0; j < number_nodes; j++)
      {
        node1.d[j] = d[j];
      }

      /*Serial.print("d[0]_sol ");
      Serial.println(node1.d[0]);
      Serial.print("d[1]_sol ");
      Serial.println(node1.d[1]);*/

      Send_solution();
      number_iterations++;
    }

    point_d_rec_cons += 2;

    if (point_d_rec_cons == 8)
      point_d_rec_cons = 0;
  }

  //Serial.println(" "); //Serial.println(" "); //Serial.println(" "); //Serial.println(" ");

  return number_iterations;
}

//Quando um nó necessita de fazer este algoritmo chama esta função que avisa os vizinhos
int Algorithm_consesus_begin()
{
  Serial.println("consesus begin");
  begin_alg();

  d[0] = -111;
  d[1] = -111;

  Send_solution();
  ////Serial.println("ola Algorithm_consesus_begin");
  d[0] = 0;
  d[1] = 0;

  point_d_rec = 2;
  point_d_rec_cons = 0;


  int n = Algorithm_consesus();
  return n;

}
