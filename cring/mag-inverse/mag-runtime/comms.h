#pragma once
#include <winsock2.h>
#include <array>

typedef std::array<double, 9> sensor_data;

int init_winsock();
int init_socket(SOCKET &);
int init_out_socket(SOCKET& s);
int get_sensor_data_from_socket(SOCKET s, sensor_data& data);
int send_solution_to_socket(SOCKET s, const double* buffer, int len_bytes);
void close_socket(SOCKET);