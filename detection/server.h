#ifndef SERVER_H
#define SERVER_H

#include <ESP8266WebServer.h>

extern ESP8266WebServer server;
extern String activity;

void handleRoot();
void handleActivity();

#endif