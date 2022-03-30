/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */


#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"

#ifdef ESP8266_WIFI

#include "esp8266_wifi.h"

#include "../../lcd/tenlog/tenlog_touch_lcd.h"
#include "ESP8266WebServer/ESP8266_AT_WebServer.h"
#include "Delay.h"

char wifi_status[100] = "";
char wifi_ssid[20] = WIFI_DEFAULT_SSID;
char wifi_pswd[20] = WIFI_DEFAULT_PSWD;
uint8_t wifi_ena = WIFI_DEFAULT_ENA;
uint32_t http_port = WIFI_DEFAULT_PORT;

int status = WL_IDLE_STATUS;			// the Wifi radio's status
int reqCount = 0;						// number of requests received
bool WifiBusy = false;

ESP8266_AT_WebServer server;		//port

#define BUFFER_SIZE     256

void handleRoot() {
  char c_command [BUFFER_SIZE];
  long _command[BUFFER_SIZE];
  char message[BUFFER_SIZE];
  if(!WifiBusy){
    WifiBusy = true;
    sprintf_P(c_command, PSTR("%s"), server.arg("code"));
    for(int i=0; i<256; i++){
      _command[i] = c_command[i];
    }

    process_command_gcode(_command);
    WifiBusy = false;
  }
  
  server.send(200, "text/html", wifi_status);
}

void handleNotFound(){
	char message[BUFFER_SIZE];
  sprintf_P(message, PSTR("404 File Not Found! \nURI:%s \nMethod: Get \n"), server.uri()); 
  server.send(404, "text/plain", message);
  TLDEBUG_LNPAIR("message:\n", message);
}


void wifiResetEEPROM(){
    wifi_ena = WIFI_DEFAULT_ENA;
    sprintf_P(wifi_ssid, "%s", WIFI_DEFAULT_SSID);
    sprintf_P(wifi_pswd, "%s", WIFI_DEFAULT_PSWD);
    http_port = WIFI_DEFAULT_PORT;
}

void esp_wifi_idle() {
  if(wifi_ena && status==WL_CONNECTED){
    server.handleClient();
  }
}

// init ESP01 WIFI module pins
void esp_wifi_init() {                          
  char message[100];
  if(tl_TouchScreenType == 1){
    sprintf_P(message, PSTR("wifisetting.cbMode.val=%d"), wifi_ena);
    TLSTJC_println(message);
    sprintf_P(message, PSTR("wifisetting.tSSID.txt=\"%s\""), wifi_ssid);
    TLSTJC_println(message);
    sprintf_P(message, PSTR("wifisetting.tPwd.txt=\"%s\""), wifi_pswd);
    TLSTJC_println(message);
    sprintf_P(message, PSTR("wifisetting.nPort.val=%d"), http_port);
    TLSTJC_println(message);
  }

  EspSerial.end();
  delay(50);
  if(wifi_ena){
  
    /*
    TLDEBUG_PGM("\nStarting Http Server on ");
    TLDEBUG_PGM(BOARD_NAME);
    TLDEBUG_PGM(" with ");
    TLDEBUG_LNPGM(SHIELD_TYPE);
    TLDEBUG_LNPGM(ESP8266_AT_WEBSERVER_VERSION);
    */

    // initialize serial for ESP module
    TlLoadingMessage("Init wifi shield...");
    TLDEBUG_LNPGM("Init wifi shield commport..");
    EspSerial.begin(115200);

    if(status == WL_CONNECTED){
      WiFi.disconnect();
      TLDEBUG_LNPGM("disconnecting wifi...");
    }
    // initialize ESP module
    WiFi.init(&EspSerial);
    TLDEBUG_LNPGM("WiFi shield init done");

    // check for the presence of the shield
    status = WiFi.status();
    if (status == WL_NO_SHIELD){
      TLDEBUG_LNPGM("WiFi shield not present");
      // don't reset..
      //kill("WiFi shield not present!");
      if(tl_TouchScreenType == 1){
        TLSTJC_println("setting.tIP.txt=\"No wifi detected\"");
        TLSTJC_println("wifisetting.tIP.txt=\"No wifi detected\"");
      }
    }

    if (status != WL_NO_SHIELD){      
      // attempt to connect to WiFi network.
      //for 3 times      
      int tryCount = 0;
      while (status != WL_CONNECTED && tryCount < 3){
        
        sprintf_P(message, "Connecting to ssid %s", wifi_ssid);
        TlLoadingMessage(message);
        TLDEBUG_LNPGM(message);

        // Connect to WPA/WPA2 network
        status = WiFi.begin(wifi_ssid, wifi_pswd);
        tryCount++;
      }
      if(status == WL_CONNECTED){
        server.begin(http_port);

        server.on("/", handleRoot);
        server.on("/inline", []()
        {
          server.send(200, "text/plain", "This works as well");
        });

        server.onNotFound(handleNotFound);
        server.begin();

        IPAddress IP = WiFi.localIP();
        if(tl_TouchScreenType == 1){
          sprintf_P(message, PSTR("setting.tIP.txt=\"%d.%d.%d.%d\""), IP[0],IP[1],IP[2],IP[3]);
          TLSTJC_println(message);
          delay(20);
          sprintf_P(message, PSTR("wifisetting.tIP.txt=\"%d.%d.%d.%d\""), IP[0],IP[1],IP[2],IP[3]);
          TLSTJC_println(message);
        }
        sprintf_P(message, PSTR("HTTP server started @ %d.%d.%d.%d"), IP[0],IP[1],IP[2],IP[3]);
        TLDEBUG_LNPGM(message);
      }else{

        TLSTJC_println("setting.tIP.txt=\"Connect SSID fail\"");
        TLSTJC_println("wifisetting.tIP.txt=\"Connect SSID fail\"");
        sprintf_P(message, PSTR("Connect to SSID %s fail."), wifi_ssid);
        TlLoadingMessage(message);
        TLDEBUG_LNPGM(message);
      }
    }
  }else{
    EspSerial.end();
    //server.end();

    TLSTJC_println("setting.tIP.txt=\"Wifi Disabled\"");
    TLSTJC_println("wifisetting.tIP.txt=\"Wifi Disabled\"");
    TLDEBUG_LNPGM("Wifi Disabled");
  }

  /*
  #if PIN_EXISTS(ESP_WIFI_MODULE_GPIO0)
    OUT_WRITE(ESP_WIFI_MODULE_GPIO0_PIN, HIGH);
  #endif
  #if PIN_EXISTS(ESP_WIFI_MODULE_GPIO2)
    OUT_WRITE(ESP_WIFI_MODULE_GPIO2_PIN, HIGH);
  #endif
  #if PIN_EXISTS(ESP_WIFI_MODULE_RESET)
    delay(1);  // power up delay (0.1mS minimum)
    OUT_WRITE(ESP_WIFI_MODULE_RESET_PIN, LOW);
    delay(1);
    OUT_WRITE(ESP_WIFI_MODULE_RESET_PIN, HIGH);
  #endif
  #if PIN_EXISTS(ESP_WIFI_MODULE_ENABLE)
    delay(1);  // delay after reset released (0.1mS minimum)
    OUT_WRITE(ESP_WIFI_MODULE_ENABLE_PIN, HIGH);
  #endif
  */
}
#endif //ESP8266_WIFI
