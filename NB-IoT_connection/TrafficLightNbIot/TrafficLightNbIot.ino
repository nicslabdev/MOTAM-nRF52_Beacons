/*********************************************************************************************/
/*
 * Traffic Light NB-IoT management
 * Created by Manuel Montenegro, April 22, 2019.
 * Developed for MOTAM project.
 * 
 *  This sketch receives management messages from MOTAM operator through NB-IoT connection in
 *  order to manage Traffic Light state.
 * 
 *  Management messages supported: trafficLightConf and trafficLightEmergency.
 *  
 *  Compatible boards: Arduino Leonardo & Arduino/Genuino 101 with AVNET BG96 Shield.
*/
/*********************************************************************************************/

#include <Bg96NbIot.h>


String HOST = "m0t4m.duckdns.org";                      // MOTAM server address
String SERVER_PATH = "/narrowband/trafficlight/events/";
// String HOST = "lti.adabyron.uma.es";
// String SERVER_PATH = "/";
int SERVER_PORT = 9090;                                 // MOTAM server port

Bg96NbIot nbiot;                                        // BG96 module instance

int pulsesPin = 12;
int lastPulses = 9;

void sendPulses (int puls)
{
    for (int i = 0; i < puls; i++)
    {
        digitalWrite (pulsesPin, LOW);
        delay(100);
        digitalWrite (pulsesPin, HIGH);
        delay(400);
    }
}

void setup() 
{
    Serial.begin(115200);
    pinMode (pulsesPin, OUTPUT);
    digitalWrite (pulsesPin, HIGH);
}


void loop()
{

    bool flag = true;

    if (flag)
    {
        Serial.println("Please, power on BG96");
        flag = nbiot.begin();
    }

    if (!flag)
    {
        Serial.println("Error on nbiot.begin");
    }

    if (flag)
    {
        Serial.println("IP: "+nbiot.getIP());
        Serial.println("IMEI: "+nbiot.getIMEI());
    }




        if (flag)
        {
            flag = nbiot.openSslSocket(HOST, SERVER_PORT);
        } 

        if (!flag)
        {
            Serial.println("Error on openSslSocket");
        }








    while (flag)
    {
        // if (flag)
        // {
        //     flag = nbiot.openSslSocket(HOST, SERVER_PORT);
        // } 

        // if (!flag)
        // {
        //     Serial.println("Error on openSslSocket");
        // }

        if (flag)
        {
            flag = nbiot.sendDataBySsl("GET "+SERVER_PATH+" HTTP/1.1\r\nHost: "+HOST+"\r\nConnection: keep-alive\r\n\r\n");
        }

        if (!flag)
        {
            Serial.println("Error on sendDataBySsl");
        }

        if (flag)
        {
            String data = nbiot.receiveDatabySsl();

            Serial.println(data);

            int state;
            int puls = 0;                                       // Number of pulses

            if (data.indexOf ("\"type\":\"trafficLightEmergency\"") > 0)
            {
                String stateString = data.substring(data.indexOf("{\"trafficLightState\":")+21,data.indexOf("{\"trafficLightState\":")+22);
                state = stateString.toInt();

                switch (state) {
                    case 0:
                        puls = 3;
                        Serial.println("Red emergency");
                        break;
                    case 2:
                        puls = 2;
                        Serial.println("Green emergency");
                        break;
                    default:
                        puls = 0;
                        break;
                }
            } 
            else if (data.indexOf ("\"type\":\"trafficLightConf\"") > 0)
            {
                puls = 1;
                Serial.println ("Normal cycle");
            }
            else
            {
                flag = false;
            }
            
            if (puls != lastPulses)
            {
                sendPulses (puls);
                lastPulses = puls;
            }
            
        }

    }

    nbiot.shutdown();

}