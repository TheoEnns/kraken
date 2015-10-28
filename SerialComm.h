#ifndef KRAKEN_SERIALCOMM_H
#define KRAKEN_SERIALCOMM_H

#define serial_IO       Serial2
#define serial_Debug    SerialUSB

#define MAX_NUM_COMM_SERIALS     3

class SerialComm {
public:
    SerialComm(void);
    ~SerialComm();
    
    //Debug or both
    void startJsonMsg(bool isDebugOnly);
    void printVar(char* varName, float value, bool isDebugOnly);
    void printVar(char* varName, int value, bool isDebugOnly);
    void printVar_F(char* varName, float value, bool isDebugOnly);
    void printVar_D(char* varName, int value, bool isDebugOnly);
    void endJsonMsg(bool isDebugOnly);
    
    //Both debug and non-debug
    void startJsonMsg();
    void printVar(char* varName, float value);
    void printVar(char* varName, int value);
    void printVar_F(char* varName, float value);
    void printVar_D(char* varName, int value);
    void endJsonMsg();
private:
    bool isJsonStart;
};

SerialComm::SerialComm(void){
    isJsonStart = false;
}

SerialComm::~SerialComm(){
}

void SerialComm::startJsonMsg(bool isDebugOnly){
    SerialUSB.print("{");
    if(!isDebugOnly){
        Serial2.print("{");
    }
    isJsonStart = true;
}

void SerialComm::printVar(char* varName, int value, bool isDebugOnly){
  printVar_D(varName, value, isDebugOnly);
}
void SerialComm::printVar(char* varName, float value, bool isDebugOnly){
  printVar_F(varName, value, isDebugOnly);
}

void SerialComm::printVar_F(char* varName, float value, bool isDebugOnly) {
    if(!isJsonStart){
        SerialUSB.print(",");
    }
    SerialUSB.print("\"");
    SerialUSB.print(varName);
    SerialUSB.print("\":");
    SerialUSB.print(value,4);
    
    if(!isDebugOnly){
        if(!isJsonStart){
            Serial2.print(",");
        }
        Serial2.print("\"");
        Serial2.print(varName);
        Serial2.print("\":");
        Serial2.print(value,4);
    }
    isJsonStart = false;
}

void SerialComm::printVar_D(char* varName, int value, bool isDebugOnly){
    if(!isJsonStart){
        SerialUSB.print(",");
    }
    SerialUSB.print("\"");
    SerialUSB.print(varName);
    SerialUSB.print("\":");
    SerialUSB.print(value,DEC);
    
    if(!isDebugOnly){
        if(!isJsonStart){
            Serial2.print(",");
        }
        Serial2.print("\"");
        Serial2.print(varName);
        Serial2.print("\":");
        Serial2.print(value,DEC);
    }
    isJsonStart = false;
}

void SerialComm::endJsonMsg(bool isDebugOnly){
    SerialUSB.println("}");
    if(!isDebugOnly)
        Serial2.println("}");
    isJsonStart = false;
}

void SerialComm::startJsonMsg() {
    startJsonMsg(false);
}

void SerialComm::printVar(char* varName, float value) {
    printVar(varName, value, false);
}

void SerialComm::printVar(char* varName, int value) {
    printVar(varName, value, false);
}

void SerialComm::printVar_F(char* varName, float value) {
    printVar_F(varName, value, false);
}

void SerialComm::printVar_D(char* varName, int value) {
    printVar_D(varName, value, false);
}

void SerialComm::endJsonMsg() {
    endJsonMsg(false);
}

extern SerialComm serialComms;

#endif
