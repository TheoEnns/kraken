#ifndef KRAKEN_SERIALCOMM_H
#define KRAKEN_SERIALCOMM_H

//#define serial_IO       Serial2
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
    serial_Debug.print("{");
    if(!isDebugOnly){
#ifdef serial_IO
        serial_IO.print("{");
#endif
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
        serial_Debug.print(",");
    }
    serial_Debug.print("\"");
    serial_Debug.print(varName);
    serial_Debug.print("\":");
    serial_Debug.print(value,4);
    
    if(!isDebugOnly){
#ifdef serial_IO
        if(!isJsonStart){
            serial_IO.print(",");
        }
        serial_IO.print("\"");
        serial_IO.print(varName);
        serial_IO.print("\":");
        serial_IO.print(value,4);
#endif
    }
    isJsonStart = false;
}

void SerialComm::printVar_D(char* varName, int value, bool isDebugOnly){
    if(!isJsonStart){
        serial_Debug.print(",");
    }
    serial_Debug.print("\"");
    serial_Debug.print(varName);
    serial_Debug.print("\":");
    serial_Debug.print(value,DEC);
    
    if(!isDebugOnly){
#ifdef serial_IO
        if(!isJsonStart){
            serial_IO.print(",");
        }
        serial_IO.print("\"");
        serial_IO.print(varName);
        serial_IO.print("\":");
        serial_IO.print(value,DEC);
#endif
    }
    isJsonStart = false;
}

void SerialComm::endJsonMsg(bool isDebugOnly){
    serial_Debug.println("}");
    if(!isDebugOnly) {
#ifdef serial_IO
        serial_IO.println("}");
#endif
    }
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

