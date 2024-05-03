#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>

#include "cs104_slave.h"
#include "hal_thread.h"
#include "hal_time.h"
#include <time.h>

#define MAX_MESSAGES 100

typedef struct {
    int messageType;
    int ioa;
    float value; // Používáme float pro všechny typy hodnot
} MessageConfig;

static MessageConfig messageConfigs[MAX_MESSAGES];
static int numMessageConfigs = 0;

static bool running = true;
static time_t lastSentTime = 0;
static bool spontaneousEnabled = false;
static int minSpontaneousInterval = 2; // defaultní minimální interval (sekundy)
static int maxSpontaneousInterval = 10; // defaultní maximální interval (sekundy)
static time_t nextSpontaneousTime = 0;
static int multiplier = 1;  // Defaultní hodnota

void sigint_handler(int signalId)
{
    running = false;
}

char* readConfigValue(const char* filePath, const char* key) {
    FILE* file = fopen(filePath, "r");
    if (file == NULL) {
        perror("Failed to open file");
        return NULL;
    }

    char* value = NULL;
    char line[256];
    size_t keyLen = strlen(key);

    while (fgets(line, sizeof(line), file)) {
        char* pos = strstr(line, key);
        if (pos != NULL && line[keyLen] == '=') {
            char* startOfValue = pos + keyLen + 1;
            char* endOfValue = startOfValue + strcspn(startOfValue, "\r\n");
            *endOfValue = '\0';
            value = strdup(startOfValue);
            break;
        }
    }

    fclose(file);
    return value;
}

#define MAX_ASDU 30
static CS101_ASDU asdu_buffer[MAX_ASDU];
static int num_asdu = 0;
#define MAX_IO 100
static InformationObject io_buffer[MAX_IO];
static int num_io = 0;

// Funkce pro vytvoření IO (Information Object) podle typu zprávy
InformationObject createIO(int messageType, int ioa, float value) {
    InformationObject io;
    switch (messageType) {
        case 1:
            io = (InformationObject)SinglePointInformation_create(NULL, ioa, true, IEC60870_QUALITY_GOOD);
            break;
        case 3:
            io = (InformationObject)DoublePointInformation_create(NULL, ioa, true, IEC60870_QUALITY_GOOD);
            break;
        case 11:
            io = (InformationObject)MeasuredValueScaled_create(NULL, ioa, 0.5, IEC60870_QUALITY_GOOD);
            break;
        case 30:
            io = (InformationObject)SinglePointWithCP56Time2a_create(NULL, ioa, 2, IEC60870_QUALITY_GOOD, CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
        case 31:
            io = (InformationObject)DoublePointWithCP56Time2a_create(NULL, ioa, 2, IEC60870_QUALITY_GOOD, CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
        case 36:
            io = (InformationObject)MeasuredValueShortWithCP56Time2a_create(NULL, ioa, 250.12, IEC60870_QUALITY_GOOD, CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
            break;
    }
    return io;
}

void readMessageConfig(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        perror("Failed to open configuration file");
        return;
    }

    char line[256];
    int messageType;
    int ioa;
    float value;
    bool startReading = false;  // Kontrola, zda máme začít číst konfigurační řádky
    while (fgets(line, sizeof(line), file)) {
        if (!startReading) {
            // Hledáme klíč "MESS="
            if (strncmp(line, "MESS=", 5) == 0) {
                startReading = true;  // Nalezen klíč, začneme číst data
            }
        } else {
            // Čtení dat po nalezení klíče
            if (line[0] == '\0') {
                // Přeskočí prázdné řádky nebo řádky s komentáři
                break;
            }
            MessageConfig config;
            if (sscanf(line, "%d;%d;%f", &messageType, &ioa, &value) == 3) {
                if (ioa > 655535) {
                    printf("%d is not a valid ioa value", ioa);
                    continue;
                }
                io_buffer[num_io] = createIO(messageType, ioa, value);
                ++num_io;
            }
        }
    }
    fclose(file);
}

// Funkce vracející ukazatel na pole struktur
MessageConfig* getMessageConfigs(int* count) {
    *count = numMessageConfigs;
    return messageConfigs;
}



void sendSpontaneousMessage(CS104_Slave slave, CS101_AppLayerParameters alParams, int multiplier) {
    for (int i = 0; i < multiplier; i++) {
        CS101_ASDU asdu = CS101_ASDU_create(alParams, false, CS101_COT_SPONTANEOUS, 0, 1, false, false);
        InformationObject io = (InformationObject) SinglePointInformation_create(NULL, 1001, rand() % 2, IEC60870_QUALITY_GOOD);
        CS101_ASDU_addInformationObject(asdu, io);
        InformationObject_destroy(io);
        CS104_Slave_enqueueASDU(slave, asdu);
        CS101_ASDU_destroy(asdu);
    }

    printf("Spontaneous messages sent count: %d at %s\n", multiplier, ctime(&nextSpontaneousTime));
}

void configureSpontaneousMessages(const char* config) {
    char* configCopy = strdup(config);
    char* token = strtok(configCopy, ";");
    if (token && atoi(token) == 1) {
        spontaneousEnabled = true;
        token = strtok(NULL, ";");
        minSpontaneousInterval = atoi(token);
        token = strtok(NULL, ";");
        maxSpontaneousInterval = atoi(token);
    }
    free(configCopy);
}

void scheduleNextSpontaneousMessage() {
    if (!spontaneousEnabled) return;
    int interval = minSpontaneousInterval + rand() % (maxSpontaneousInterval - minSpontaneousInterval + 1);
    nextSpontaneousTime = time(NULL) + interval;
}

void sendPeriodicMessages(CS104_Slave slave, CS101_AppLayerParameters alParams, int multiplier)
{
    for (int i = 0; i < multiplier; i++) {
        CS101_ASDU asdu = CS101_ASDU_create(alParams, false, CS101_COT_PERIODIC, 0, 1, false, false);
        InformationObject io = (InformationObject) SinglePointInformation_create(NULL, 1001, true, IEC60870_QUALITY_GOOD);
        CS101_ASDU_addInformationObject(asdu, io);
        InformationObject_destroy(io);
        CS104_Slave_enqueueASDU(slave, asdu);
        CS101_ASDU_destroy(asdu);
    }

    printf("Periodic messages sent count: %d\n", multiplier);
}

void logMessage(FILE* logFile, const char* message) {
    if (logFile) {
        time_t now = time(NULL);  // Nyní korektně deklarováno
        char* timeStr = ctime(&now);
        timeStr[strcspn(timeStr, "\n")] = 0; // Odstranění nového řádku z časového řetězce
        fprintf(logFile, "%s: %s\n", timeStr, message);
        fflush(logFile); // Zajistí okamžitý zápis do souboru
    }
}

void
printCP56Time2a(CP56Time2a time)
{
    printf("%02i:%02i:%02i %02i/%02i/%04i", CP56Time2a_getHour(time),
           CP56Time2a_getMinute(time),
           CP56Time2a_getSecond(time),
           CP56Time2a_getDayOfMonth(time),
           CP56Time2a_getMonth(time),
           CP56Time2a_getYear(time) + 2000);
}



/* Callback handler to log sent or received messages (optional) */
static void
rawMessageHandler(void* parameter, IMasterConnection conneciton, uint8_t* msg, int msgSize, bool sent)
{
    if (sent)
        printf("SEND: ");
    else
        printf("RCVD: ");

    int i;
    for (i = 0; i < msgSize; i++) {
        printf("%02x ", msg[i]);
    }

    printf("\n");
}

static bool
clockSyncHandler (void* parameter, IMasterConnection connection, CS101_ASDU asdu, CP56Time2a newTime)
{
    printf("Process time sync command with time "); printCP56Time2a(newTime); printf("\n");

    uint64_t newSystemTimeInMs = CP56Time2a_toMsTimestamp(newTime);

    /* Set time for ACT_CON message */
    CP56Time2a_setFromMsTimestamp(newTime, Hal_getTimeInMs());

    /* update system time here */

    return true;
}

static bool
interrogationHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu, uint8_t qoi)
{
    printf("Received interrogation for group %i\n", qoi);

    if (qoi == 22) { /* only handle station interrogation */

        CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);

        IMasterConnection_sendACT_CON(connection, asdu, false);

        /* The CS101 specification only allows information objects without timestamp in GI responses */

        CS101_ASDU newAsdu1, newAsdu3, newAsdu7, newAsdu11, newAsdu30, newAsdu31, newAsdu36;

        /*vytvorit asdu, pro kazdej typ novou asdu (muzes pretypovavat ale nedoporucuju)*/
        newAsdu1 = CS101_ASDU_create(alParams, true, CS101_COT_INTERROGATED_BY_STATION, 0, 1, false, false);
        newAsdu3 = CS101_ASDU_create(alParams, true, CS101_COT_INTERROGATED_BY_STATION, 0, 1, false, false);
        newAsdu7 = CS101_ASDU_create(alParams, false, CS101_COT_INTERROGATED_BY_STATION, 0, 1, false, false);
        newAsdu11 = CS101_ASDU_create(alParams, true, CS101_COT_INTERROGATED_BY_STATION, 0, 1, false, false);
        newAsdu30 = CS101_ASDU_create(alParams, true, CS101_COT_INTERROGATED_BY_STATION, 0, 1, false, false);
        newAsdu31 = CS101_ASDU_create(alParams, true, CS101_COT_INTERROGATED_BY_STATION, 0, 1, false, false);
        newAsdu36 = CS101_ASDU_create(alParams, true, CS101_COT_INTERROGATED_BY_STATION, 0, 1, false, false);


        /*vytvorit ioa*/
        InformationObject io1 = (InformationObject) SinglePointInformation_create(NULL, 1000, true, IEC60870_QUALITY_GOOD);
        InformationObject io3 = (InformationObject) DoublePointInformation_create(NULL, 1500, true, IEC60870_QUALITY_GOOD);
        InformationObject io7 = (InformationObject) BitString32_create(NULL, 659, 0xaaaa);
        InformationObject io11 = (InformationObject) MeasuredValueScaled_create(NULL, 112, 0.5, IEC60870_QUALITY_GOOD);
        InformationObject io30 = (InformationObject) SinglePointWithCP56Time2a_create(NULL, 3500, 2, IEC60870_QUALITY_GOOD,CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
        InformationObject io31 = (InformationObject) DoublePointWithCP56Time2a_create(NULL, 4000, 2, IEC60870_QUALITY_GOOD,CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));
        InformationObject io36 = (InformationObject) MeasuredValueShortWithCP56Time2a_create(NULL, 3002, 250.12, IEC60870_QUALITY_GOOD, CP56Time2a_createFromMsTimestamp(NULL, Hal_getTimeInMs()));


        /*priradis io do asdu*/
        CS101_ASDU_addInformationObject(newAsdu1, io1);
        CS101_ASDU_addInformationObject(newAsdu3, io3);
        CS101_ASDU_addInformationObject(newAsdu7, io7);
        CS101_ASDU_addInformationObject(newAsdu11, io11);
        CS101_ASDU_addInformationObject(newAsdu30, io30);
        CS101_ASDU_addInformationObject(newAsdu31, io31);
        CS101_ASDU_addInformationObject(newAsdu36, io36);





        /*odesles*/
        IMasterConnection_sendASDU(connection, newAsdu1);
        IMasterConnection_sendASDU(connection, newAsdu3);
        IMasterConnection_sendASDU(connection, newAsdu7);
        IMasterConnection_sendASDU(connection, newAsdu11);
        IMasterConnection_sendASDU(connection, newAsdu30);
        IMasterConnection_sendASDU(connection, newAsdu31);
        IMasterConnection_sendASDU(connection, newAsdu36);


        /*uvolnis misto v pameti*/
        InformationObject_destroy(io1);
        InformationObject_destroy(io3);
        InformationObject_destroy(io7);
        InformationObject_destroy(io11);
        InformationObject_destroy(io30);
        InformationObject_destroy(io31);
        InformationObject_destroy(io36);
        CS101_ASDU_destroy(newAsdu1);
        CS101_ASDU_destroy(newAsdu3);
        CS101_ASDU_destroy(newAsdu7);
        CS101_ASDU_destroy(newAsdu11);
        CS101_ASDU_destroy(newAsdu30);
        CS101_ASDU_destroy(newAsdu31);
        CS101_ASDU_destroy(newAsdu36);

        /*sem jsem něco dopsal*/

    }
    else {
        IMasterConnection_sendACT_CON(connection, asdu, true);
    }

    return true;
}

static bool
asduHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu)
{
    if (CS101_ASDU_getTypeID(asdu) == C_SC_NA_1) {
        printf("received single command\n");

        if  (CS101_ASDU_getCOT(asdu) == CS101_COT_ACTIVATION) {
            InformationObject io = CS101_ASDU_getElement(asdu, 0);

            if (io) {
                if (InformationObject_getObjectAddress(io) == 5000) {
                    SingleCommand sc = (SingleCommand) io;

                    printf("IOA: %i switch to %i\n", InformationObject_getObjectAddress(io),
                           SingleCommand_getState(sc));

                    CS101_ASDU_setCOT(asdu, CS101_COT_ACTIVATION_CON);
                }
                else
                    CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA);

                InformationObject_destroy(io);
            }
            else {
                printf("ERROR: message has no valid information object\n");
                return true;
            }
        }
        else
            CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_COT);

        IMasterConnection_sendASDU(connection, asdu);

        return true;
    }

    return false;
}

static bool
connectionRequestHandler(void* parameter, const char* ipAddress)
{
    printf("New connection request from %s\n", ipAddress);

#if 0
    if (strcmp(ipAddress, "127.0.0.1") == 0) {
        printf("Accept connection\n");
        return true;
    }
    else {
        printf("Deny connection\n");
        return false;
    }
#else
    return true;
#endif
}

static void
connectionEventHandler(void* parameter, IMasterConnection con, CS104_PeerConnectionEvent event)
{
    if (event == CS104_CON_EVENT_CONNECTION_OPENED) {
        printf("Connection opened (%p)\n", con);
    }
    else if (event == CS104_CON_EVENT_CONNECTION_CLOSED) {
        printf("Connection closed (%p)\n", con);
    }
    else if (event == CS104_CON_EVENT_ACTIVATED) {
        printf("Connection activated (%p)\n", con);
    }
    else if (event == CS104_CON_EVENT_DEACTIVATED) {
        printf("Connection deactivated (%p)\n", con);
    }
}

int
main(int argc, char** argv)
{
    /* Add Ctrl-C handler */
    signal(SIGINT, sigint_handler);

    lastSentTime = time(NULL);
    char* ip = readConfigValue("/home/klient/Desktop/KONFIGSERVER104.txt", "IP config");
    char* interface = readConfigValue("/home/klient/Desktop/KONFIGSERVER104.txt", "Interface");
    char* portStr = readConfigValue("/home/klient/Desktop/KONFIGSERVER104.txt", "Port");
    char* originatorAddressStr = readConfigValue("/home/klient/Desktop/KONFIGSERVER104.txt", "Originator Address");
    char* commonAddressStr = readConfigValue("/home/klient/Desktop/KONFIGSERVER104.txt", "Common Address");
    char* logs = readConfigValue("/home/klient/Desktop/KONFIGSERVER104.txt", "LOGS");
    char* spontaneousConfig = readConfigValue("/home/klient/Desktop/KONFIGSERVER104.txt", "SPONTANEOUS");
    char* multiplierStr = readConfigValue("/home/klient/Desktop/KONFIGSERVER104.txt", "MULTI");
    readMessageConfig("/home/klient/Desktop/KONFIGSERVER104.txt");
    for (int i = 0; i < numMessageConfigs; i++) {
        printf("Message Type: %d, IOA: %d, Value: %.2f\n",
               messageConfigs[i].messageType, messageConfigs[i].ioa, messageConfigs[i].value);
    }
    FILE* logFile = NULL;

    if (!ip || !interface || !portStr || !originatorAddressStr || !commonAddressStr) {
        fprintf(stderr, "Failed to read some configuration values. Exiting...\n");
        free(ip); free(interface); free(portStr); free(originatorAddressStr); free(commonAddressStr);
        return -1;
    }

    if (logs && atoi(logs) == 1) {
        logFile = fopen("/home/klient/Desktop/LOGS.txt", "a"); // Append mode
        if (!logFile) {
            perror("Failed to open log file");
            return -1;
        }
    }

    if (spontaneousConfig) {
        configureSpontaneousMessages(spontaneousConfig);
        free(spontaneousConfig);
        scheduleNextSpontaneousMessage();
    }

    if (multiplierStr) {
        multiplier = atoi(multiplierStr);
        free(multiplierStr);
    }

    // Načtení konfiguračních hodnot
    char* periodStr = readConfigValue("/home/klient/Desktop/KONFIGSERVER104.txt", "PERIOD");
    int periodicInterval = periodStr ? atoi(periodStr) : 20;  // Defaultní perioda je 20 sekund, pokud není specifikováno jinak
    free(periodStr);

    int port = atoi(portStr);
    int originatorAddress = atoi(originatorAddressStr);
    int commonAddress = atoi(commonAddressStr);

    //vytvorit asdu a naplnit se io

    printf("IP Address: %s\n", ip);
    printf("Interface: %s\n", interface);
    printf("Port: %d\n", port);
    printf("Originator Address: %d\n", originatorAddress);
    printf("Common Address: %d\n", commonAddress);

    CS104_Slave slave = CS104_Slave_create(10, 10);
    CS104_Slave_setLocalAddress(slave, ip);
    CS104_Slave_setLocalPort(slave, port);

    /* Set mode to a single redundancy group
     * NOTE: library has to be compiled with CONFIG_CS104_SUPPORT_SERVER_MODE_SINGLE_REDUNDANCY_GROUP enabled (=1)
     */
    CS104_Slave_setServerMode(slave, CS104_MODE_SINGLE_REDUNDANCY_GROUP);

    /* get the connection parameters - we need them to create correct ASDUs -
     * you can also modify the parameters here when default parameters are not to be used */
    CS101_AppLayerParameters alParams = CS104_Slave_getAppLayerParameters(slave);

    /* when you have to tweak the APCI parameters (t0-t3, k, w) you can access them here */
    CS104_APCIParameters apciParams = CS104_Slave_getConnectionParameters(slave);

    printf("APCI parameters:\n");
    printf("  t0: %i\n", apciParams->t0);
    printf("  t1: %i\n", apciParams->t1);
    printf("  t2: %i\n", apciParams->t2);
    printf("  t3: %i\n", apciParams->t3);
    printf("  k: %i\n", apciParams->k);
    printf("  w: %i\n", apciParams->w);

    /* set the callback handler for the clock synchronization command */
    CS104_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);

    /* set the callback handler for the interrogation command */
    CS104_Slave_setInterrogationHandler(slave, interrogationHandler, NULL);

    /* set handler for other message types */
    CS104_Slave_setASDUHandler(slave, asduHandler, NULL);

    /* set handler to handle connection requests (optional) */
    CS104_Slave_setConnectionRequestHandler(slave, connectionRequestHandler, NULL);

    /* set handler to track connection events (optional) */
    CS104_Slave_setConnectionEventHandler(slave, connectionEventHandler, NULL);

    /* uncomment to log messages */
    //CS104_Slave_setRawMessageHandler(slave, rawMessageHandler, NULL);

    CS104_Slave_start(slave);

    logMessage(logFile, "Server start attempted");

    int16_t scaledValue = 0;

    printf("Server will send messages every %d seconds with multiplier: %d.\n", periodicInterval, multiplier);
    lastSentTime = time(NULL);  // Nastavit čas při startu

    while (running) {
        time_t currentTime = time(NULL);

        // Odesílání periodických zpráv
        if (difftime(currentTime, lastSentTime) >= periodicInterval) {
            printf("Sending periodic messages...\n");
            for (int i = 0; i < numMessageConfigs; ++i) {
//                MessageConfig msg = messageConfigs[i];
////                CS101_ASDU newAsdu = createAsdu(msg.messageType, msg.COT, msg.value, originatorAddress, commonAddress, ioa, alParams);
//                for (int j = 0; j < multiplier; j++) {
//                    CS104_Slave_enqueueASDU(slave, newAsdu);
//                }
            }
//                InformationObject_destroy(io1);
//                InformationObject_destroy(io2);
//                InformationObject_destroy(io3);
//
//                CS101_ASDU_destroy(newAsdu1);
//                CS101_ASDU_destroy(newAsdu3);
//                CS101_ASDU_destroy(newAsdu7);
//            }
            lastSentTime = currentTime;  // Update the last sent time
        }

        // Odesílání spontánních zpráv
        if (spontaneousEnabled && currentTime >= nextSpontaneousTime) {
            for (int i = 0; i < multiplier; i++) {
                sendSpontaneousMessage(slave, alParams, multiplier);  // Now correctly passing all required parameters
            }
            scheduleNextSpontaneousMessage();  // Schedule the next spontaneous message
        }

        Thread_sleep (4000);
    }

    /*CS104_Slave_stop(slave);*/
    free(ip);
    free(interface);
    free(portStr);
    free(originatorAddressStr);
    free(commonAddressStr);
    return 0;
    /*exit_program:
    CS104_Slave_destroy(slave);
    Thread_sleep(500)*/
}