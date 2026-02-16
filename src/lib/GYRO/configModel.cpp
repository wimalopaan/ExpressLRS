#include "config.h"
#include "gyro.h"


static char *wingTypeName[]={"Empty","Normal","2-Ail","Delta"};
static char *tailTypeName[]={"Empty","Normal","V-Tail","Taileron","Rud-Only"};


void quickModelSetup(int wingType, int tailType) {

  config.SetGyroDefaults(false);
  
  config.SetGyroChannel(8, FN_IN_GYRO_MODE, FN_NONE, false); // Mode
  config.SetGyroChannel(9, FN_IN_GYRO_GAIN, FN_NONE, false); // Gain

  switch (wingType) {
    case 0: break;  // Empty
    case 2: // 2-Aileron
        config.SetGyroChannel(5, FN_IN_ROLL, FN_AILERON, false); // Ail2
        // continue to normal
    case 1: // Normal
        config.SetGyroChannel(0, FN_IN_ROLL, FN_AILERON, false); // Ail
        break;
    case 3: // DELTA
        config.SetGyroChannel(0, FN_IN_ROLL, FN_ELEVON_L, false); // Ail
        config.SetGyroChannel(1, FN_IN_PITCH, FN_ELEVON_R, false); // Ele
        break;
  }

  switch (tailType) {
    case 0: break;  // Empty
    case 1: // Normal
        config.SetGyroChannel(1, FN_IN_PITCH, FN_ELEVATOR, false); // Ele
        config.SetGyroChannel(3, FN_IN_YAW, FN_RUDDER, false); // Rud
        break;
    case 2: //Vtail
        config.SetGyroChannel(1, FN_IN_PITCH, FN_VTAIL_R, false); // Ele
        config.SetGyroChannel(3, FN_IN_YAW, FN_VTAIL_L, false);   // Rud
        break;
    case 3: // Taileron
        config.SetGyroChannel(0, FN_IN_ROLL, FN_ELEVON_L, false); // Ail
        config.SetGyroChannel(1, FN_IN_PITCH, FN_ELEVON_R, false); // Ele
        config.SetGyroChannel(3, FN_IN_YAW, FN_RUDDER, false); // Rud
        break;
    case 4: //Rudder only
        config.SetGyroChannel(3, FN_IN_YAW, FN_RUDDER, false); // Rud
        break;
  }

  config.Commit();
  gyro.reload();
}