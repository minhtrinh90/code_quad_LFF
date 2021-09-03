/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA


//============================ added =============================================
#define ID_NUM        1      // the leader - agent 1

#define AGENT_0       0
#define AGENT_1       1
#define AGENT_2       2
#define AGENT_3       3
#define AGENT_4       4
#define AGENT_5       5
#define AGENT_6       6
#define AGENT_7       7


#define GPS_DATA      1
#define MODE_CHANGE   2

#define STATE_INITIALIZED   1
#define STATE_ARMED         2
#define STATE_TAKEOFF       3
#define STATE_CONTROLLED    4
#define STATE_LANDING       5
#define STATE_DISARMED      6


uint8_t New_Flight_mode_ON;
uint32_t mediumloop_counter;

//XBee data
typedef union {
  uint32_t  data[2];
  uint8_t   b[8];
}xbee_data;

//XBee packet
typedef struct {
  uint8_t p[27];
}xbee_packet;

static xbee_data	transmit_data;
static xbee_data        destination_ID;
static xbee_packet	xb_packet;
static uint32_t     my_count;

//================================================================================


#endif  // USERHOOK_VARIABLES





