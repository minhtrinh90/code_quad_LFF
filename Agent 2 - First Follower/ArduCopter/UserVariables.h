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
#define ID_NUM        2      // The 1st follower - Agent 2

#define AGENTID_0_HIGH  0x0013A200
#define AGENTID_0_LOW   0x40AE1A3E

#define AGENTID_1_HIGH  0x0013A200
#define AGENTID_1_LOW   0x40AE1A2B

#define AGENTID_2_HIGH  0x0013A200
#define AGENTID_2_LOW   0x40AE1A48

#define AGENTID_3_HIGH  0x0013A200
#define AGENTID_3_LOW   0x40AE1A2D

#define AGENTID_4_HIGH  0x0013A200
#define AGENTID_4_LOW   0x40AE1A4F

#define AGENTID_5_HIGH  0x0013A200
#define AGENTID_5_LOW   0x40AE1A4E

#define AGENT_0       0
#define AGENT_1       1
#define AGENT_2       2
#define AGENT_3       3
#define AGENT_4       4
#define AGENT_5       5

#define MAX_XBQ       26

#define GPS_DATA      1
#define MODE_CHANGE   2

#define STATE_INITIALIZED   1
#define STATE_ARMED         2
#define STATE_TAKEOFF       3
#define STATE_CONTROLLED    4
#define STATE_LANDING       5
#define STATE_DISARMED      6
//the desired bearing vector
#define bearing_x_1d 1.0
#define bearing_y_1d 0.0

uint8_t New_Flight_mode_ON;
uint32_t mediumloop_counter;

uint8_t Safe_count;

float control_input_x;
float control_input_y;

float bearing_x_1;
float bearing_y_1;

//XBee data
typedef union {
  uint32_t  data[2];
  uint8_t   b[8];
}xbee_data;

//XBee packet
typedef struct {
  uint8_t p[27];
}xbee_packet;

//XBee receive Queue
typedef struct {
  uint8_t  packet[MAX_XBQ];
  int  front;
  int  rear;
}xbee_queue;

static xbee_data	transmit_data;
static xbee_data	destination_ID;
static xbee_data	xb_data_f_l;    //data from leader
static xbee_packet	xb_packet;

static xbee_queue  queue_f_l;  //queue from leader
static uint8_t my_count;

float lat_dist_from_leader;
float lng_dist_from_leader;
static float lat_d, lng_d;


//================================================================================


#endif  // USERHOOK_VARIABLES






