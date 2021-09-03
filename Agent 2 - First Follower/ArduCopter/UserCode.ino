/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//    First follower agent
//=========================================================== added ==================================================================
void do_takeoff_user()
{
    //cliSerial->printf_P(PSTR("do_takeoff_user start\r\n"));
  
    // set roll-pitch mode
    set_roll_pitch_mode(AUTO_RP);
//    set_roll_pitch_mode(ROLL_PITCH_LOITER_USER);

    // set yaw mode
    set_yaw_mode(YAW_HOLD);

    // set throttle mode to AUTO although we should already be in this mode
    set_throttle_mode(THROTTLE_AUTO);

    // set our nav mode to loiter
    set_nav_mode(NAV_WP);

    // Set wp navigation target to safe altitude above current position
    Vector3f pos = inertial_nav.get_position();
    pos.z = 200.0;//max(pos.z, 100);
    wp_nav.set_destination(pos);

    // prevent flips
    // To-Do: check if this is still necessary
    reset_I_all();    
}


/////////////transfer
uint32_t xbee_make_idpadding_mask_high(int id_num)
{
  uint32_t output;

  switch(id_num)
  {
    case 0:
    output = AGENTID_0_HIGH;
    break;

    case 1:
    output = AGENTID_1_HIGH;
    break;

    case 2:
    output = AGENTID_2_HIGH;
    break;

    case 3:
    output = AGENTID_3_HIGH;
    break;
    
    case 4:
    output = AGENTID_4_HIGH;
    break;
    
    case 5:
    output = AGENTID_5_HIGH;
    break;
  }

  return output;
}


uint32_t xbee_make_idpadding_mask_low(int id_num)
{
  uint32_t output;

  switch(id_num)
  {
    case 0:
    output = AGENTID_0_LOW;
    break;
	
    case 1:
    output = AGENTID_1_LOW;
    break;
    
    case 2:
    output = AGENTID_2_LOW;
    break;
    
    case 3:
    output = AGENTID_3_LOW;
    break;

    case 4:
    output = AGENTID_4_LOW;
    break;
    
    case 5:
    output = AGENTID_5_LOW;
    break;
  }

  return output;
}

void xbee_data_to_packet(int id_num, uint8_t mode)
{
  uint8_t temp_byte = 0x00;
  uint8_t checksum_byte = 0x00;
  uint8_t index = 0;

  destination_ID.data[0] = xbee_make_idpadding_mask_high(id_num);
  destination_ID.data[1] = xbee_make_idpadding_mask_low(id_num);

  for(int i=0; i<27; i++)
  {
    xb_packet.p[i] &= 0x00;
  }

  xb_packet.p[index++] |= 0x7E;
  xb_packet.p[index++] |= 0x00;
  xb_packet.p[index++] |= 0x17;

  xb_packet.p[index] |= 0x10;
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= 0x00;
  checksum_byte += xb_packet.p[index++];

  xb_packet.p[index] |= destination_ID.b[3];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= destination_ID.b[2];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= destination_ID.b[1];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= destination_ID.b[0];
  checksum_byte += xb_packet.p[index++];

  xb_packet.p[index] |= destination_ID.b[7];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= destination_ID.b[6];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= destination_ID.b[5];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= destination_ID.b[4];
  checksum_byte += xb_packet.p[index++];

  xb_packet.p[index] |= 0xFF;
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= 0xFE;
  checksum_byte += xb_packet.p[index++];

  xb_packet.p[index] |= 0x00;
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= 0x01;
  checksum_byte += xb_packet.p[index++];

  xb_packet.p[index] |= mode;
  checksum_byte += xb_packet.p[index++];

  xb_packet.p[index] |= transmit_data.b[3];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= transmit_data.b[2];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= transmit_data.b[1];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= transmit_data.b[0];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= transmit_data.b[7];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= transmit_data.b[6];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= transmit_data.b[5];
  checksum_byte += xb_packet.p[index++];
  xb_packet.p[index] |= transmit_data.b[4];
  checksum_byte += xb_packet.p[index++];

  xb_packet.p[index] |= ~(checksum_byte & 0xff);

  for(int i=0; i<27; i++)
  {
    cliSerial->printf_P(PSTR("%c"), xb_packet.p[i]);
  }
}
///////////transfer


//////////receive
void xbee_clear_queue()
{
  queue_f_l.front = 0;
  queue_f_l.rear  = 0;
}

void xbee_insert_queue(uint8_t seg)
{
  int f = queue_f_l.front;
  int r = queue_f_l.rear;

  if( f == r )  //if queue is empty, check whether this is first segment.
  {
    if( seg == 0x7E )
    {
      queue_f_l.packet[r] = seg;
      queue_f_l.rear = (r+1)%MAX_XBQ;
    }
  }
  else
  {
    queue_f_l.packet[r] = seg;
    queue_f_l.rear = (r+1)%MAX_XBQ;
  }

  if( (queue_f_l.rear+1)%MAX_XBQ == queue_f_l.front)    //full
  {
    xbee_packet_to_data();
    xbee_clear_queue();
  }
}

void xbee_packet_to_data()
{
  xbee_data result;
  uint8_t ch_sum = 0x00;
  uint8_t mode_received;

  int n = queue_f_l.front;

  for(int i=n+3; i<n+MAX_XBQ-2; i++)
  {
    ch_sum += (queue_f_l.packet[i%MAX_XBQ] & 0xff);
  }

  if( (~ch_sum & 0xff) == (queue_f_l.packet[(n+24)%MAX_XBQ] & 0xff) )
  {
//    if(Safe_count>0)  Safe_count--;
//    if(Safe_count>0)  Safe_count--;

    for(int i=0; i<8; i++)
    {
      result.b[i] = 0x00;
    }

    n = (n+15)%MAX_XBQ;    //exepts start segment

    mode_received = (queue_f_l.packet[n]);
    n = ++n%MAX_XBQ;

    result.b[3] |= (queue_f_l.packet[n]);
    n = ++n%MAX_XBQ;
    result.b[2] |= (queue_f_l.packet[n]);
    n = ++n%MAX_XBQ;
    result.b[1] |= (queue_f_l.packet[n]);
    n = ++n%MAX_XBQ;
    result.b[0] |= (queue_f_l.packet[n]);
    n = ++n%MAX_XBQ;

    result.b[7] |= (queue_f_l.packet[n]);
    n = ++n%MAX_XBQ;
    result.b[6] |= (queue_f_l.packet[n]);
    n = ++n%MAX_XBQ;
    result.b[5] |= (queue_f_l.packet[n]);
    n = ++n%MAX_XBQ;
    result.b[4] |= (queue_f_l.packet[n]);

    if(mode_received==GPS_DATA)
    {  
      xb_data_f_l.data[0] = result.data[0];
      xb_data_f_l.data[1] = result.data[1];
    }
    else if(mode_received==MODE_CHANGE)
    {
      if(result.data[0]==STATE_INITIALIZED)
      {
        if(New_Flight_mode_ON != STATE_INITIALIZED)
        {
            New_Flight_mode_ON = STATE_INITIALIZED;
        }
      }
      else if(result.data[0]==STATE_ARMED)
      {
        if(New_Flight_mode_ON != STATE_ARMED)
        {
              mediumloop_counter = 0;
              
              New_Flight_mode_ON = STATE_ARMED;
              
              init_arm_motors();
        }
      }
      else if(result.data[0]==STATE_TAKEOFF)
      {
        if((New_Flight_mode_ON==STATE_ARMED) || (New_Flight_mode_ON==STATE_LANDING)) 
        {
            mediumloop_counter = 0;
            
            New_Flight_mode_ON = STATE_TAKEOFF;
        }
      }
      else if(result.data[0]==STATE_CONTROLLED)
      {
            New_Flight_mode_ON = STATE_CONTROLLED;
      }
      else if(result.data[0]==STATE_LANDING)
      {
        mediumloop_counter = 0;
        
        New_Flight_mode_ON = STATE_LANDING;
      }
      else if(result.data[0]==STATE_DISARMED)
      {
        if(New_Flight_mode_ON != STATE_DISARMED)
        {
          mediumloop_counter = 0;
          
          New_Flight_mode_ON = STATE_DISARMED;
        }
      }
      else
      {
        mediumloop_counter = 0;
        
        New_Flight_mode_ON = STATE_LANDING;
      }
    }
    else
    {
      mediumloop_counter = 0;
      
      New_Flight_mode_ON = STATE_LANDING;
    }
  }
  else
  {
    int f = queue_f_l.front;
    int r = queue_f_l.rear;

    f = (f+1)%MAX_XBQ;

    while( (queue_f_l.packet[f] != 0x7E)  &&  (f != r) )        //@@If we can modity, do it.
    {
      f = (f+1)%MAX_XBQ;
    }

    queue_f_l.front = f;
  }
}

//============================================================== orignal ==============================================================
#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
  transmit_data.data[0] = 0;
  transmit_data.data[1] = 0;    
  xb_data_f_l.data[0] = 0;
  xb_data_f_l.data[1] = 0;    
  lat_dist_from_leader = 0;
  lng_dist_from_leader = 0;
  
  New_Flight_mode_ON = STATE_INITIALIZED;
  mediumloop_counter = 0;
  Safe_count = 0;

  control_input_x = 0.0;
  control_input_y = 0.0;
//
//  // cm
//  desired_distance = 700.0;

//  ez_x = 0;
//  ez_y = 0;
//  estimated_velocity_x = 0.0;
//  estimated_velocity_y = 0.0;

  xbee_clear_queue();
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
  // put your 10Hz code here  //@@transfer all segments  
  if(!ap_system.usb_connected)
  {
//    Safe_count++;
    
      if(New_Flight_mode_ON==STATE_INITIALIZED)
      {
          mediumloop_counter = 0;

          transmit_data.data[0] = g_gps->latitude%1000000;
          transmit_data.data[1] = g_gps->longitude%1000000;
          xbee_data_to_packet(AGENT_0,GPS_DATA);

      }
      else if(New_Flight_mode_ON==STATE_ARMED)
      {
          if(mediumloop_counter==0) 
          {
            mediumloop_counter++;
            
            transmit_data.data[0] = STATE_ARMED;
            transmit_data.data[1] = 0;
            xbee_data_to_packet(AGENT_0,MODE_CHANGE);
            xbee_data_to_packet(AGENT_3,MODE_CHANGE);
            xbee_data_to_packet(AGENT_4,MODE_CHANGE);
            g.rc_3.control_in = 1;
          }
         else g.rc_3.control_in = 0;    
          transmit_data.data[0] = STATE_ARMED;
          transmit_data.data[1] = 0;
          xbee_data_to_packet(AGENT_0,MODE_CHANGE);
          xbee_data_to_packet(AGENT_3,MODE_CHANGE);
          transmit_data.data[0] = lat_d;
          transmit_data.data[1] = lng_d;
      }
      else if(New_Flight_mode_ON==STATE_TAKEOFF)
      {
          if(transmit_data.data[0] != STATE_TAKEOFF)
          {
            do_takeoff_user();
          }
    
          mediumloop_counter++;
    
          if(mediumloop_counter<20)
          {
            transmit_data.data[0] = STATE_TAKEOFF;
            transmit_data.data[1] = 0;
            xbee_data_to_packet(AGENT_0,MODE_CHANGE);
            xbee_data_to_packet(AGENT_3,MODE_CHANGE);
            xbee_data_to_packet(AGENT_3,MODE_CHANGE);
          }
          else if(mediumloop_counter<40)
          {
              transmit_data.data[0] = current_loc.lat%1000000;   //get gps
              transmit_data.data[1] = current_loc.lng%1000000;
              xbee_data_to_packet(AGENT_0,GPS_DATA);
              xbee_data_to_packet(AGENT_3,GPS_DATA);
              xbee_data_to_packet(AGENT_4,GPS_DATA);
          }
          else if(mediumloop_counter<50)
          {
//            g.rc_1.control_in = 0;
//            g.rc_2.control_in = 0;
              set_mode(LOITER);
              
              transmit_data.data[0] = current_loc.lat%1000000;   //get gps
              transmit_data.data[1] = current_loc.lng%1000000;
              xbee_data_to_packet(AGENT_0,GPS_DATA);
              xbee_data_to_packet(AGENT_3,GPS_DATA);
              xbee_data_to_packet(AGENT_4,GPS_DATA);
          }
          else
          {
              mediumloop_counter = 0;
              New_Flight_mode_ON = STATE_CONTROLLED;
              
            // Controller
              set_roll_pitch_mode(ROLL_PITCH_AUTO);
              set_yaw_mode(YAW_HOLD);
              set_throttle_mode(THROTTLE_HOLD_USER);
              set_nav_mode(NAV_NONE);
            // fixed
//             set_roll_pitch_mode(ROLL_PITCH_LOITER);
//              set_yaw_mode(YAW_HOLD);
//              set_throttle_mode(THROTTLE_HOLD_USER);
//             set_nav_mode(LOITER_NAV);
          }
      }
      else if(New_Flight_mode_ON==STATE_CONTROLLED)
      {
          mediumloop_counter = 0;
          
          lat_dist_from_leader = 0.0001*((float)(xb_data_f_l.data[0]) - (float)(current_loc.lat%1000000));
          lng_dist_from_leader = 0.0001*((float)(xb_data_f_l.data[1]) - (float)(current_loc.lng%1000000));
          
          transmit_data.data[0] = current_loc.lat%1000000;   //get gps
          transmit_data.data[1] = current_loc.lng%1000000;
          xbee_data_to_packet(AGENT_0, GPS_DATA);
          xbee_data_to_packet(AGENT_3, GPS_DATA);
          xbee_data_to_packet(AGENT_4, GPS_DATA);

          bearing_x_1 = 8*lat_dist_from_leader/sqrt(lat_dist_from_leader*lat_dist_from_leader + lng_dist_from_leader*lng_dist_from_leader);
          bearing_y_1 = 8*lng_dist_from_leader/sqrt(lat_dist_from_leader*lat_dist_from_leader + lng_dist_from_leader*lng_dist_from_leader);
          
          control_input_x = 0.5*(-bearing_x_1d * bearing_y_1 + bearing_y_1d * bearing_x_1)*bearing_y_1;
          control_input_y = -0.5*(-bearing_x_1d * bearing_y_1 + bearing_y_1d * bearing_x_1)*bearing_x_1;
        
          if(control_input_x > 300.0) control_input_x = 300.0;
          else if(control_input_x < -300.0) control_input_x = -300.0;
          
          if(control_input_y > 300.0) control_input_y = 300.0;
          else if(control_input_y < -300.0) control_input_y = -300.0;
                      
          wp_nav.get_loiter_velocity_to_acceleration(control_input_x,control_input_y,0.1);
        
      }
      else if(New_Flight_mode_ON==STATE_LANDING)
      {
            if(mediumloop_counter==0)
            {
                mediumloop_counter++;                
                set_mode(LAND);
            }
            transmit_data.data[0] = STATE_LANDING;
            transmit_data.data[1] = 0;
            xbee_data_to_packet(AGENT_0,MODE_CHANGE);
            xbee_data_to_packet(AGENT_3,MODE_CHANGE);
            xbee_data_to_packet(AGENT_4,MODE_CHANGE);
      }
      else if(New_Flight_mode_ON==STATE_DISARMED)
      {
            if(transmit_data.data[0] != STATE_DISARMED)
            {
                init_disarm_motors();
            }
      
              transmit_data.data[0] = STATE_DISARMED;
              transmit_data.data[1] = 0;
              xbee_data_to_packet(AGENT_0,MODE_CHANGE);
              xbee_data_to_packet(AGENT_3,MODE_CHANGE);
              xbee_data_to_packet(AGENT_4,MODE_CHANGE);
      }
      else
      {
            mediumloop_counter = 0;
            transmit_data.data[0] = 119;
            transmit_data.data[1] = 119;
            xbee_data_to_packet(AGENT_0,GPS_DATA);
            xbee_data_to_packet(AGENT_3,MODE_CHANGE); 
            xbee_data_to_packet(AGENT_4,MODE_CHANGE);
      }
      
     if(New_Flight_mode_ON!=STATE_INITIALIZED && New_Flight_mode_ON!=STATE_ARMED && New_Flight_mode_ON!=STATE_LANDING && (Safe_count>10))
     {
       Safe_count = 0;
       
       mediumloop_counter = 0;
       
       New_Flight_mode_ON = STATE_LANDING;
     }

  }
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
