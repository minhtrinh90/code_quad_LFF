/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//    Leader agent
//=========================================================== added ==================================================================
/////////////발신

void do_takeoff_user()
{
    //cliSerial->printf_P(PSTR("do_takeoff_user start\r\n"));
  
    // set roll-pitch mode
    set_roll_pitch_mode(AUTO_RP);

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





uint32_t xbee_make_idpadding_mask_high(int id_num)
{
  uint32_t output;

  switch(id_num)
  {
    case 0:
    output = 0x0013A200;
    break;
	
    case 1:
    output = 0x0013A200;
    break;
    
    case 2:
    output = 0x0013A200;
    break;
    
    case 3:
    output = 0x0013A200;
    break;

    case 4:
    output = 0x0013A200;
    break;
    
    case 5:
    output = 0x0013A200;
    break;
    
    case 6:
    output = 0x0013A200;
    break;
    
    case 7:
    output = 0x0013A200;
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
    output = 0x40AE1A3E;
    break;
	
    case 1:
    output = 0x40AE1A2B;
    break;
    
    case 2:
    output = 0x40AE1A48;
    break;
    
    case 3:
    output = 0x40AE1A2D;
    break;
    
    case 4:
    output = 0x40AE1A4F;
    break;
    
    case 5:
    output = 0x40AE1A4E;
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

//============================================================== orignal ==============================================================
#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    New_Flight_mode_ON = STATE_INITIALIZED;
    mediumloop_counter = 0;
    my_count = 0;
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
  if(!ap_system.usb_connected)
  {
  // put your 10Hz code here  //@@transfer all segments
  if(New_Flight_mode_ON==STATE_INITIALIZED)
  {
    mediumloop_counter = 0;
    
    transmit_data.data[0] = g_gps->latitude%1000000;
    transmit_data.data[1] = g_gps->longitude%1000000;
    xbee_data_to_packet(AGENT_0,GPS_DATA);
  }
  else if(New_Flight_mode_ON==STATE_ARMED)
  {
      mediumloop_counter = 0;
      
      transmit_data.data[0] = STATE_ARMED;
      transmit_data.data[1] = 0;
      xbee_data_to_packet(AGENT_0,MODE_CHANGE);
      xbee_data_to_packet(AGENT_2,MODE_CHANGE);
      xbee_data_to_packet(AGENT_3,MODE_CHANGE);
//      xbee_data_to_packet(AGENT_4,MODE_CHANGE);
  }
  else if(New_Flight_mode_ON==STATE_TAKEOFF)
  { 
      if(mediumloop_counter==0)
      {
        do_takeoff_user();
      }
      
      mediumloop_counter++;
      if(mediumloop_counter<=20)
      {
        transmit_data.data[0] = STATE_TAKEOFF;
        transmit_data.data[1] = 0;
        xbee_data_to_packet(AGENT_0,MODE_CHANGE);
        xbee_data_to_packet(AGENT_2,MODE_CHANGE);
        xbee_data_to_packet(AGENT_3,MODE_CHANGE);
//        xbee_data_to_packet(AGENT_4,MODE_CHANGE);
      }
      else if(mediumloop_counter>20 && mediumloop_counter<=50)
      {
        transmit_data.data[0] = g_gps->latitude%1000000;   //get gps
        transmit_data.data[1] = g_gps->longitude%1000000;
        xbee_data_to_packet(AGENT_0,GPS_DATA);
        xbee_data_to_packet(AGENT_2,GPS_DATA);
        xbee_data_to_packet(AGENT_3,GPS_DATA);
//        xbee_data_to_packet(AGENT_4,GPS_DATA);
        
      }
      else if(mediumloop_counter>50)
      {
        mediumloop_counter = 0;
        New_Flight_mode_ON = STATE_CONTROLLED;
        
          // controller
//          set_roll_pitch_mode(ROLL_PITCH_AUTO);
//          set_yaw_mode(YAW_HOLD);
//          set_throttle_mode(THROTTLE_HOLD_USER);
//          set_nav_mode(NAV_NONE);
          
          // fixed
          set_roll_pitch_mode(ROLL_PITCH_LOITER);
          set_yaw_mode(YAW_HOLD);
          set_throttle_mode(THROTTLE_HOLD_USER);
          set_nav_mode(LOITER_NAV);
      }
  }
  else if(New_Flight_mode_ON==STATE_CONTROLLED)
  {
      mediumloop_counter++;
      
//      xbee_data_to_packet(AGENT_0, GPS_DATA);
      transmit_data.data[0] = g_gps->latitude%1000000;   //get gps
      transmit_data.data[1] = g_gps->longitude%1000000;
      xbee_data_to_packet(AGENT_0, GPS_DATA);
      xbee_data_to_packet(AGENT_2,GPS_DATA);
      xbee_data_to_packet(AGENT_3,GPS_DATA);


//      if(mediumloop_counter<300)
//      {
//          wp_nav.get_loiter_velocity_to_acceleration(-30.0247348063092,95.3861378807819,0.1);
//      }
//      else if(mediumloop_counter<400)
//      {
//          wp_nav.get_loiter_velocity_to_acceleration(95.3861378807819,30.0247348063092,0.1);
//      }
//      else if(mediumloop_counter<500)
//      {
//          wp_nav.get_loiter_velocity_to_acceleration(5.3861378807819,5.0247348063092,0.1);
//      }
//      else if(mediumloop_counter<800)
//      {
//          wp_nav.get_loiter_velocity_to_acceleration(30.0247348063092,-95.3861378807819,0.1);
//      }
  }
  else if(New_Flight_mode_ON==STATE_LANDING)
  {
      mediumloop_counter = 0;
      
      transmit_data.data[0] = STATE_LANDING;
      transmit_data.data[1] = 0;
      xbee_data_to_packet(AGENT_0,MODE_CHANGE);
      xbee_data_to_packet(AGENT_2,MODE_CHANGE);
      xbee_data_to_packet(AGENT_3,MODE_CHANGE);
//      xbee_data_to_packet(AGENT_4,MODE_CHANGE);
  }
  else if(New_Flight_mode_ON==STATE_DISARMED)
  {
      mediumloop_counter = 0;
      
      transmit_data.data[0] = STATE_DISARMED;
      transmit_data.data[1] = 0;
      xbee_data_to_packet(AGENT_0,MODE_CHANGE);
      xbee_data_to_packet(AGENT_2,MODE_CHANGE);
      xbee_data_to_packet(AGENT_3,MODE_CHANGE);
//      xbee_data_to_packet(AGENT_4,MODE_CHANGE);
  }
  else
  {
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
