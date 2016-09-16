// Copyright (c) 2015-2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
// 
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/** @file griffin_powermate.h
 *  On /griffin_powermate/events topic it publishes griffin_powermate::PowermateEvent
 *  messages that contain direction and integral of the turn wheel as well as the 
 *  information about push button being pressed or depressed.
 * 
 *  NOTE
 *  If you get permission denied when running this ROS node, use
 * 	ls -l /dev/input/event*
 *  to learn which group can access linux input events. Then add your username to
 *  that group by issuing
 *  	sudo usermod -a -G [group_name] [user_name]
 *  You need to log out and back in for these changes to take effect.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "griffin_powermate/griffin_powermate.h"

/** Opens the input device and checks whether its meaningful name (ie, EVIOCGNAME in ioctl()) is listed in valid_substrings_.
 *  @param device_path file name for linux event.
 *  @return file descriptor to PowerMate event if all checks out, otherwise -1.
 */
int PowerMate::openPowerMate(const char *device_path)
{
  printf("Opening device: %s \n", device_path);
  
  // Open device at device_path for READONLY and get file descriptor 
  int fd = open(device_path, O_RDONLY);
  
  // If failed to open device at device_path
  if(fd < 0)
  {
    ROS_ERROR("Failed to open \"%s\"\n", device_path);
    return -1;
  }

  // Meaningful, i.e., EVIOCGNAME name
  char name[255];
  // Fetch the meaningful (i.e., EVIOCGNAME) name
  if(ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0)
  {
    ROS_ERROR("\"%s\": EVIOCGNAME failed.", device_path);
    close(fd);
    // Returns -1 if failed to fetch the meaningful name
    return -1;
  }

  // Let's check if the meaningful name matches one listed in valid_substrings_
  std::ostringstream sstream;
  // Convert name given as char* to stringstream
  sstream << name;
  // stringstream to string
  std::string name_as_string = sstream.str();
  int i;
  for (i=0; i < valid_substrings_.size(); i++)
  {
    // Does the meaningful name contain a predefined valid substring?
    std::size_t found = name_as_string.find( valid_substrings_[i] );
    if (found!=std::string::npos)
    {
      // if everything checks out, print on screen and return the file descriptor
      ROS_INFO("Found \x1b[1;34m'%s'\x1b[0m device. Starting to read ...\n", name);
      return fd;
    } // end if
  } // end for
  
  close(fd);
  return -1;
} // end openPowerMate

/** Goes through all the event files in /dev/input/ to locate Griffin PowerMate USB.
 *  @return file descriptor if all checks out, otherwise -1.
 */
int PowerMate::findPowerMate()
{
  // Using glob() [see: http://linux.die.net/man/3/glob ] for getting event files in /dev/input/
  glob_t gl;
  // Number of event files found in /dev/input/
  int num_event_dev = 0;
  // Counts for filenames that match the given pattern
  if(glob("/dev/input/event*", GLOB_NOSORT, NULL, &gl) == 0)
  {
    // Get number of event files
    num_event_dev = gl.gl_pathc;
  }

  int i, r;
  // Goes through all the found event files
  for(i = 0; i < num_event_dev; i++)
  {
    // Tries to open an event file as a PowerMate device
    r = openPowerMate(gl.gl_pathv[i]);
    // If opened file is PowerMate event, return file descriptor
    if(r >= 0) return r;
  } // for
  
  // free memory allocated for globe struct
  globfree(&gl);

  // return error -1 because no PowerMate device was found
  return -1;
} // end findPowerMate

/** Closes the device specificed by descriptor_. */
void PowerMate::closePowerMate()
{
  printf("Closing PowerMate device.\n");
  close(descriptor_);
  return;
}

/** Checks if the PowerMate event file has been succesfully opened.
 *  @return TRUE when descriptor_ is not negative, FALSE otherwise.
 */
bool PowerMate::isReadable ()
{
  if (descriptor_ < 0) return false;
  return true;
}

/** Processes the event data and publishes it as PowermateEvent message.
 *  @param ev input event.
 *  @param ros_publisher ROS publisher.
 */
void PowerMate::processEvent(struct input_event *ev, ros::Publisher& ros_publisher)
{
  // PowermateEvent ROS message
  griffin_powermate::PowermateEvent ros_message;

  // ---- Event information about Griffin PowerMate USB from evtest ----
  //
  // Input device name: "Griffin PowerMate"
  // Supported events:
  //	Event type 0 (EV_SYN)
  //	Event type 1 (EV_KEY)
  //		Event code 256 (BTN_0)
  //	Event type 2 (EV_REL)
  //		Event code 7 (REL_DIAL)
  //	Event type 4 (EV_MSC)
  //		Event code 1 (MSC_PULSELED)
  // -------------------------------------------------------------------
  
  // Switch to a case based on the event type
  switch(ev->type)
  {
    case EV_SYN:				// no need to do anything
//      printf("SYN REPORT\n");
      break; 
    case EV_MSC:				// unused for this ROS publisher
      ROS_INFO("The LED pulse settings were changed; code=0x%04x, value=0x%08x\n", ev->code, ev->value);
      break;
    case EV_REL:				// Upon receiving rotation data
      if(ev->code != REL_DIAL)
	ROS_WARN("Unexpected rotation event; ev->code = 0x%04x\n", ev->code);
      else
      {
	// Reads direction value from turn knob
	signed char dir = (signed char)ev->value;
	// Sums consecutive dir values to find integral
	integral_ += (long long)dir;
	// Composing a ros_message
	ros_message.direction = dir;
	ros_message.integral = integral_;
	ros_message.is_pressed = pressed_;
	ros_message.push_state_changed = false;
	// Publish ros_message
	ros_publisher.publish( ros_message );
	//printf("Button was rotated %d units; Shift from start is now %d units\n", (int)ev->value, total_shift);
      }
      break;
    case EV_KEY:				// Upon receiving data about pressing and depressing the dial button
      if(ev->code != BTN_0)
	ROS_WARN("Unexpected key event; ev->code = 0x%04x\n", ev->code);
      else
      {
	// reads EV_KEY value, converts it to bool
	pressed_ = (bool)ev->value;
	// Composing a ros_message
	ros_message.direction = 0;
	ros_message.integral = integral_;
	ros_message.is_pressed = pressed_;
	ros_message.push_state_changed = true;
	// Publish ros_message
	ros_publisher.publish( ros_message );
	//printf("Button was %s\n", ev->value? "pressed":"released");
      }
      break;
    default:					// default case
      ROS_WARN("Unexpected event type; ev->type = 0x%04x\n", ev->type);
  } // end switch

  fflush(stdout);
} // end processEvent

/** Method for reading the event data and ROS spinning.
 *  @param ros_publisher ROS publisher used to publish PowermateEvent message.
 */
void PowerMate::spinPowerMate(ros::Publisher& ros_publisher)
{
  int const BUFFER_SIZE = 32;
  
  // see: https://www.kernel.org/doc/Documentation/input/input.txt
  struct input_event ibuffer[BUFFER_SIZE];
  int r, events, i;

  while( ros::ok() )
  {  
    // read() reads a binary file [http://pubs.opengroup.org/onlinepubs/9699919799/functions/read.html] and returns the number of bytes read.
    // The program waits in read() until there's something to read; thus it always gets a new event but ROS cannot make a clean exit while in read().
    // TODO: Figure out a way for ROS to exit cleanly.
    r = read(descriptor_, ibuffer, sizeof(struct input_event) * BUFFER_SIZE);
    if( r > 0 )
    {
      // Calculate the number of events
      events = r / sizeof(struct input_event);
      // Go through each read events
      for(i = 0; i < events; i++)
      {
	// Process event and publish data
	processEvent(&ibuffer[i], ros_publisher);
	// spin
	ros::spinOnce();
      } // end for
    } // end if
    else
    {
      // Let user know if read() has failed
      ROS_WARN("read() failed.\n");
      return;
    } // end else

  } // end while
  
  return;
} // end spinPowerMate

/** Main method. */
int main(int argc, char *argv[])
{
  // ROS init
  ros::init(argc, argv, "griffin_powermate");
  
  // Private nodehandle for ROS
  ros::NodeHandle pnh("~");
  
  // Getting user-specified path from ROS parameter server
  std::string powermate_path;
  pnh.param<std::string>("path", powermate_path, "");
  
  // Let's construct PowerMate object 
  PowerMate powermate(powermate_path);
  
  // If failed to open any PowerMate USB device, print info and exit
  if( !powermate.isReadable() )
  {
    ROS_ERROR("Unable to locate any PowerMate device.");
    ROS_INFO("You may try specifying path as ROS parameter, e.g., rosrun griffin_powermate griffin_powermate _path:=<device_event_path>");
    return -1;
  }

  // Creates publisher that advertises griffin_powermate::PowermateEvent messages on topic /griffin_powermate/events
  ros::Publisher pub_powermate_events = pnh.advertise<griffin_powermate::PowermateEvent>("events", 100);
  
  // After PowerMate is succesfully opened, read its input, publish ROS messages, and spin.
  powermate.spinPowerMate(pub_powermate_events);

  // Close PowerMate
  powermate.closePowerMate();

  return 0;
} //end main
