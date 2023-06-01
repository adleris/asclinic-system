/*

*/

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include <gpiod.h>
#include <mutex>
#include <thread>
#include "asclinic_pkg/LeftRightInt32.h"
#include "asclinic_pkg/LeftRightFloat32.h"

// Member Variables for the node
ros::Publisher m_encoderCounts_Publisher;
ros::Timer m_timer_publishing;

int m_encoder_counts_for_motor_left_a = 0, 
    m_encoder_counts_for_motor_left_b = 0,
    m_encoder_counts_for_motor_right_a = 0,
    m_encoder_counts_for_motor_right_b = 0;

std::mutex m_counting_mutex;

int m_line_number_for_motor_left_channel_a = 84, 
    m_line_number_for_motor_left_channel_b = 130,
    m_line_number_for_motor_right_channel_a = 105,
    m_line_number_for_motor_right_channel_b = 106;

bool encoder_thread_should_count = true;

float m_delta_t_for_publishing_counts = 0.05;
int m_gpiochip_number = 1;

void timerCallbackForPublishing(const ros::TimerEvent&)
{
    int counts_motor_left_a_local_copy, 
        counts_motor_left_b_local_copy,
        counts_motor_right_a_local_copy,
        counts_motor_right_b_local_copy;
    
    //locking for a copy and reset
    m_counting_mutex.lock();
    //Copying Count
    counts_motor_left_a_local_copy = m_encoder_counts_for_motor_left_a;
    counts_motor_left_b_local_copy = m_encoder_counts_for_motor_left_b;
    counts_motor_right_a_local_copy = m_encoder_counts_for_motor_right_a;
    counts_motor_right_b_local_copy = m_encoder_counts_for_motor_right_b;

    //reseting the count
    m_encoder_counts_for_motor_left_a = 0; 
    m_encoder_counts_for_motor_left_b = 0;
    m_encoder_counts_for_motor_right_a = 0;
    m_encoder_counts_for_motor_right_b = 0;
    m_counting_mutex.unlock();

    //publishing messages 
    //? Could add the conversion here or do the average
    asclinic_pkg::LeftRightInt32 msg;
    msg.left = counts_motor_left_a_local_copy  + counts_motor_left_b_local_copy;
	msg.right = counts_motor_right_a_local_copy + counts_motor_right_b_local_copy;
	m_encoderCounts_Publisher.publish(msg);
}

void encoderCountingThreadMain()
{
    std::stringstream temp_string_stream;
    temp_string_stream << "/dev/gpiochip" << m_gpiochip_number;
    const char * gpio_chip_name = temp_string_stream.str().c_str();

    int line_number_left_a  = m_line_number_for_motor_left_channel_a,
	    line_number_left_b  = m_line_number_for_motor_left_channel_b,
	    line_number_right_a = m_line_number_for_motor_right_channel_a,
	    line_number_right_b = m_line_number_for_motor_right_channel_b;
    
    // Initialise a GPIO chip, line, and event objects
	struct gpiod_chip *chip;
	struct gpiod_line *line_left_a;
	struct gpiod_line *line_left_b;
	struct gpiod_line *line_right_a;
	struct gpiod_line *line_right_b;
	struct gpiod_line_bulk line_bulk;
	struct gpiod_line_event event;
	struct gpiod_line_bulk event_bulk;

    struct timespec timeout_spec = { 0, 10000000 };

    int returned_wait_flag;
	int returned_read_flag;

    int value;
    
    // > For left motor channel A
    value = gpiod_ctxless_get_value(gpio_chip_name, line_number_left_a, false, "foobar");
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_left_a << " returned value = " << value);
	// > For left motor channel B
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_left_b, false, "foobar");
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_left_b << " returned value = " << value);
	// > For right motor channel A
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_right_a, false, "foobar");
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_right_a << " returned value = " << value);
	// > For right motor channel B
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_right_b, false, "foobar");
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_right_b << " returned value = " << value);

    // Open the GPIO chip
	chip = gpiod_chip_open(gpio_chip_name);
	// Retrieve the GPIO lines
	line_left_a  = gpiod_chip_get_line(chip,line_number_left_a);
	line_left_b  = gpiod_chip_get_line(chip,line_number_left_b);
	line_right_a = gpiod_chip_get_line(chip,line_number_right_a);
	line_right_b = gpiod_chip_get_line(chip,line_number_right_b);
	// Initialise the line bulk
	gpiod_line_bulk_init(&line_bulk);
	// Add the lines to the line bulk
	gpiod_line_bulk_add(&line_bulk, line_left_a);
	gpiod_line_bulk_add(&line_bulk, line_left_b);
	gpiod_line_bulk_add(&line_bulk, line_right_a);
	gpiod_line_bulk_add(&line_bulk, line_right_b);

	// Display the status
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Chip " << gpio_chip_name << " opened and lines " << line_number_left_a << ", " << line_number_left_b << ", " << line_number_right_a << " and " << line_number_right_b << " retrieved");

    gpiod_line_request_bulk_falling_edge_events(&line_bulk, "foobar");

    while (encoder_thread_should_count)
	{

		// Monitor for the requested events on the GPIO line bulk
		// > Note: the function "gpiod_line_event_wait" returns:
		//    0  if wait timed out
		//   -1  if an error occurred
		//    1  if an event occurred.
		returned_wait_flag = gpiod_line_event_wait_bulk(&line_bulk, &timeout_spec, &event_bulk);

		// Respond based on the the return flag
		if (returned_wait_flag == 1)
		{
			// Get the number of events that occurred
			int num_events_during_wait = gpiod_line_bulk_num_lines(&event_bulk);

			// Lock the mutex while before counting the events
			m_counting_mutex.lock();

			// Iterate over the event
			for (int i_event = 0; i_event < num_events_during_wait; i_event++)
			{
				// Get the line handle for this event
				struct gpiod_line *line_handle = gpiod_line_bulk_get_line(&event_bulk, i_event);

				// Get the number of this line
				unsigned int this_line_number = gpiod_line_offset(line_handle);

				// Read the event on the GPIO line
				// > Note: the function "gpiod_line_event_read" returns:
				//    0  if the event was read correctly
				//   -1  if an error occurred
				returned_read_flag = gpiod_line_event_read(line_handle,&event);

				// Respond based on the the return flag
				if (returned_read_flag == 0)
				{
					// Increment the respective count
					if (this_line_number == line_number_left_a)
						m_encoder_counts_for_motor_left_a++;
					else if (this_line_number == line_number_left_b)
						m_encoder_counts_for_motor_left_b++;
					else if (this_line_number == line_number_right_a)
						m_encoder_counts_for_motor_right_a++;
					else if (this_line_number == line_number_right_b)
						m_encoder_counts_for_motor_right_b++;

				} // END OF: "if (returned_read_flag == 0)"

			} // END OF: "for (int i_event = 0; i_event < num_events_during_wait; i_event++)"

			// Unlock the mutex
			m_counting_mutex.unlock();

		} // END OF: "if (returned_wait_flag == 1)"
	} // END OF: "while (true)"

	// Release the lines
	gpiod_line_release_bulk(&line_bulk);
	// Close the GPIO chip
	gpiod_chip_close(chip);
	// Inform the user
	ROS_INFO("[ENCODER READ MULTI THREADED] Lines released and GPIO chip closed");  
}

int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "Encoder_read_multi_Thread");
	ros::NodeHandle nodeHandle("~");

	// Get the GPIO line number to monitor
	// Notes:
	// > If you look at the "encoder.launch" file located in
	//   the "launch" folder, you see the following lines of code:
	//       <param
	//           name   = "line_number_for_motor_left_channel_a"
	//           value  = 133
	//       />
	// > These lines of code add a parameter named to this node
	//   with the parameter name: "line_number_for_motor_left_channel_a"
	// > Thus, to access this parameter, we first get a handle to
	//   this node within the namespace that it was launched.
	//

	// Get the "gpiochip" number parameter:
	if ( !nodeHandle.getParam("gpiochip_number", m_gpiochip_number) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"gpiochip_number\" parameter. Using default value instead.");
	}

	// Get the line number parameters:
	// > For channel A of the left side motor
	if ( !nodeHandle.getParam("line_number_for_motor_left_channel_a", m_line_number_for_motor_left_channel_a) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_left_channel_a\" parameter. Using default value instead.");
	}
	// > For channel B of the left side motor
	if ( !nodeHandle.getParam("line_number_for_motor_left_channel_b", m_line_number_for_motor_left_channel_b) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_left_channel_b\" parameter. Using default value instead.");
	}
	// > For channel A of the right side motor
	if ( !nodeHandle.getParam("line_number_for_motor_right_channel_a", m_line_number_for_motor_right_channel_a) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_right_channel_a\" parameter. Using default value instead.");
	}
	// > For channel A of the right side motor
	if ( !nodeHandle.getParam("line_number_for_motor_right_channel_b", m_line_number_for_motor_right_channel_b) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_right_channel_b\" parameter. Using default value instead.");
	}
	// > Display the line numbers being monitored
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Will monitor line_numbers = " << m_line_number_for_motor_left_channel_a << ", " << m_line_number_for_motor_left_channel_b << ", " << m_line_number_for_motor_right_channel_a << ", and " << m_line_number_for_motor_right_channel_b);

	// Get the "detla t" parameter for the publishing frequency
	if ( !nodeHandle.getParam("delta_t_for_publishing_counts", m_delta_t_for_publishing_counts) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"delta_t_for_publishing_counts\" parameter. Using default value instead.");
	}

    std::string ns_for_group = ros::this_node::getNamespace();
	ros::NodeHandle nh_for_group(ns_for_group);

    m_encoderCounts_Publisher = nh_for_group.advertise<asclinic_pkg::LeftRightInt32>("encoder_counts", 10, false);
    m_timer_publishing = nodeHandle.createTimer(ros::Duration(m_delta_t_for_publishing_counts), timerCallbackForPublishing, false);

    std::thread encoder_counting_thread (encoderCountingThreadMain);

    ros::spin();

    encoder_thread_should_count = false;

	// Join back the encoder counting thread
	encoder_counting_thread.join();

	return 0;
}