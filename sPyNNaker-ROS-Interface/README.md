# ROS Spinnaker Interface
Connect SpiNNaker with ROS and use transfer functions to convert between live spikes and ROS values.

This project is part of my Bachelor Thesis SpiNNaker ROS Integration at the Chair of Neuroscientific System Theory of the Technical University of Munich.

Transform incoming ROS Messages into spikes and inject them into the Spinnaker Board and the other way round.


# Requirements and Installation
To run this interface you will need ROS and the Spinnaker Software, including the SpyNNakerExternalDevicesPlugin.
For Spinnaker, have a look at the PyNN on SpiNNaker Installation Guide on http://spinnakermanchester.github.io.

Also you will need a working ROS distribution, including rospy.

This package is not yet published on PyPI, but you can simply install it using the following command.

	pip install git+https://github.com/reiths/ros_spinnaker_interface

I highly recommend using a virtual python environment. Make sure rospy and all other python packages are linked correctly.


# Notes
This is work in progress, please tell me if any errors occur. 

Possible improvements could be manual timesteps in the base spike source / sink, like in spinnaker.
A more flexible way to change the ros message type would also be nice. The biggest problem here is the hardcoding of the message data field.


# Documentation
## Interface

    Args:

    n_neurons_source (int):  The number of neurons of the Spike Source.

    transfer_function_send (function handle): A handle to the transfer function used to convert
        the ROS input data into spikes.
    
    transfer_function_recv (function handle): A handle to the transfer function used to convert
        the live spikes to a ROS value.
    
    output_population (pynn.Population): The pyNN.Population you want to get the live spikes from.
        Defaults to None, so the live output is disabled.

    ros_topic_send (str): The ROS Topic used for sending into spinnaker.
        Defaults to "to_spinnaker".

    ros_topic_recv (str): The ROS Topic used for sending into ROS.
        Defaults to "from_spinnaker".

    clk_rate (int): The frequency the ROS Node is running with in Hz.
        Defaults to 1000 Hz.
    
    ros_output_rate (int): The frequency with which ros messages are sent out.
        Defaults to 10 Hz.

    benchmark (bool): Receive a timing output at the end of the simulation.
        Defaults to False.


    Attributes:

    InjectorPopulation: The ExternalDevices.SpikeInjector instance which is used internally.


    Functions:
        
        is_roscore_running(): True if the ros core is runnig else False.

        activate_live_output_for(pynn.Population): Set the pynn population you want to get the live spikes from.

        add_simulation_start_callback(function): Register the function as callback at simulation start.


    Examples:
        Have a look at the ros_spinnaker_interface_example.py or other example scripts.

    Notes:
        This interface uses the Spinnaker LiveSpikesConnection internally with the local ports
        19999 and 17895 and the spinnaker port 12345. These ports are widely used for live spikes and
        therefore should'nt cause any problems, however you can also simply change them in the constructor if needed.
        For each parallel interface used, these port numbers are increased by one, so the second interface will use
        the local ports 20000 and 17896 and 12346 on spinnaker, etc.

        If you want to change or extend this interface, consider that there is a sub process started by the 
        interface itself, as well as a thread controlled by spinnaker. Make sure they terminate and communicate properly.

        Currently only the std_msgs.msg.Int64 type is supported for ROS Messages. If you want to use your own
        ros message types it is possible, but you need to change some code yourself:
            - in the _incoming_ros_package_callback unpack the ros message fields and decide what to do with it.
            - in run_ros_node adjust the Publisher and Subscriber message types and (if needed) the publisher callback.

## Transfer Functions
The transfer functions (actually classes) define simple transformations, from incoming ROS values to the injected
live spikes or from observed spikes in ROS live output.

The base classes do all the work, so all you do here is inherit from them and define their behaviour
via the following functions.


Spike Source:

    The spike source must inherit from BasicSpikeSource.
    The behaviour of a spike source is defined by just one function:

    on_update:

        This function is called every time when a neuron needs a new inter spike interval.
        Basically every time a neuron has spiked, it asks when to spike next.

        Args:
            ros_value (usually int): The value of the last ros message. The type depends on the ros message type used.
            
            neuron (int): The ID of the neuron asking when to spike next.

            n_neurons (int): The total number of neurons of the spike source.

        Returns (int):
            The next inter spike interval in ms.


Spike Sink:

    The spike sink must inherit from BasicSpikeSink.
    The behaviour of the Spike Sink is defined by two different functions and one static variable:

    on_update_calling_rate (int): This static variable defines the rate in which on_update is called.
        For example a value of 10 means on_update is called every 10 ms.
        Defaults to 1 (on_update is called every 1 ms).

    on_update:
        This function is called periodically and can be used for periodic changes of the ROS output value.
        The calling rate can be set explicitly (see above).

        Args:
            neurons (list): list of neurons. A neuron has the following data structure:
                neuron.spike_times (list): list of times when the neuron had spiked:
                    spike_time (int): Number of ms since start of the spinnaker simulation:
                        Example: 5819 -> spike occured after 5819 ms
                neuron.key (int): The key (=ID) of the neuron. 0 <= key <= n_neurons

            sim_time (int): Current Time. Number of ms since start of the simulation.

            curr_ros_value (usually int): The current ROS value being published. Use it if you want to calculate
                the new ros value in dependency of the current one.

        Returns (int): 
            The new ROS value. Again the default type is int, because the default ROS message type used is Int64.

    on_spike:
        This function is called every time a spike occurs in the output population. Use it for event-based changes
        of the ROS output value.

        Args:
            spike_time (int): The time of the spike in number of ms since start of the simulation.

            neuron_id (int): The ID (=key) of the neuron that spiked.

            curr_ros_value (usually int): The current ROS value being published. Use it if you want to calculate
                the new ros value in dependency of the current one.


Plotting:
    In both classes you can optionally define a plot function without parameters.
    If you define it, it is called once at the end of the simulation. I stored all the ros values in a list and 
    used it to plot the ROS values over time in the end. I found it useful while testing the behaviour.
    Changing the name (for example from plot to _plot, you will see this in here) can be used to disable it again.
    Have a look at the examples.
