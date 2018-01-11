#!/usr/bin/env python
'''
@file 	ros_spinnaker_interface.py
@author Stephan Reith
@date 	13.07.2016

'''

import rospy
import socket
import sys
import time
import pydoc
from std_msgs.msg import Int64
from std_msgs.msg import Float32
from std_msgs.msg import String
from multiprocessing import Process, Queue, Lock
from itertools import count
from myo_blink import srv
from roboy_communication_middleware.msg import MotorCommand
import numpy as np


import spynnaker7.pyNN as p7
#import spynnaker_external_devices_plugin.pyNN as ExternalDevices

#from spynnaker_external_devices_plugin.pyNN.connections.\
#    spynnaker_live_spikes_connection import SpynnakerLiveSpikesConnection as LiveSpikesConnection


lock = Lock()


class _ROS_Spinnaker_Interface(object):

    """
    Transform incoming ROS Messages into spikes and inject them into the Spinnaker Board and the other way round.


    Args:

    n_neurons_source (int):  The number of neurons of the Spike Source.

    transfer_function_send (function handle): A handle to the transfer function used to convert
        the ROS input data into spikes.
    
    transfer_function_recv (function handle): A handle to the transfer function used to convert
        the live spikes to a ROS value.
    
    output_population (p7.Population): The pyNN.Population you want to get the live spikes from.
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

        activate_live_output_for(p7.Population): Set the p population you want to get the live spikes from.

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
    """


    _instance_counter = count(0)


    def __init__(self,
            n_neurons_source=None,
            Spike_Source_Class=None,
            Spike_Sink_Class=None,
            output_population=None,
            ros_topic_send='to_spinnaker',
            ros_topic_recv='from_spinnaker',
            clk_rate=1000,
            ros_output_rate=10,
            benchmark=False,
            muscle_id=None,
            population_coding_param=None,
            total_sim_time=5000.,
            max_min_motor_rates=None):

        # Members
        self.n_neurons = n_neurons_source if n_neurons_source is not None else 1
        self._Spike_Source_Class = Spike_Source_Class
        self._Spike_Sink_Class = Spike_Sink_Class
        self._output_population = output_population
        self.send_topic = ros_topic_send
        self.recv_topic = ros_topic_recv
        self._clk_rate = clk_rate  # in Hz
        self._ros_output_rate = ros_output_rate  # Hz
        self._benchmark = benchmark
        self.muscle_id = muscle_id
        self.muscle_value=0
        self.old_muscle_value=0
        self.population_coding_param=population_coding_param
        self.record_from_neurons=[]
        self.record_counter=0
        self.total_simulation_time=total_sim_time-500.
        if max_min_motor_rates is not None:
            try:
                self.min_motor_rate=max_min_motor_rates[0]
                self.max_motor_rate=max_min_motor_rates[1]
            except:
                self.min_motor_rate=10
                self.max_motor_rate=800
        else:
                self.min_motor_rate=10
                self.max_motor_rate=800
            
        self.av_ros_value=0.
        self.ros_value_counter=0.        
        
        self.interface_id = self._instance_counter.next()

        self._injector_label = 'injector{}'.format(self.interface_id)
        spike_injector_port = 12345 + self.interface_id
        local_port = 19999 #+ self.interface_id
        local_recv_port = 17895
        self._database_notify_port = local_port

        self._queue_ros_spinnaker = Queue()
        self._queue_spinnaker_ros = Queue()

        # My own "population" data structures to send and receive spikes, initialized later.
        self._spike_source = None
        self._spike_sink = None

        send_labels = [self._injector_label]
        rcv_labels = None

        self.sender_active = n_neurons_source is not None and self._Spike_Source_Class is not None
        self.receiver_active = self._output_population is not None and self._Spike_Sink_Class is not None

        if self.receiver_active:
            rcv_labels = [self._output_population.label]


        self._spike_injector_population = p7.Population(size=self.n_neurons,
                                                          cellclass=p7.external_devices.SpikeInjector,
                                                          cellparams={'port': spike_injector_port,
                                                                      'database_notify_port_num':19995},
                                                          label=self._injector_label)
        
        
        # SpiNNaker connection should be defined manually in the main script where the interface is called
       
    def _init_ros_node(self, label, sender):
        """
        Initialize the spike source and start the ros node.
        
        This is started as thread from the SpynnakerLiveSpikesConnection at the beginning of the simulation.
        """
        timestep = 1.0 / self._clk_rate * 1000

        if self.sender_active:
            self._spike_source = self._Spike_Source_Class(self.n_neurons,
                                                          label,
                                                          sender,
                                                          self._queue_ros_spinnaker,
                                                          timestep)
            
            try:
                self._spike_source.set_neuron_prop(self.population_coding_param,self._spike_source.neurons)
            except:
                print('Not population coding')

        if self.receiver_active:
            self._spike_sink = self._Spike_Sink_Class(len(self._output_population),  # get number of neurons
                                                      self._queue_spinnaker_ros,
                                                      timestep)

        if not self.is_roscore_running():
            sys.exit(0)

        p = Process(target=self.run_ros_node)
        p.daemon = True
        print("Interface {} started".format(self.interface_id))
        p.start()

    def run_ros_node(self):
        """
        Initialize a ROS Node and subscribe and publish to the given ROS Topics.

        ROS requires this function to run in its own child process.

        The tick generator makes sure that it runs once per timestep.
        """
        rospy.init_node('spinnaker_ros_interface{}'.format(self.interface_id), anonymous=True)

        if self.receiver_active:
            if self.muscle_id is not None:
                publisher = rospy.Publisher(self.recv_topic, MotorCommand, queue_size=10)
            else:
                publisher = rospy.Publisher(self.recv_topic, Int64, queue_size=10)
#            if self.muscle_name!=None:
#                myoarm_service=rospy.ServiceProxy('myo_blink/move',srv.moveMotor) 

	    
        if self.sender_active:
            rospy.Subscriber(self.send_topic, Float32, self._incoming_ros_package_callback)

        rospy.on_shutdown(self.on_ros_node_shutdown)

        def ros_publisher_callback(event):
            if not self.receiver_active:
                return
            try:
                ros_value=self._spike_sink._get_ros_value()
                dcn_min=35. # The min firing rate of the population this interface listens
                dcn_max=70. # The max firing rate of the population this interface listens
                ros_value=(self.max_motor_rate-self.min_motor_rate)*(ros_value-dcn_min)/dcn_max+self.min_motor_rate

                if ros_value <= 0:
#                    print("Requested displacement for {}: {}".format(self.muscle_id,ros_value))
                    ros_value=0
                elif ros_value >800:
#                    print("Requested displacement for {}: {}".format(self.muscle_id,ros_value))
                    ros_value=800
                    publisher.publish(id=0,motors=[self.muscle_id],setPoints=[ros_value])
                    print("Requested displacement for {}: {}".format(self.muscle_id,int(ros_value)))
                else:
                    print("Requested displacement for {}: {}".format(self.muscle_id,int(ros_value)))
                    publisher.publish(id=0,motors=[self.muscle_id],setPoints=[ros_value])
                    pass

                self.av_ros_value+=ros_value
                self.ros_value_counter+=1
            except rospy.ROSException:
                return



            

        rospy.Timer(rospy.Duration(1.0 / self._ros_output_rate), ros_publisher_callback)  # 10 Hz default
        
        ros_timer = rospy.Rate(self._clk_rate)
        
        self.interface_start_time = time.time()

        if self._benchmark:
            last = time.clock()
            self._num_timer_warnings = 0
            self._num_ticks = 0
            self._mainloop_execution_times = []
        
        while not rospy.is_shutdown():
            if self.sender_active:
                A=self._spike_source._update()     
                ''' The returned values from _update() are used to see if 
                encoding is correct
                '''
                if self.record_counter<self.total_simulation_time:                  
                    self.record_from_neurons.append(A) 
                    self.record_counter+=1
                elif self.record_counter==self.total_simulation_time:
                    np.save("record_from_neurons{}".format(self._injector_label),np.array(self.record_from_neurons))
                    self.record_counter+=1
                    try:
                        print("Average_value",self.interface_id,self.av_ros_value/(self.ros_value_counter+0.0))
                    except:
                        pass
                else:
                    pass
            if self.receiver_active:
                self._spike_sink._update()
            
            # Count if the mainloop execution takes too long
            if self._benchmark:
                self._num_ticks += 1
                now = time.clock()
                self._mainloop_execution_times.append(now - last)
                if (now - last) > (1.0 / self._clk_rate):
                    self._num_timer_warnings += 1
                last = now

            ros_timer.sleep()
        
    def _incoming_ros_package_callback(self, ros_msg):
        """
        Callback for the incoming data. Forwards the data via UDP to the Spinnaker Board.
        """
        self._queue_ros_spinnaker.put(ros_msg.data)  # data is the name of the ros std_msgs data field.
	

    def _incoming_spike_callback(self, label, time, neuron_ids):
        """
        Call this callback to process incoming live spikes.
        """
        for neuron_id in neuron_ids:
            spike = (label, time, neuron_id)
#            print(spike)
            self._queue_spinnaker_ros.put(spike)

    def is_roscore_running(self):
        """
        Returns True if the ROS Core is running and False otherwise.
        """
        return True  # TODO
        try:
            _, _, ros_master_pid = rospy.get_master().getPid()
            return True

        except socket.error:
            print('\n\n[!] Cannot communicate with ROS Master. Please check if ROS Core is running.\n\n')
            return False
    
    @property
    def InjectorPopulation(self):
        """
        The handle to the p7.external_devices.SpikeInjector which is used internally.
        Can be used for p7.Connectors
        """
        return self._spike_injector_population if self.sender_active else None

    def __str__(self):
        return 'ROS-Spinnaker-Interface'

    def __repr__(self):
        return self._spike_injector_population
    
    def on_ros_node_shutdown(self):
        # These do nothing on default. The plot functions need to be redefined in the SpikeSink/Source used to
        # actually do something.

        if self._benchmark:
            lock.acquire()
            print("Interface {} Benchmark".format(self.interface_id))
            # print("startet running on time {}".format(self.interface_start_time))
            # print("stopped runnning on time {}".format(time.time()))
            print("Number of times the mainloop took too long: {}".format(self._num_timer_warnings))
            print("Number of Mainloop Calls: {}".format(self._num_ticks))
            import numpy as np
            mean_execution_time = np.mean(self._mainloop_execution_times)
            print("Mean Mainloop Execution Time: {} ms, (max {} ms)".format(mean_execution_time, 1.0 / self._clk_rate))
            print("Highest possible interface clock rate: {} Hz\n".format(1.0 / mean_execution_time))
            lock.release()

        if self.sender_active:
            self._spike_source.plot()
        if self.receiver_active:
            self._spike_sink.plot()
    
    def add_simulation_start_callback(self, function):
        if self.sender_active:
            self._spinnaker_connection.add_start_callback(self._injector_label, function)



def ROS_Spinnaker_Interface(*args, **kwargs):
    """
    ROS_Spinnaker_Interface is the factory function for the actual _ROS_Spinnaker_Interface.

    Returns the p7.SpikeInjector population instead of the interface instance,
    so the interface can be used directly for p7.Projections.

    Help for the actual _ROS_Spinnaker_Interface:

    """
    try:
        interface = _ROS_Spinnaker_Interface(*args, **kwargs)
        return interface.InjectorPopulation,interface

    except TypeError:
        print("\nOops the Initialisation went wrong.")
        print("Please use help(_ROS_Spinnaker_Interface) and double check the arguments.")
        raise

ROS_Spinnaker_Interface.__doc__ += pydoc.text.document(_ROS_Spinnaker_Interface)  # append help(_ROS_Spinnaker_Interface)



if __name__ == '__main__':
    pass
