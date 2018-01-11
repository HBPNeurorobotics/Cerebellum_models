#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@author Stephan Reith
@date 	31.08.2016

This is a simple example to demonstrate how the ROS Spinnaker Interface can be used.

You will also need a ROS Listener and a ROS Talker to send and receive data.
Make sure they communicate over the same ROS topics and std_msgs.Int64 ROS Messages used in here.
"""

import spynnaker.pyNN as pynn

from ros_spinnaker_interface import ROS_Spinnaker_Interface
# import transfer_functions as tf
from ros_spinnaker_interface import SpikeSourcePoisson
from ros_spinnaker_interface import SpikeSinkSmoothing


ts = 0.1
n_neurons = 1
simulation_time = 10000  # ms


pynn.setup(timestep=ts, min_delay=ts, max_delay=2.0*ts)


pop = pynn.Population(size=n_neurons, cellclass=pynn.IF_curr_exp, cellparams={}, label='pop')


# The ROS_Spinnaker_Interface just needs to be initialised. The following parameters are possible:
ros_interface = ROS_Spinnaker_Interface(
        n_neurons_source=n_neurons,                 # number of neurons of the injector population
        Spike_Source_Class=SpikeSourcePoisson,   # the transfer function ROS Input -> Spikes you want to use.
        Spike_Sink_Class=SpikeSinkSmoothing,     # the transfer function Spikes -> ROS Output you want to use.
                                                    # You can choose from the transfer_functions module
                                                    # or write one yourself.
        output_population=pop,                      # the pynn population you wish to receive the
                                                    # live spikes from.
        ros_topic_send='to_spinnaker',              # the ROS topic used for the incoming ROS values.
        ros_topic_recv='from_spinnaker',            # the ROS topic used for the outgoing ROS values.
        clk_rate=1000,                              # mainloop clock (update) rate in Hz.
        ros_output_rate=10)                         # number of ROS messages send out per second.

# Build your network, run the simulation and optionally record the spikes and voltages.
pynn.Projection(ros_interface, pop, pynn.OneToOneConnector(weights=5, delays=1))


pop.record()
pop.record_v()

pynn.run(simulation_time)

spikes = pop.getSpikes()

pynn.end()

# Plot
import pylab

spike_times = [spike[1] for spike in spikes]
spike_ids = [spike[0] for spike in spikes]

pylab.plot(spike_times, spike_ids, ".")
pylab.xlabel('Time (ms)')
pylab.ylabel('Neuron ID')
pylab.title('Spike Plot')
pylab.xlim(xmin=0)
pylab.show()
