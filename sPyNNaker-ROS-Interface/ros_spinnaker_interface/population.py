#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
@file 	spike_source.py
@author Stephan Reith
@date 	02.08.2016

Spike Source for the ROS-Spinnaker-Interface
'''


import time


class Neuron(object):
    """
    Simple Bunch Class. Basically a mutable version of the namedtuple.
    Used to make working with the neurons more readable.

    Example:
    >>> n = Neuron(spike_interval=100, counter=100, update_interval_immediately=True)
    >>> n.spike_interval = 1000
    """
    def __init__(self, **kwds):
        self.__dict__.update(kwds)

    def __repr__(self):
        return str(self.__dict__)



class BasicSpikeSource(object):


    def __init__(self, n_neurons, label, sender, queue, timestep):
        self._n_neurons = n_neurons
        self._label = label
        self._sender = sender
        self._q = queue
        self._timestep = timestep  # int?

        self.neurons = [Neuron (key=i, counter=None)
                        for i in range(n_neurons)]

        self._current_ros_value = None  # unused
        
    def _update(self):
        """
        Do the actual spike source logic.
        """
        if not self._q.empty():
            # incoming ros message
            ros_msg = self._q.get()
            self._current_ros_value = ros_msg

        for neuron in self.neurons:
            neuron.enabled = self._current_ros_value is not None

            if neuron.counter is not None:
                # neuron is not disabled
                neuron.counter -= self._timestep

                if neuron.counter <= 0:
                    # counter expired
                    self._send_spike(neuron.key)
                    neuron.counter = self.on_update(self._current_ros_value, neuron.key, self._n_neurons)
                    
            else:
                neuron.counter = self.on_update(self._current_ros_value, neuron.key, self._n_neurons)
        return [self._current_ros_value,[neuron.counter for neuron in self.neurons]]
    def _send_spike(self, key):
            #self._sender.send_spike(self._label, key, send_full_keys=True)
            self._sender.send_spike(self._label, key)

    def on_update(self, ros_msg, neuron, n_neurons):
        raise NotImplementedError("Please implement on_update(...) in a subclass to build your own spike source " + \
                                  "or choose a spike source from the transfer_functions module.")

    def plot(self):
        pass



class BasicSpikeSink(object):
    
    """
    Please inherit from this class and implement the on_spike and on_update methods.
    """

    def __init__(self, n_neurons, queue, timestep):
        self._n_neurons = n_neurons
        self._q = queue

        self._ros_value = 0

        self._neurons = [Neuron (key=i, spike_times=[])
                                         for i in range(n_neurons)]
        
        if hasattr(self, "on_update_calling_rate"):
            self.on_update_calling_rate = max(self.on_update_calling_rate / timestep, 1)
            # man this is some fancy python... tests if on_update_calling_rate exists as static in the child class.
        else:
            self.on_update_calling_rate = 1

        # get all class attributes defined in the subclasses
        static_class_attributes = [attr for attr in self.__class__.__dict__
                                   if not callable(getattr(self, attr)) and not attr.startswith('__')]
        
        # and make them object members
        # this feature is currently unused but useful when controlling this base class based on static child attributes
        # just like on_update_calling_rate above
        for attr in static_class_attributes:
            setattr(self, attr, getattr(self, attr))

        self._call_counter = 0

        # This constructor is called by the LiveSpikesConnection.add_start_callback at the start of the simulation
        # working with this timestamp to measure elapsed simulation time should be as accurate as possible.
        self._sim_start = time.time()
            
    def _update(self):
        """
        Process an incoming spike.
        
        Called with normal rate.
        """
        if not self._q.empty():
            spike = self._q.get()
            label, spike_time, neuron_id = spike
            self._neurons[neuron_id].spike_times.append(spike_time)
            
            new_ros_value = self.on_spike(spike_time, neuron_id, self._ros_value)
            self._ros_value = new_ros_value if new_ros_value is not None else self._ros_value
    
        self._call_counter += 1

        if self._call_counter >= self.on_update_calling_rate:
            self._call_counter = 0

            # Call on_update with a specific rate
            sim_time = int((time.time() - self._sim_start) * 1000)  # ms
            new_ros_value = self.on_update(self._neurons, sim_time, self._ros_value)
            self._ros_value = new_ros_value if new_ros_value is not None else self._ros_value


    def _get_ros_value(self):
        return int(self._ros_value)

    def on_spike(self, spike_time, neuron_id, curr_ros_value):
        raise NotImplementedError("Please implement on_spike(...) in a subclass to build your own spike sink " + \
                                  "or choose a spike sink from the transfer_functions module.")

    def on_update(self, neurons, sim_time, curr_ros_value):
        raise NotImplementedError("Please implement on_update(...) in a subclass to build your own spike sink " + \
                                  "or choose a spike sink from the transfer_functions module.")

    def plot(self):
        pass


if __name__ == "__main__":
    pass
