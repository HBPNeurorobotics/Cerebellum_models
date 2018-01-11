# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 15:46:02 2017

@author: emec
"""

import spynnaker7.pyNN as sim
import numpy as np
from python_models7.neuron.builds\
    .if_curr_exp_supervision import IFCurrExpSupervision
from python_models7.neuron.plasticity.stdp.timing_dependence\
    .timing_dependence_cerebellum import TimingDependenceCerebellum
import time
import IPython


from ros_spinnaker_interface import ROS_Spinnaker_Interface
# import transfer_functions as tf
from ros_spinnaker_interface import SpikeSourcePoisson
from ros_spinnaker_interface import SpikeSinkSmoothing
from ros_spinnaker_interface import SpikeSourceConstantRate
from ros_spinnaker_interface import SpikeSourcePopulationRate




#################### SIMULATION PARAMETERS ####################################
timestep=1.0
simulation_time=200000.
sim.setup(timestep=timestep,n_chips_required=49)

#################### MODEL DEFINITIONS ########################################
# Number of neurons in each layer per muscle
# MoF: Mossy Fibers
# GrC: Granular Cells
# InO: Inferior Olives
# PuC: Purkinje Cells
# DCN: Deep cerebellar nuclei

n_MoF=16
n_GrC=n_MoF**2
n_InO=8
n_PuC=8
n_DCN=4

# Neuron models in each layer
m_GrC=sim.IF_curr_exp
m_PuC=IFCurrExpSupervision
m_DCN=sim.IF_curr_exp


# Neuron parameters in each layer
p_MoF={}
p_GrC={
 'tau_refrac' : 1.29099444874,
 'tau_m' : 12.9099444874,
 'cm' : 0.00258198889747,
 'v_thresh' : -40.0,
 'tau_syn_E' : 0.645497224368,
 'v_rest' : -70.0,
 'tau_syn_I' : 12.9099444874,
 'v_reset' : -70.0,
 }
p_InO={}
p_PuC={'cm': 0.25,  # nF
   'i_offset': 0.0,
   'tau_m': 10.0,
   'tau_refrac': 2.0,
   'tau_syn_E': 2.5,
   'tau_syn_I': 2.5,
   'v_reset': -70.0,
   'v_rest': -65.0,
   'v_thresh': -55.4
   }
p_DCN= {
 'tau_refrac' : 1.29099444874,
 'tau_m' : 12.9099444874,
 'cm' : 0.00258198889747,
 'v_thresh' : -40.0,
 'tau_syn_E' : 0.645497224368,
 'v_rest' : -70.0,
 'tau_syn_I' : 12.9099444874,
 'v_reset' : -70.0,
 }

# Min and max firing rates
r_MoF_min=5.
r_MoF_max=30.
r_InO_min=5.
r_InO_max=40.

# Min and max sensor values
# MoF takes angle values 
# InO takes error values
s_MoF_min=1700.
s_MoF_max=3400.

s_InO_left_min=40.0
s_InO_left_max=1800.0
s_InO_right_min=-1800.0
s_InO_right_max=-40.0

# Gauss width (intervals between centers of neurons)
g_MoF=(s_MoF_max-s_MoF_min)/(3.*n_MoF) #20.
g_InO=(s_InO_right_max-s_InO_right_min)/(3.*n_InO)

# Population coding dictionary
MoF_current_dict={'min_rate':r_MoF_min,'max_rate':r_MoF_max,'sensor_min':s_MoF_min,'sensor_max':s_MoF_max,'gauss_width':g_MoF}
MoF_set_dict={'min_rate':r_MoF_min,'max_rate':r_MoF_max,'sensor_min':s_MoF_min,'sensor_max':s_MoF_max,'gauss_width':g_MoF}
InO_left_dict={'min_rate':r_InO_min,'max_rate':r_InO_max,'sensor_min':s_InO_left_min,'sensor_max':s_InO_left_max,'gauss_width':g_InO}
InO_right_dict={'min_rate':r_InO_min,'max_rate':r_InO_max,'sensor_min':s_InO_right_min,'sensor_max':s_InO_right_max,'gauss_width':g_InO}

#################### CONNECTION PARAMETERS #################################### 
mult=80
coeff=[60.0    ,30.0     ,40.0     ,1.0]
### max DCN with 40 is 150
W_MAX=5.
raw_weight_MoF_GrC = coeff[0]*0.0044#0.0044 
raw_weight_MoF_DCN=coeff[1]* 0.006#0.006
raw_weight_PuC_DCN= coeff[2]*0.002 #0.002
rng = sim.NumpyRNG(seed=hash(time.time()))
weight_dist_GrC_PuC= sim.RandomDistribution('uniform',[0.50,0.70],rng)

#################### POPULATION DEFINITIONS ################################### 


### GrC layer (one for each joint)
GrC_layer_pop=[]
GrC_layer_pop.append(sim.Population(n_GrC,m_GrC,p_GrC,label="GrC"))
GrC_layer_pop[0].record()


### PuC layer (left and right for each joint)
PuC_layer_pop=[]
PuC_layer_pop.append(sim.Population(n_PuC,m_PuC,p_PuC,label="PuC_left"))
PuC_layer_pop.append(sim.Population(n_PuC,m_PuC,p_PuC,label="PuC_right"))
for pop in PuC_layer_pop:
    pop.record()

### DCN layer (left and right for each joint)
DCN_layer_pop=[]
DCN_labels=[]
DCN_layer_pop.append(sim.Population(n_DCN,m_DCN,p_DCN,label="DCN_left"))
DCN_layer_pop.append(sim.Population(n_DCN,m_DCN,p_DCN,label="DCN_right"))
for pop in DCN_layer_pop:
    DCN_labels.append(pop.label)
    pop.record()

    
#################### ROS INTERFACE POPULATIONS ################################ 
#0:biceps , 1:triceps, 2: wrist flexor, 3: wrist extensor
MoF_current, MoF_current_interface = ROS_Spinnaker_Interface(
        n_neurons_source=n_MoF,                
        Spike_Source_Class=SpikeSourcePopulationRate,   
        Spike_Sink_Class=SpikeSinkSmoothing,                                          
        output_population=DCN_layer_pop[0],                                     
        ros_topic_send="/desired_angles/current",# '/myo_blink/joints/upper/angle', #"/desired_angles/current"
        ros_topic_recv='/roboy/middleware/MotorCommand',         
        clk_rate=1000,                             
        ros_output_rate=50,
        muscle_id=0,
        population_coding_param=MoF_current_dict,
        total_sim_time=simulation_time,
        max_min_motor_rates=[80,800])                  

MoF_set,MoF_set_interface = ROS_Spinnaker_Interface(
        n_neurons_source=n_MoF,                
        Spike_Source_Class=SpikeSourcePopulationRate,   
        Spike_Sink_Class=SpikeSinkSmoothing,                                          
        output_population=DCN_layer_pop[1],                                     
        ros_topic_send='/desired_angles/angle',   
        ros_topic_recv='/roboy/middleware/MotorCommand',
        clk_rate=1000,                             
        ros_output_rate=50,
        muscle_id=1,
        population_coding_param=MoF_set_dict,
        total_sim_time=simulation_time,
        max_min_motor_rates=[5,100])

InO_left, InO_left_interface= ROS_Spinnaker_Interface(
        n_neurons_source=n_InO,                 
        Spike_Source_Class=SpikeSourceConstantRate,   
        ros_topic_send='/desired_angles/error', 
        clk_rate=1000,          
        population_coding_param=InO_left_dict,
        total_sim_time=simulation_time)

InO_right, InO_right_interface = ROS_Spinnaker_Interface(
        n_neurons_source=n_InO,         
        Spike_Source_Class=SpikeSourceConstantRate,  
        ros_topic_send='/desired_angles/error', 
        clk_rate=1000,   
        population_coding_param=InO_right_dict,
        total_sim_time=simulation_time) 

recv_pop_interfaces=[MoF_current_interface,MoF_set_interface]
send_pop_interfaces=[MoF_current_interface,MoF_set_interface,InO_left_interface,InO_right_interface]
recv_labels=[interface._output_population.label for interface in recv_pop_interfaces ]
send_labels=[interface._injector_label for interface in send_pop_interfaces]

## Activate live output for receivers
for interface in recv_pop_interfaces:
    sim.external_devices.activate_live_output_for(interface._output_population,
                                              database_notify_port_num=19995)
                                              
## Create live spikes connection
live_spikes_connection = sim.external_devices.SpynnakerLiveSpikesConnection(
    receive_labels=recv_labels,\
    local_port=19995, send_labels=send_labels)

## Initialize callbacks
for interface in recv_pop_interfaces:
    live_spikes_connection.add_receive_callback(interface._output_population.label,interface._incoming_spike_callback)
    
for interface in send_pop_interfaces:
    live_spikes_connection.add_start_resume_callback(interface._injector_label,interface._init_ros_node)


#################### POPULATION DEFINITIONS ################################### 
### MoF layer for current (cur) and desired (set) states
MoF_layer_pop=[]
try:
    MoF_layer_pop.append(MoF_current)
    MoF_layer_pop.append(MoF_set)
except:
    pass
for pop in MoF_layer_pop:
    pop.record()
## InO layer (left and right for each joint)
InO_layer_pop=[]
try:
    InO_layer_pop.append(InO_left)
    InO_layer_pop.append(InO_right)
except:
    pass
for pop in InO_layer_pop:
    pop.record()


#################### PROJECTIONS ############################################## 

## MoF to GrC
out_list= np.arange(0,n_GrC) # Indices of neurons in post-synaptic population [0 ... 255]
proj_MoF_GrC = [] # Projection list of the connections from MoF layer to GrC layer
dimsize = 1 # To have a diagonal shape conn list
scaled_weight_MoF_GrC = raw_weight_MoF_GrC / len(MoF_layer_pop)
for inpop in MoF_layer_pop:
    in_list = (out_list / dimsize) % inpop.size # Create indices of input neurons to be connected to corresponding neurons in out_list [0 1 ... 15 0 1 ... ] or [0 ... 0 1 ... 1 2 ... 2 ... 15]
    dimsize *=inpop.size # To give the diagonal shape to the connectivity matrix
    weights_list = scaled_weight_MoF_GrC * np.ones_like(out_list) # list of weights to create the list connector
    delays_list = np.ones_like(out_list) # list of delays to create the list connector
    connlist = zip (in_list, out_list, weights_list, delays_list) # Create connlist
#    print(in_list,out_list)
    proj_MoF_GrC.append(sim.Projection(inpop,GrC_layer_pop[0],sim.FromListConnector(connlist))) # Connect MoF current and set populations to GrC layer


## MoF to DCN
proj_MoF_DCN=[]

scaled_weight_MoF_DCN=raw_weight_MoF_DCN/len(MoF_layer_pop)

for inpop in MoF_layer_pop:
    for outpop in DCN_layer_pop:
        proj_MoF_DCN.append(sim.Projection(inpop,outpop,sim.AllToAllConnector(weights=scaled_weight_MoF_DCN,delays=1.),target="excitatory",label="proj_"+inpop.label+"_"+outpop.label))    

## PuC to DCN
proj_PuC_DCN = []

repeat_par = n_PuC/n_DCN # Should be 2, such that 2 PuC are mappend to 1 DCN
target_list = np.repeat(range(n_PuC),repeat_par).tolist() # Create target list by mapping 2 PuC to the same DCN
source_list = range(n_DCN)
#connlist_PuC_DCN=zip(source_list,target_list,[raw_weight_PuC_DCN]*n_PuC,[1.0]*n_DCN)
connlist_PuC_DCN=zip(source_list,target_list,[raw_weight_PuC_DCN]*n_PuC,list(np.random.randint(0,10,n_DCN)))
for inpop,outpop in zip(PuC_layer_pop,DCN_layer_pop):
    sim.Projection(inpop,outpop,sim.FromListConnector(connlist_PuC_DCN),target="inhibitory",label="proj_"+inpop.label+"_"+outpop.label)
    pass


## Plastic synapses (GrC to PuC)
tau = 60. #450. # 60.
peak_time = 100.#100.
# Create cerebellar learning rule 
stdp_model = sim.STDPMechanism(
    timing_dependence = TimingDependenceCerebellum(tau=tau, peak_time=peak_time),
    weight_dependence = sim.AdditiveWeightDependence(w_min=0.0, w_max=W_MAX, A_plus=0.01, A_minus=0.01)
)
#weight_dependence = sim.AdditiveWeightDependence(w_min=0.0, w_max=W_MAX, A_plus=0.0015, A_minus=0.0020)
weigth_first=[]
proj_GrC_PuC=[]
for outpop in PuC_layer_pop:
    proj_GrC_PuC.append(sim.Projection(GrC_layer_pop[0],outpop,sim.FixedProbabilityConnector(1.0,weights=weight_dist_GrC_PuC),
                                       synapse_dynamics=sim.SynapseDynamics(slow=stdp_model),
                                        target="excitatory", label="proj_"+GrC_layer_pop[0].label+"_"+outpop.label))
    weigth_first.append(proj_GrC_PuC[-1].getWeights())


## Teaching Signal Projection (InO to PuC)

proj_InO_PuC=[]
for inpop,outpop in zip(InO_layer_pop,PuC_layer_pop):
    proj_InO_PuC.append(sim.Projection(inpop,outpop,sim.OneToOneConnector(weights=0.0,delays=1.0), target= "supervision", label="proj_"+inpop.label+"_"+outpop.label))


#################### EXTERNAL INPUT OUTPUT SETTINGS ###########################


sim.run(simulation_time)
spikes_dcn=[]
for pop in DCN_layer_pop:
    spikes_dcn.append(pop.getSpikes())
spikes_mof=[]
for pop in MoF_layer_pop:
    spikes_mof.append(pop.getSpikes())
spikes_ino=[]
for pop in InO_layer_pop:
    spikes_ino.append(pop.getSpikes())
spikes_puc=[]
for pop in PuC_layer_pop:
    spikes_puc.append(pop.getSpikes())
spikes_grc=[]
for pop in GrC_layer_pop:
    spikes_grc.append(pop.getSpikes())
wei=[]
for proj in proj_GrC_PuC:
    wei.append(proj.getWeights())
print(wei)

print("***","DCN_spikes:",len(spikes_dcn[0]),len(spikes_dcn[1]))
print("***","MoF_spikes:",len(spikes_mof[0]),len(spikes_mof[1]))
print("***","InO_spikes:",len(spikes_ino[0]),len(spikes_ino[1]))
print("***","PuC_spikes:",len(spikes_puc[0]),len(spikes_puc[1]))
print("***","GrC_spikes:",len(spikes_grc[0]))
np.save("last_stdp_weights.npy",wei)
import pylab as plt
plt.plot(wei[0]);plt.hold(True)
plt.plot(wei[1])
plt.hold(False)
plt.figure()
spike_time_mof_1=[i[1] for i in spikes_mof[1]]
spike_id_mof_1=[i[0] for i in spikes_mof[1]]
plt.plot(spike_time_mof_1,spike_id_mof_1,'.')
plt.show()
IPython.embed()

sim.end()




























