import xml.dom.minidom
from xml.etree.ElementTree import Element, SubElement, tostring
import random
import dm_control.mujoco
from dm_control import mujoco
import time
import numpy as np

class SlidingCreatureMuJoCoModel:
    # Functions to create the creature structure
    def __init__(self, num_limbs):
        self.root = Element('mujoco', model="sliding_creature")
        self.num_limbs = num_limbs

    def add_option(self):
        SubElement(self.root, 'option', timestep="0.01")

    def add_default(self):
        default = SubElement(self.root, 'default')
        SubElement(default, 'joint', armature="0.1")

    def add_asset(self):
        asset = SubElement(self.root, 'asset')
        SubElement(asset, 'material', name="color1_limb", rgba="0.3 0.5 0.8 1")
        SubElement(asset, 'material', name="color_plane", rgba="0.6 0.2 0.2 1")
        SubElement(asset, 'material', name="color2_limb", rgba="0.9 0.9 0.1 1")

    def add_worldbody(self):
        worldbody = SubElement(self.root, 'worldbody')
        SubElement(worldbody, 'light', cutoff="100", diffuse="1 1 1", dir="-0 0 -1.3", directional="true", exponent="1", pos="0 0 1.3", specular=".1 .1 .1")
        SubElement(worldbody, 'geom', conaffinity="1", condim="3", material="color_plane", name="floor", pos="0 0 -0.1", size="40 40 0.1", type="plane")
        torso = SubElement(worldbody, 'body', name="torso", pos="0 0 0")
        SubElement(torso, 'camera', name="track", mode="trackcom", pos="0 -3 3", xyaxes="1 0 0 0 1 1")
        SubElement(torso, 'geom', density="1000", fromto="1.5 0 0 0.5 0 0", size="0.1", type="capsule", material="color2_limb")
        SubElement(torso, 'joint', name="free_body_rot", pos="0 0 0", type="free")

        # Recursive function to add limbs
        def add_limb(parent, limb_number, pos):
            if limb_number > self.num_limbs:
                return
            limb_name = f"limb{limb_number}"
            body = SubElement(parent, 'body', name=limb_name, pos=pos)
            material = "color1_limb" if limb_number % 2 else "color2_limb"
            SubElement(body, 'geom', density="1000", fromto="0 0 0 -1 0 0", size="0.1", type="capsule", material=material)
            SubElement(body, 'joint', axis="0 0 1", limited="true", name=f"joint{limb_number}", pos="0 0 0", range="-100 100", type="hinge")
            add_limb(body, limb_number + 1, "-1 0 0")

        add_limb(torso, 1, "0.5 0 0")

    def add_actuator(self):
        actuator = SubElement(self.root, 'actuator')
        for i in range(1, self.num_limbs + 1):
            SubElement(actuator, 'motor', ctrllimited="true", ctrlrange="-1 1", gear="300.0", joint=f"joint{i}")
    
    # MUTATION FUNCTIONS 
    # Adds a new limb to the creature
    def add_limb(self, length=1, size=0.1, gear=300.0):
        # get the last limb to which we will attach the new limb
        last_limb_name = f"limb{self.num_limbs}"
        last_limb = self.root.find(f".//body[@name='{last_limb_name}']")
        
        # Get the from to of the last limb to determine the position of the new limb using last limb's length
        geom = last_limb.find('geom')
        fromto = geom.get('fromto').split()
        fromto_floats = list(map(float, fromto))

        # Use the number of limbs to determine the new limb's name 
        limb_name = f"limb{self.num_limbs + 1}"
        motor_name = f"joint{self.num_limbs + 1}"

        # Determine the position of the new limb
        pos = f"{fromto_floats[3]} 0 0"
        # Determine the color of the new limb
        material = "color2_limb" if self.num_limbs % 2 else "color1_limb"

        # Add the new limb to the worldbody
        body = SubElement(last_limb, 'body', name=limb_name, pos=pos)
        SubElement(body, 'geom', density="1000", fromto=f"0 0 0 -{length} 0 0", size=str(size), type="capsule", material=f"{material}")
        SubElement(body, 'joint', axis="0 0 1", limited="true", name=motor_name, pos="0 0 0", range="-100 100", type="hinge")

        # Add an actuator for the new limb's joint
        actuators = self.root.find('.//actuator')
        if actuators is None:
            actuators = SubElement(self.root, 'actuator')
        SubElement(actuators, 'motor', ctrllimited="true", ctrlrange="-1 1", gear=str(gear), joint=motor_name)

        # Increment the limb count
        self.num_limbs += 1
        
    # Deletes the last n limbs
    def delete_limb(self, num_limbs_to_be_deleted):
        delete_from = self.num_limbs - num_limbs_to_be_deleted + 1
        limb_name = f"limb{delete_from-1}"
        limb = self.root.find(f".//body[@name='{limb_name}']")
        body = limb.find('body')
        limb.remove(body)

        for i in range(delete_from, self.num_limbs + 1):
            motor_name = f"joint{i}"
            actuators = self.root.find('.//actuator')
            motor = actuators.find(f".//motor[@joint='{motor_name}']")
            actuators.remove(motor)

        self.num_limbs -= num_limbs_to_be_deleted

    # Changes the gear values of the motors across the limbs
    # option values can be "random", "incremental", "alternate", "middle", "front_back"
    # "random" - Randomly assigns gear values to the motors
    # "incremental" - Assigns gear values to the motors in an incremental fashion
    # "alternate" - Assigns gear values to the motors in an alternate fashion
    # "middle" - Assigns larger gear values to the motors in the middle of the creature
    # "front_back" - Assigns larger gear values to the motors in the front and back of the creature
    def change_motor_values(self, option="random", gear=100.0):
        if option == "random":
            for i in range(1, self.num_limbs + 1):
                motor = self.root.find(f".//motor[@joint='joint{i}']")
                if motor is not None:
                    motor.set('gear', str(random.randint(100, 1000)))

        elif option == "incremental":
            gear = 300
            for i in range(1, self.num_limbs + 1):
                motor = self.root.find(f".//motor[@joint='joint{i}']")
                if motor is not None:
                    motor.set('gear', str(gear))
                if i % 2 == 0:
                    gear += 100
                if i % 4 == 0:
                    gear += 100

        elif option == "alternate":
            for i in range(1, self.num_limbs + 1):
                motor = self.root.find(f".//motor[@joint='joint{i}']")
                if motor is not None:
                    gear = 300 if i % 2 == 1 else 500
                    motor.set('gear', str(gear))

        elif option == "middle":
            for i in range(2, self.num_limbs):
                motor = self.root.find(f".//motor[@joint='joint{i}']")
                if motor is not None:
                    motor.set('gear', str(600))

        elif option == "front_back":
            for i in range(1, self.num_limbs + 1):
                motor = self.root.find(f".//motor[@joint='joint{i}']")
                if motor is not None:
                    if i == 1 or i == self.num_limbs:
                        motor.set('gear', str(600))

        elif option == "all":
            for i in range(1, self.num_limbs + 1):
                motor = self.root.find(f".//motor[@joint='joint{i}']")
                if motor is not None:
                    motor.set('gear', str(gear))

    # Fixes the joint ranges across the limbs
    # option values can be "alternate", "middle", "front_back"
    # "alternate" - Fixes the joint ranges across the limbs in an alternate fashion
    # "middle" - Fixes the joint ranges across the limbs in the middle of the creature
    # "front_back" - Fixes the joint ranges across the limbs in the front and back of the creature
    def fix_joints(self, option="alternate"):
        if option == "alternate":
            for i in range(1, self.num_limbs + 1):
                joint = self.root.find(f".//joint[@name='joint{i}']")
                if joint is not None:
                    joint_range = "-1 1" if i % 2 == 1 else "-100 100"
                    joint.set('range', joint_range)
                    self.change_motor_values(option="all", gear=700)

        elif option == "middle":
            for i in range(2, self.num_limbs):
                joint = self.root.find(f".//joint[@name='joint{i}']")
                if joint is not None:
                    joint.set('range', "-1 1")
                    self.change_motor_values(option="all", gear=700)

        elif option == "front_back":
            for i in range(1, self.num_limbs + 1):
                joint = self.root.find(f".//joint[@name='joint{i}']")
                if joint is not None:
                    if i == 1 or i == self.num_limbs:
                        joint.set('range', "-1 1")
                        self.change_motor_values(option="all", gear=700)


    # Changes the lengths of the limbs
    # option values can be "random", "front", "back"
    # "random" - Randomly changes the lengths of the limbs
    # "front" - Changes the length of the front limb
    # "back" - Changes the length of the back limb
    # "all" - Changes the length of all the limbs
    def change_lengths(self, option, length=None):
        if option == "random":
            torso = self.root.find(".//body[@name='torso']")
            if torso is not None:
                length = random.uniform(0.5, 2.1)
                geom = torso.find('geom')
                fromto = geom.get('fromto').split()
                fromto_floats = list(map(float, fromto))
                geom.set('fromto', f"{(fromto_floats[3]+length):.2f} 0 0 {fromto_floats[3]:.2f} 0 0")
                for i in range(1, self.num_limbs + 1):
                    length = random.uniform(0.5, 3.0)
                    limb = self.root.find(f".//body[@name='limb{i}']")
                    limb.set('pos', f"{fromto_floats[3]:.2f} 0 0")
                    geom = limb.find('geom')
                    if geom is not None:
                        geom.set('fromto', f"0 0 0 -{length:.2f} 0 0")
                        fromto = geom.get('fromto').split()
                        fromto_floats = list(map(float, fromto))
                        if length > 2:
                            motor = self.root.find(f".//motor[@joint='joint{i}']")
                            if motor is not None:
                                motor.set('gear', str(1000))
        
        elif option == "front":
            if length is None:
                length = random.uniform(0.5, 2.1)
            torso = self.root.find(".//body[@name='torso']")
            if torso is not None:
                geom = torso.find('geom')
                fromto = geom.get('fromto').split()
                fromto_floats = list(map(float, fromto))
                geom.set('fromto', f"{(fromto_floats[3]+length):.2f} 0 0 {fromto_floats[3]:.2f} 0 0")
                if length > 2:
                    motor = self.root.find(".//motor[@joint='joint1']")
                    if motor is not None:
                        motor.set('gear', str(1000))
        
        elif option == "back":
            if length is None:
                length = random.uniform(0.5, 2.1)
            limb = self.root.find(f".//body[@name='limb{self.num_limbs}']")
            if limb is not None:
                geom = limb.find('geom')
                fromto = geom.get('fromto').split()
                fromto_floats = list(map(float, fromto))
                geom.set('fromto', f"0 0 0 -{length:.2f} 0 0")
                if length > 2:
                    motor = self.root.find(f".//motor[@joint='joint{self.num_limbs}']")
                    if motor is not None:
                        motor.set('gear', str(1000))
        
        elif option == "all":
            torso = self.root.find(".//body[@name='torso']")
            if torso is not None:
                if length is None:
                    length = random.uniform(0.5, 2.1)
                geom = torso.find('geom')
                fromto = geom.get('fromto').split()
                fromto_floats = list(map(float, fromto))
                geom.set('fromto', f"{(fromto_floats[3]+length):.2f} 0 0 {fromto_floats[3]:.2f} 0 0")
                for i in range(1, self.num_limbs + 1):
                    limb = self.root.find(f".//body[@name='limb{i}']")
                    limb.set('pos', f"{fromto_floats[3]:.2f} 0 0")
                    geom = limb.find('geom')
                    if geom is not None:
                        geom.set('fromto', f"0 0 0 -{length:.2f} 0 0")
                        fromto = geom.get('fromto').split()
                        fromto_floats = list(map(float, fromto))
                        if length > 2:
                            motor = self.root.find(f".//motor[@joint='joint{i}']")
                            if motor is not None:
                                motor.set('gear', str(1000))

    # Changes the size of the limbs
    def change_size(self, size=None):
        if size is None:
            size = random.uniform(0.1, 0.5)
        for geom in self.root.iter('geom'):
            name = geom.get('name')
            if not name:
                geom.set('size', str(size))

    # Generates the creature
    # used to get original creature as well
    def generate_creature(self, num_limbs=None):
        self.root = Element('mujoco', model="sliding_creature")
        if num_limbs is not None:
            self.num_limbs = num_limbs
        self.add_option()
        self.add_default()
        self.add_asset()
        self.add_worldbody()
        self.add_actuator()

    # util function to generate xml
    def generate_xml(self):
        rough_string = tostring(self.root, 'utf-8')
        reparsed = xml.dom.minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ")

# function to write xml
def write_xml(xml_string, xml_name):
    with open(xml_name, "w") as f:
        f.write(xml_string)

# function to calculate distance for fitness
def calculate_distance(initial_position, final_position):
    return np.sqrt((final_position[0] - initial_position[0])**2 + (final_position[1] - initial_position[1])**2)

# function to calculate energy consumed for fitness
def calculate_energy_consumed(control_signals, timestep):
    # Simplified energy calculation: sum of absolute control signals over time
    return np.sum(np.abs(control_signals)) * timestep

# function to calculate stability for fitness
def calculate_stability_heights(heights):
    # If the height is too much, it means that the creature is not stable or flying off
    # print(heights)
    return np.abs(heights).sum()

# function to calculate fitness
def calculate_fitness(distance_traveled, energy_consumed, stability_variance):
    # Fitness function: weighted sum of distance traveled, energy consumed, and stability
    # The only goal is to maximize distance traveled and make sure that the creature is not too unstable
    weight_distance = 1
    weight_energy = 0.1
    weight_stability = -0.01

    # If the creature travels more than 200 units, it means it has flown off and is unstable, so we don't want to reward that
    if distance_traveled > 200:
        weight_distance = 0

    # If the fitness is negative, we set it to 0
    return max(0,(weight_distance * distance_traveled) + (weight_energy * 1 / (1+ energy_consumed)) + (weight_stability *stability_variance))

# function to simulate the creature
def simulate(xml_path):
    model = dm_control.mujoco.MjModel.from_xml_path(xml_path)
    data = dm_control.mujoco.MjData(model)

    total_steps = 1000  # Total number of steps to simulate
    timestep = 0.01  # Simulation timestep

    # Initial setup for tracking
    initial_position = np.copy(data.qpos[:2])  
    energy_consumed = 0
    heights = []  # Track the z-coordinate for stability
    control_signals = []

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 65
        viewer.cam.azimuth = 60
        viewer.cam.elevation = -15

        for step in range(total_steps):
            current_time = step * timestep
            amplitude = 2
            phase_offset = np.pi / 2  # Adjust for smoother transition between segments
            frequency = 1  
            for i in range(len(data.ctrl)):
                control_signal = amplitude * np.sin(2 * np.pi * frequency * current_time + i * phase_offset)
                data.ctrl[i] = control_signal
                control_signals.append(control_signal)
            dm_control.mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(timestep)
            # Track height for stability
            heights.append(data.qpos[2])

    final_position = np.copy(data.qpos[:2])

    distance_traveled = calculate_distance(initial_position, final_position)
    energy_consumed = calculate_energy_consumed(control_signals, timestep)
    stability_variance = calculate_stability_heights(heights)
    
    fitness = calculate_fitness(distance_traveled, energy_consumed, stability_variance)
    viewer.close()
    return fitness

# function to simulate the creature without the viewer for faster simulation and evaluation of evolving creatures in iterations
def simulate_without_viewer(xml_string, xml_path = None):
    model = dm_control.mujoco.MjModel.from_xml_string(xml_string)
    if xml_path is not None:
        model = dm_control.mujoco.MjModel.from_xml_path(xml_path)
    data = dm_control.mujoco.MjData(model)

    total_steps = 2000  # Total number of steps to simulate
    timestep = 0.01  # Simulation timestep

    # Initial setup for tracking
    initial_position = np.copy(data.qpos[:2])
    control_signals = []
    heights = []  # Track the z-coordinate for stability

    for step in range(total_steps):
        current_time = step * timestep
        amplitude = 2
        phase_offset = np.pi / 2  # Adjust for smoother transition between segments
        frequency = 1
        for i in range(len(data.ctrl)):
            control_signal = amplitude * np.sin(2 * np.pi * frequency * current_time + i * phase_offset)
            data.ctrl[i] = control_signal
            control_signals.append(control_signal)
        dm_control.mujoco.mj_step(model, data)
        # Track height for stability
        heights.append(data.qpos[2])

    final_position = np.copy(data.qpos[:2])

    distance_traveled = calculate_distance(initial_position, final_position)
    energy_consumed = calculate_energy_consumed(control_signals, timestep)
    stability_variance = calculate_stability_heights(heights)

    fitness = calculate_fitness(distance_traveled, energy_consumed, stability_variance)
    del model
    del data

    return fitness