import random
import xml.dom.minidom
from xml.etree.ElementTree import Element, SubElement, tostring
import dm_control.mujoco
import mujoco.viewer
import time
import numpy as np
import csv

# Class to generate a quadruped model using Mujoco
# The creature includes a torso and four legs with hip and ankle joints
# The model creates a simple ant with random ranges of motion with a torso and four legs
# Then generates randomly sized creatures using the structure of the model with random ranges of motion
# The model randomly adjusts the size of the torso, the lengths of the legs, the thickness of the legs, and the strength of the actuators within a certain range
# All classes and functions take values as well (to test the function with different values)
# No values are provided to create random creatures
class QuadrupedMujocoModel:
    def __init__(self, ankle_range=None, hip_range=None):
        self.root = Element('mujoco', model="simple_quadruped")
        self.counter = 1
        
        # Set random ranges of motion for the hip and ankle joints
        self.hip_range_options = ["-30 30", "-10 10", "-40 40"]
        self.hip_range = hip_range if hip_range else random.choice(self.hip_range_options)

        self.ankle_range_options = [[30, 70], [5, 10], [80, 90]]
        self.ankle_range = ankle_range if ankle_range else random.choice(self.ankle_range_options)

    # Functions to add elements to the XML model
    def add_option(self, timestep=0.01):
        option = SubElement(self.root, 'option', timestep=str(timestep))

    def add_default(self):
        default = SubElement(self.root, 'default')
        joint = SubElement(default, 'joint', armature="1", damping="1", limited="true")
        geom = SubElement(default, 'geom', conaffinity="0", condim="3", density="5.0", friction="1 0.5 0.5", margin="0.01")

    def add_worldbody(self):
        worldbody = SubElement(self.root, 'worldbody')
        floor = SubElement(worldbody, 'geom', conaffinity="1", condim="3", name="floor", pos="0 0 0", rgba="0.7 0.9 0.7 1", size="40 40 40", type="plane")
        torso = SubElement(worldbody, 'body', name="torso", pos="0 0 0.75")  # Changed pos to "0 0 0.75"
        torso_geom = SubElement(torso, 'geom', name="torso_geom", pos="0 0 0", size="0.25", type="sphere", rgba="1 0.5 0.5 1")
        root_joint = SubElement(torso, 'joint', armature="0", damping="0", limited="false", margin="0.01", name="root", pos="0 0 0", type="free")
        
        # Functions to add legs
        def add_leg(parent, name, pos, hip_range, ankle_range, leg_fromto, ankle_fromto, ankle_axis):
            nonlocal self
            leg = SubElement(parent, 'body', name=name, pos=pos)
            hip_joint = SubElement(leg, 'joint', axis="0 0 1", name=f"hip_{self.counter}", pos="0.0 0.0 0.0", range=hip_range, type="hinge")
            leg_geom = SubElement(leg, 'geom', fromto=leg_fromto, name=f"{name}_leg_geom", size="0.08", type="capsule")
            ankle_body = SubElement(leg, 'body', pos=pos)  # Changed pos to match the leg's pos
            ankle_joint = SubElement(ankle_body, 'joint', axis=ankle_axis, name=f"ankle_{self.counter}", pos="0.0 0.0 0.0", range=ankle_range, type="hinge")
            ankle_geom = SubElement(ankle_body, 'geom', fromto=ankle_fromto, name=f"{name}_ankle_geom", size="0.08", type="capsule")
            self.counter += 1

        ankle_range = self.ankle_range
        hip_range = self.hip_range

        add_leg(torso, "front_left_leg", "0.2 0.2 0", hip_range, f"{ankle_range[0]} {ankle_range[1]}", "0.0 0.0 0.0 0.2 0.2 0.0", "0.0 0.0 0.0 0.4 0.4 0.0", "-1 1 0")
        add_leg(torso, "front_right_leg", "-0.2 0.2 0", hip_range, f"-{ankle_range[1]} -{ankle_range[0]}", "0.0 0.0 0.0 -0.2 0.2 0.0", "0.0 0.0 0.0 -0.4 0.4 0.0", "1 1 0")
        add_leg(torso, "back_left_leg", "0.2 -0.2 0", hip_range, f"{ankle_range[0]} {ankle_range[1]}", "0.0 0.0 0.0 0.2 -0.2 0.0", "0.0 0.0 0.0 0.4 -0.4 0.0", "1 1 0")
        add_leg(torso, "back_right_leg", "-0.2 -0.2 0", hip_range, f"-{ankle_range[1]} -{ankle_range[0]}", "0.0 0.0 0.0 -0.2 -0.2 0.0", "0.0 0.0 0.0 -0.4 -0.4 0.0", "-1 1 0")

    def add_actuator(self):
        actuator = SubElement(self.root, 'actuator')
        motors = ["hip", "ankle"]
        for i in range(1, self.counter):
            for motor in motors:
                motor_element = SubElement(actuator, 'motor', ctrllimited="true", ctrlrange="-1.0 1.0", joint=f"{motor}_{i}", gear="150")

    # util function to generate the XML string
    def generate_xml(self):
        rough_string = tostring(self.root, 'utf-8')
        reparsed = xml.dom.minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ")
    
    # Randomization Functions

    # Function to create a quadruped model with random torso sizes
    # If the torso_size parameter is too large, the legs and motors are also adjusted
    # Added torso_size parameter to test the function with different sizes
    # If size is not provided, a random size is generated
    def adjust_torso_size(self, torso_size=None):
        new_size = 0 
        if torso_size:
            new_size = torso_size 
        else:
            new_size = random.uniform(0.2, 1.0)
        
        # Find the torso geom and update its size
        for geom in self.root.iter('geom'):
            if geom.get('name') == 'torso_geom':
                geom.set('size', str(new_size))
                break

        for body in self.root.iter('body'):
            name = body.get('name')
            if name and name == 'torso':
                body.set('pos', str(f'0 0 {new_size}'))
                break
        
        # Update related positions proportionately
        scale_factor = new_size/0.25 # Original size was 0.25
        leg_size = new_size - new_size/5  # Adjusted leg size based on the torso size
        
        # Update leg position
        for body in self.root.iter('body'):
            name = body.get('name')
            if name and 'leg' in name:
                new_x = (1 if 'left' in name else -1) * leg_size
                new_y = (1 if 'front' in name else -1) * leg_size
                new_z = 0 
                body.set('pos', f" {new_x} {new_y} 0")
        
        if new_size > 0.5:
            new_size = 0.2 * scale_factor * 1.3
            new_thickness = 0.08 * scale_factor / 2
            self.adjust_leg_lengths(new_size + 0.3, new_size)
            self.adjust_leg_thickness(new_thickness)
            self.adjust_actuator_strength(300)

    # Function to create a quadruped model with random leg lengths
    # If the lower_leg_length and upper_leg_length parameters are too large, the motors are also adjusted
    # Added lower_leg_length and upper_leg_length parameters to test the function with different sizes
    def adjust_leg_lengths(self, lower_leg_length=None, upper_leg_length=None):
        # Generate random lengths for the lower and upper parts of the legs, if not provided
        lower_leg_length_new = 0
        upper_leg_length_new = 0
        if lower_leg_length and upper_leg_length:
            lower_leg_length_new = lower_leg_length
            upper_leg_length_new = upper_leg_length
        else: 
            lower_leg_length_new = random.uniform(0.2, 1.5)
            upper_leg_length_new = random.uniform(0.2, 1.5)
        
        if lower_leg_length_new > 0.75 or upper_leg_length_new > 0.75:
            self.adjust_actuator_strength(300)

        for body in self.root.iter('body'):
            name = body.get('name')
            if name and 'leg' in name:
                
                # pos = body.get('pos').split()
                for geom in body.iter('geom'):
                    name = geom.get('name')
                    
                    if name and 'leg_geom' in name:
                        fromto = geom.get('fromto').split()
                        fromto_floats = list(map(float, fromto))
                        new_x = (1 if 'left' in name else -1) * upper_leg_length_new
                        new_y = (1 if 'front' in name else -1) * upper_leg_length_new
                        geom.set('fromto', f"{fromto_floats[0]} {fromto_floats[1]} {fromto_floats[2]} {new_x} {new_y} {fromto_floats[5]}")
                        
                    elif name and 'ankle_geom' in name:
                        fromto = geom.get('fromto').split()
                        fromto_floats = list(map(float, fromto))
                        new_x = (1 if 'left' in name else -1) * lower_leg_length_new
                        new_y = (1 if 'front' in name else -1) * lower_leg_length_new
                        geom.set('fromto', f"{fromto_floats[0]} {fromto_floats[1]} {fromto_floats[2]} {new_x} {new_y} {fromto_floats[5]}")
                        
                for child_body in body.iter('body'):
                    name = child_body.get('name')
                    if not name:
                        child_pos = child_body.get('pos').split()
                        new_x = (1 if 'left' in geom.get('name') else -1) * upper_leg_length_new
                        new_y = (1 if 'front' in geom.get('name') else -1) * upper_leg_length_new
                        child_body.set('pos', f"{new_x:.2f} {new_y:.2f} {child_pos[2]}")

    # Function to create a quadruped model with random leg thickness
    # If the leg_thickness parameter is too large, the motors are also adjusted
    # Added leg_thickness parameter to test the function with different sizes
    def adjust_leg_thickness(self, leg_thickness=None):
        # Generate a random thickness for the legs
        new_thickness = 0
        if leg_thickness:
            new_thickness = leg_thickness
        else:
            new_thickness = random.uniform(0.05, 0.16)
        # Adjust the 'size' attribute for legs and ankles
        for geom in self.root.iter('geom'):
            name = geom.get('name')
            if name:
                if 'leg_geom' in geom.get('name') or 'ankle_geom' in geom.get('name'):
                    geom.set('size', str(new_thickness))
    
    # Function to create a quadruped model with random actuator strength
    # Added actuator_strength parameter to test the function with different strengths
    def adjust_actuator_strength(self, actuator_strength=None):
        new_strength = 0
        if actuator_strength:
            new_strength = actuator_strength
        else:
            new_strength = random.uniform(150, 300)
        for motor in self.root.iter('motor'):
            motor.set('gear', str(new_strength))
    
    # Function to get original model for testing
    def get_original_model(self):
        self.adjust_torso_size(0.25)
        self.adjust_leg_lengths(0.4, 0.2)
        self.adjust_leg_thickness(0.08)
        self.adjust_actuator_strength(150)
        return self.root

    # Function to get fitness measurements for fitness calculation
    def get_fitness_measurements(self):
        sizes = []
        for geom in self.root.iter('geom'):
            name = geom.get('name')
            if name and name != 'floor':
                size = geom.get('size')
                sizes.append(float(size))
        total_size = sum(sizes)
        return total_size, self.hip_range, self.ankle_range

# Function to calculate the fitness score of the model
# Not considreing total steps because it is constant for all simulations
def calculate_fitness_score(initial_position, data, weight):
    # Calculate the absolute distance traveled from the initial position
    distance_traveled = abs(data.qpos[0] - initial_position[0])
    
    # Stability measure - sum of squared velocities. Lower is better, so inverting the measure.
    stability_measure = 1 / (np.sum(data.qvel**2) + 1)
    
    # Integrate the weight into the fitness score
    # Since weight is a negative factor, inverting it to make it positive to minimize the effect
    weight_factor = 1 / (weight + 1) 
    
    # Combining the measures into a single fitness score
    # Giving equal weight to each measure in this calculation
    fitness = (distance_traveled * 0.5) + (stability_measure * 0.5) + (weight_factor * 0.5)
    
    return fitness

# Function to simulate the model with a galloping motion
def simulate(xml_path, weight):
    # Load the model from the XML file
    model = dm_control.mujoco.MjModel.from_xml_path(xml_path)
    data = dm_control.mujoco.MjData(model)

    # Value for calculating fitness
    initial_position = np.copy(data.qpos)

    total_steps = 1000  # Total number of steps to simulate
    timestep = 0.01  # Simulation timestep
    ramp_up_steps = 100  # Number of steps to gradually apply control

    fitness = 0
    # Simulate the model and apply control signals
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set the camera position
        viewer.cam.distance = 20
        viewer.cam.azimuth = 60
        viewer.cam.elevation = -15

        # Iterate through the simulation steps
        for step in range(total_steps):
            ramp_factor = min(step / ramp_up_steps, 1)  # Gradually increase control signal
            current_time = step * timestep

            # Galloping pattern: create a phase offset for each leg
            for leg in range(4):  # 4 legs
                hip_index = leg * 2
                ankle_index = leg * 2 + 1

                # Phase offsets for a galloping motion
                # Front legs have similar phases, and back legs follow a similar pattern
                if leg < 2:  # Front legs
                    phase_offset = np.pi / 3
                else:  # Back legs
                    phase_offset = np.pi

                # Applying the galloping pattern with slight variations for each leg
                # Hip movement
                data.ctrl[hip_index] = np.sin(current_time * 4 + phase_offset) * ramp_factor
                # Ankle movement, keeping the ankles relatively stable
                data.ctrl[ankle_index] = -0.3 * ramp_factor  # Slightly lift the feet off the ground

            # Step the simulation and sync the viewer
            dm_control.mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(timestep)

    viewer.close()

    # Calculate the fitness score
    fitness = calculate_fitness_score(initial_position, data, weight)
    print(f"Fitness: {fitness}")

    return fitness
        
# util function to write the XML string to a file
def write_xml_file(xml_string, file_name):
    with open(file_name, "w") as xml_file:
        xml_file.write(xml_string)

def main(): 
    # Create Creature
    # Data rows of the CSV file
    rows = [
        ["weight_measure", "hip_range", "ankle_range", "fitness"]
    ]
    
    # Generates random ranges of motion
    mujoco_model = QuadrupedMujocoModel()
    mujoco_model.add_option()
    mujoco_model.add_default()
    mujoco_model.add_worldbody()
    mujoco_model.add_actuator()
    weight, hip_range, ankle_range = mujoco_model.get_fitness_measurements()
    
    xml_string = mujoco_model.generate_xml()
    xml_name = "mujoco_model_range_of_motion_1.xml"
    write_xml_file(xml_string, xml_name)
    fitness = simulate(xml_name, weight)
    row = []
    row = [weight, hip_range, ankle_range, fitness]
    rows.append(row)
    time.sleep(3)

    # Model Initialization for randomizations of actuator strength, leg thickness, leg lengths, and torso size
    # The later part can be commented and we can directly randomise sizes as well
    # Doing it in chinks to show changes in isolation
    ankle_range = [30,70]
    hip_range = "-30 30"
    mujoco_model = QuadrupedMujocoModel(ankle_range=ankle_range, hip_range=hip_range)
    mujoco_model.add_option()
    mujoco_model.add_default()
    mujoco_model.add_worldbody()
    mujoco_model.add_actuator()

    # Torso Size
    mujoco_model.adjust_torso_size()
    weight, hip_range, ankle_range = mujoco_model.get_fitness_measurements()

    xml_string = mujoco_model.generate_xml()
    xml_name = "mujoco_model_torso_size.xml"
    write_xml_file(xml_string, xml_name)
    fitness = simulate(xml_name, weight)
    row = []
    row = [weight, hip_range, ankle_range, fitness]
    rows.append(row)
    time.sleep(3)

    mujoco_model.get_original_model()
    # Get Random Leg Lengths
    mujoco_model.adjust_leg_lengths()
    weight, hip_range, ankle_range = mujoco_model.get_fitness_measurements()

    xml_string = mujoco_model.generate_xml()
    xml_name = "mujoco_model_leg_lengths.xml"
    write_xml_file(xml_string, xml_name)
    fitness = simulate(xml_name, weight)
    row = []
    row = [weight, hip_range, ankle_range, fitness]
    rows.append(row)
    time.sleep(3)

    mujoco_model.get_original_model()
    # Get Random Leg Thicknes
    mujoco_model.adjust_leg_thickness()
    weight, hip_range, ankle_range = mujoco_model.get_fitness_measurements()

    xml_string = mujoco_model.generate_xml()
    xml_name = "mujoco_model_leg_thickness.xml"
    write_xml_file(xml_string, xml_name)
    fitness = simulate(xml_name, weight)
    row = []
    row = [weight, hip_range, ankle_range, fitness]
    rows.append(row)
    time.sleep(3)

    mujoco_model.get_original_model()
    # Get Random Actuator Strength
    mujoco_model.adjust_actuator_strength()
    weight, hip_range, ankle_range = mujoco_model.get_fitness_measurements()

    xml_string = mujoco_model.generate_xml()
    xml_name = "mujoco_model_actuator_strength.xml"
    write_xml_file(xml_string, xml_name)
    fitness = simulate(xml_name, weight)
    row = []
    row = [weight, hip_range, ankle_range, fitness]
    rows.append(row)
    time.sleep(3)

    # Name of the CSV file
    filename = "fitness_tracker.csv"
    # Writing to the CSV file
    with open(filename, 'w', newline='') as csvfile:
        # Creating a csv writer object
        csvwriter = csv.writer(csvfile)
        
        # Writing the data rows
        csvwriter.writerows(rows)

if __name__=="__main__": 
    main() 