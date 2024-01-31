import dm_control.mujoco
from dm_control import mjcf
import mujoco.viewer
import time
import numpy as np

# Create Creature Structure
xml_filename = "walking_creature.xml"

class Crab:
    def __init__(self):
        self.model = mjcf.RootElement()
        self.model.option.timestep = 0.01
        self.model.visual.headlight.ambient = [0.5, 0.5, 0.5]
        self.model.worldbody.add('light', diffuse='.5 .5 .5', pos='0 0 3', dir='0 0 -1')
        self.model.worldbody.add('geom', type='plane', size='50 50 0.1', rgba='1 1 .2 1')

        self.main_body = self.model.worldbody.add('body', pos='0 0 20', euler='0 180 0')
        self.main_body.add('joint', type='free', axis='-1 0 0', pos='0 0 -0.5')
        self.main_body.add('geom', type='ellipsoid', size='1.58 1 0.8', rgba='225 0 225 1', mass='1')

        self.limbs = []
        limb_positions_and_eulers = [
            ("0 -1 0", "30 0 0"),    # Limb 1
            ("0 1 0", "-30 0 0"),    # Limb 2
            ("1 -0.6 0", "30 0 0"),  # Limb 3
            ("1 0.6 0", "-30 0 0"),  # Limb 4
            ("-1 -0.6 0", "30 0 0"), # Limb 5
            ("-1 0.6 0", "-30 0 0")  # Limb 6
        ]

        for i, (pos, euler) in enumerate(limb_positions_and_eulers, start=1):
            self.create_limb(f'limb{i}', pos, euler)

    def create_limb(self, name, pos, euler):
        limb = self.main_body.add('body', name=name, pos=pos, euler=euler)

        # Each limb consists of 3 parts
        for part in range(1, 4):
            part_name = f'{name}_part{part}'
            geom_fromto = '0 0 0 0 0 1'
            limb.add('geom', type='cylinder', fromto=geom_fromto, size='0.1 0.3', rgba='0 1 0 1')
            joint_name = f'{name}_joint{part}'
            joint_pos = '0 0 1'
            limb.add('joint', name=joint_name, type='hinge', axis='0 0 1', pos=joint_pos, range='-45 45')
        
            # Nested body for parts 2 and 3
            if part < 3:
                euler = '0 45 0' if part == 1 else '0 -90 0'
                limb = limb.add('body', name=part_name, pos='0 0 1', euler=euler)
        
        self.limbs.append(limb)

    def create_actuator(self):
        print(self.limbs)
        for idx, limb in enumerate(self.limbs):
            for part in range(1, 4):
                joint_name = f'limb{idx+1}_joint{part}'
                self.model.actuator.add('motor', name=f'motor_{joint_name}', joint=joint_name, gear='50', ctrllimited='true', ctrlrange='-45 45')


crab_creature = Crab()
crab_creature.create_actuator()

xml_str = crab_creature.model.to_xml_string()

# Write the XML string to a file
with open(xml_filename, "w") as file:
    file.write(xml_str)

# Move the Body
model = dm_control.mujoco.MjModel.from_xml_path(xml_filename)
data = dm_control.mujoco.MjData(model)

# Viewing Parameters
total_movt = 1500 
timestep = 0.01  

# Open the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 20
    viewer.cam.azimuth = 60
    viewer.cam.elevation = -15  

    wave_speed = 2  # Controls the speed of the wave
    wave_length = 3  # Controls the length of the wave

    for step in range(total_movt):
        for limb in range(6):  # 6 limbs
            for part in range(3):  # 3 parts per limb
                joint_index = limb*3 + part

                # Calculate the phase based on limb and part position
                phase = (limb + part) * np.pi / wave_length

                # Create a moving wave based on the step and phase
                angle = np.deg2rad(45) * np.sin(2 * np.pi * wave_speed * step * timestep + phase)

                # Apply the angle to the joint
                data.ctrl[joint_index] = angle*60

        # Step the simulation and sync the viewer
        dm_control.mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)
