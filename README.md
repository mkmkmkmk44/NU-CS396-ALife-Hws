# NU-CS396-ALife-Hws

### Homework 1
Object: Cycle

Running Instructions: 
mjpython simulate.py

### Homework 2
Object: Walking Creature

Running Instructions: 
mjpython creature_simulate.py

### Homework 3
Object: Quadraped

Running Instructions: 
mjpython create_random_creature.py

#### Description:
1. Class to generate a quadruped model using Mujoco
2. The creature includes a torso and four legs with hip and ankle joints
3. The model creates a simple ant with random ranges of motion with a torso and four legs
4. Then generates randomly sized creatures using the structure of the model with random ranges of motion
5. The model randomly adjusts the size of the torso, the lengths of the legs, the thickness of the legs, and the strength of the actuators within a certain range
6. All classes and functions take values as well (to test the function with different values)
7. No values are provided to create random creatures

### Homework 4
Object: Snake

Running Instructions: 
mjpython mutation_code.py

#### Description:
A Sliding Creature with n limbs joined with hinge joints  
Original Creature has uniform lengths accross all limbs and uniform actuator values

Mutations
1. Addition of Extra limbs
2. Having fewer limbs 
3. Bigger Sized Limbs (radius)
4. Joint Range Mutations
    i.   Front and Back joints are fixed and moves from the joints in middle body
    ii.  Middle joints are fixed and motion is performed using first and last joint
    iii. Alternate joints in the body are fixed
5. Joint Control/Actuator Mutations
    i.   All the joints have uniform Control
    ii.  All joints have random Control
    iii. Front and Back joints have greater Control
    iv.  Joints in the middle have greater Control
    v.   Alternate Control values
    vi.  Joints have incremental control - Lower Joints have more control
