# NU-CS396-ALife-Project

Object: Moving Creature with Cylindrical Limbs connected sequentially
Goal for each creature: To move as much distance as we can in the given time, and not fly off
In this project, we create a population of a certain size, and evolve it through generation and see creatures better at achieving the Goal through the generations.

Running Instructions: 
mjpython evolve.py

#### File Descriptions
1. evolve.py -- contains the evolution process
In the file we use these values to initialize population size and generation size
main(num_generations=num_generations, population_size=population_size, save_xml=False, show_simulation=False)

2. mutation_code -- contains code to generate the creatures and mutate them

3. fitness_plot.py -- can be used to plot fitness values

### How are the creature evolving?
I initialized a population of 100 random creatures
Calculated fitness for each creature in the generation
Sorted them by fitness
Retained top 5% in the newer population
Randomly chose multiple sets of parents from top 50% of the current generation
Performed a crossover between the 2 parents
Then performed a random number of mutations on the offspring
Added them to the population
Replaced the newer population with older population
This is done iteratively for the given number of generations

### Parent Selection
A subset of top50% of most fit people from current generation is created
2 sets of parents are randomly selected for each new offspring

### Crossover function
First we check number of limbs of parent 1 and parent 2
Then the child has the average number of limbs of the two parents
We randomly select one of the parents to perform the crossover -- i.e. off the two parents, we choose the limb lengths, joint angles, and motor values of the randomly selected parent
Then adjust the number of limbs of the child to be the average of the number of limbs of the two parents
Then return the child

### Mutation Information
1) Addition of Extra limbs
2) Having fewer limbs 
3) Bigger Sized Limbs (radius)
4) Joint Range Mutations
        Front and Back joints are fixed and moves from the joints in middle body
        Middle joints are fixed and motion is performed using first and last joint
        Alternate joints in the body are fixed
5) Joint Control/Actuator Mutations
        All the joints have uniform Control
        All joints have random Control
        Front and Back joints have greater Control
        Joints in the middle have greater Control
        Alternate Control values
        Joints have incremental control - Lower Joints have more control
