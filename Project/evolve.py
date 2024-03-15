from mutation_code import *
import copy
import csv
import os

# Function that generates random creatures
def generate_random_creature():
    # initialize a random number of limbs
    num_limbs = random.randint(3, 9)

    # create an initial creature with random number of limbs
    model = SlidingCreatureMuJoCoModel(num_limbs)
    model.generate_creature()

    # options for randomization to have a variety of creatures in the population

    # random options for mutations for parts of the creature
    mutation_for_motor = ["random", "incremental", "alternate", "middle", "front_back"]
    mutation_for_joint = ["alternate", "middle", "front_back"]
    mutation_for_length = ["random", "front", "back"]

    # creating a list of choices for mutations 
    mutation_choices = ["motor", "joint", "length", "limb_size", "none"]

    num_mutations = random.randint(1, 3)  # random number of mutations
    for _ in range(num_mutations):
        # randomly choosing a mutation choice
        mutation_choice = random.choice(mutation_choices)

        # applying the mutation choice to the creature
        # the mutation choice is randomly chosen from the list of mutation choices
        if mutation_choice == "motor":
            model.change_motor_values(random.choice(mutation_for_motor))
        elif mutation_choice == "joint":
            model.fix_joints(random.choice(mutation_for_joint))
        elif mutation_choice == "length":
            model.change_lengths(random.choice(mutation_for_length))
        elif mutation_choice == "limb_size":
            model.change_size()
        elif mutation_choice == "none":
            pass

    # return the creature
    return model

# Function that initializes the population
def initialize_population(size):
    return [generate_random_creature() for _ in range(size)]

# Function that selects parents from the population
# Selection Process is random
# Two parents are selected from the subset of population passed to the function
def select_parents(population):
    selected_parents = random.choices(population, k=2)
    return selected_parents[0], selected_parents[1]

# Function that performs crossover between two parents
# First we check number of limbs of parent 1 and parent 2
# Then the child has the average number of limbs of the two parents
# We randomly select one of the parents to perform the crossover -- i.e. off the two parents, we choose the limb lengths, joint angles, and motor values of the randomly selected parent
# Then adjust the number of limbs of the child to be the average of the number of limbs of the two parents
# Then return the child
def crossover(parent1, parent2):
    num_limbs_parent1 = parent1.num_limbs
    num_limbs_parent2 = parent2.num_limbs
    num_limbs_child = (num_limbs_parent1 + num_limbs_parent2) // 2
    
    # Randomly select one of the parents to perform the crossover
    random_parent = random.choice([parent1, parent2])
    model = copy.deepcopy(random_parent)

    # Adjust the number of limbs of the child to be the average of the number of limbs of the two parents
    if model.num_limbs == num_limbs_child:
        return model
    if model.num_limbs > num_limbs_child:
        model.delete_limb(model.num_limbs-num_limbs_child)
    else:
        model.add_limb(num_limbs_child-model.num_limbs)
    
    # Return the child
    return model

# Function that performs mutation on the creatures in a similar fashion as above
def mutate(model):
    mutation_for_motor = ["random", "incremental", "alternate", "middle", "front_back"]
    mutation_for_joint = ["alternate", "middle", "front_back"]
    mutation_for_length = ["random", "front", "back", "all"]
    
    # mutation_choices = ["motor", "joint", "length", "add_limb", "delete_limb", "increase_motor", "decrease_motor"]
    mutation_choices = ["motor", "joint", "length", "add_limb", "delete_limb", "decrease_motor", "none"]
    mutation_choice = random.choice(mutation_choices)
    
    if mutation_choice == "motor":
        model.change_motor_values(random.choice(mutation_for_motor))
    elif mutation_choice == "joint":
        model.fix_joints(random.choice(mutation_for_joint))
    elif mutation_choice == "length":
        model.change_lengths(random.choice(mutation_for_length))
    elif mutation_choice == "increase_motor":
        model.change_motor_values("all", 800)
    elif mutation_choice == "decrease_motor":
        model.change_motor_values("all", 300)
    elif mutation_choice == "add_limb":
        if model.num_limbs < 8:
            limb_num = random.randint(1, 9-model.num_limbs)
            model.add_limb(limb_num)
    elif mutation_choice == "delete_limb":
        if model.num_limbs > 4:
            limb_num = random.randint(1, model.num_limbs-3)
            model.delete_limb(limb_num)  
    elif mutation_choice == "none":
        pass
    return model

# Main Function
def main(num_generations, population_size, save_xml = False, show_simulation=False):
    # Initialize the population
    population = initialize_population(population_size)

    # Initialize lists to store the best and average fitness scores for each generation
    best_fitness_scores = []
    average_fitness_scores = []
    
    # Iterate through the generations
    for generation in range(num_generations):
        print("Generation", generation)
        
        # Evaluate the fitness of each creature in the population
        fitness_scores = []
        for creature in population:
            creature_xml = creature.generate_xml()
            fitness_score = simulate_without_viewer(creature_xml)
            fitness_scores.append(fitness_score)
        
        # Sort the population by fitness scores in descending order
        paired_population = list(zip(population, fitness_scores))
        sorted_paired_population = sorted(paired_population, key=lambda x: x[1], reverse=True)
        sorted_population, sorted_fitness_scores = zip(*sorted_paired_population)
        # Convert tuples back to lists if necessary
        sorted_population = list(sorted_population)
        sorted_fitness_scores = list(sorted_fitness_scores)

        # Store the best and average fitness scores for this generation
        best_fitness_scores.append(sorted_fitness_scores[0])
        average_fitness_scores.append(sum(sorted_fitness_scores) / len(sorted_fitness_scores))

        # Print the best fitness score for this generation
        print("Generation", generation, "best fitness:", sorted_fitness_scores[0])

        # Save the best creature's XML to a file, if specified
        if save_xml:
            best_creature = sorted_population[0]
            best_creature_xml_string = best_creature.generate_xml()
            xml_name = f"model_xml/gen_{generation+1}_best_creature.xml"
        
        # Show the simulation of the creatures, if specified
        if show_simulation == 1:
            best_creature = sorted_population[0]
            best_creature_xml_string = best_creature.generate_xml()
            xml_name = f"model_xml/gen_{generation+1}_best_creature.xml"
            write_xml(best_creature_xml_string, xml_name)
            simulate(xml_name)
            # if not save_xml:
            #     os.remove(xml_name)

        # Save the creature's that evolve to be better than all the previous generations
        if(len(best_fitness_scores) >= 2):
            if best_fitness_scores[-1]>best_fitness_scores[-2]:
                best_creature = sorted_population[0]
                best_creature_xml_string = best_creature.generate_xml()
                xml_name = f"model_xml/gen_{generation+1}_best_creature.xml"
                write_xml(best_creature_xml_string, xml_name)
                # simulate(xml_name)
                # if not save_xml:
                #     os.remove(xml_name)

        
        ### Evolutionary Algorithm ###
        # Create a new population by retaining the top 20% of the population
        new_population = sorted_population[:population_size//5].copy()
        # new_population = []

        # Create the rest of the new population through crossover and mutation
        while len(new_population) < population_size:
            # Select two parents from the top 50% of the population
            parent1, parent2 = select_parents(sorted_population[:population_size//2])

            # Perform crossover
            offspring = crossover(parent1, parent2)

            # Perform mutation a random number of times
            num_mutations = random.randint(0, 5) 
            for _ in range(num_mutations): 
                mutate(offspring)

            # Add the offspring to the new population
            new_population.append(offspring)
        
        # Replace the old population with the new population
        population = new_population.copy()

    # Write the best and average fitness scores to a CSV file
    filename = 'fitness_scores.csv'
    generations = list(range(1, num_generations+1))

    # Open the file in write mode
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)

        # Optionally write headers to the CSV file
        writer.writerow(['Generation', 'Best Fitness', 'Average Fitness'])

        # Write the array values
        for gen_num, best, avg in zip(generations, best_fitness_scores, average_fitness_scores):
            writer.writerow([gen_num, best, avg])

if __name__ == "__main__":
    num_generations = 100  
    population_size = 100   
    main(num_generations=num_generations, population_size=population_size, save_xml=False, show_simulation=False)