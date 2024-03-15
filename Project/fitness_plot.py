import pandas as pd
import matplotlib.pyplot as plt

filename = 'fitness_scores.csv'

# Read the CSV file into a DataFrame
df = pd.read_csv(filename)

# Plotting the data
plt.plot(df['Best Fitness'], label='Best Fitness')
# plt.plot(df['Average Fitness'], label='Average Fitness')
plt.title("Fitness Scores from CSV")
plt.xlabel("Measurement Number")
plt.ylabel("Fitness Score")
plt.legend()
plt.savefig('fitness_scores_plot.png')  
plt.show()
