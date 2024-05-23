import pandas as pd
import matplotlib.pyplot as plt

# Load the data
df = pd.read_csv('output.csv')

# Create a figure and a set of subplots
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))

# Plot acceleration data on the first subplot
ax1.plot(df['Time'], df['a_real'], label='a_real')
ax1.plot(df['Time'], df['a_filt'], label='a_filt')
ax1.set_xlabel('Time')
ax1.set_ylabel('Acceleration')
ax1.legend()
ax1.set_title('Acceleration vs Time')

# Plot speed data on the second subplot
ax2.plot(df['Time'], df['v_real'], label='v_real')
ax2.plot(df['Time'], df['v_filt'], label='v_filt')
ax2.set_xlabel('Time')
ax2.set_ylabel('Speed')
ax2.legend()
ax2.set_title('Speed vs Time')

# Plot displacement data on the third subplot
ax3.plot(df['Time'], df['s_real'], label='s_real')
ax3.plot(df['Time'], df['s_filt'], label='s_filt')
ax3.set_xlabel('Time')
ax3.set_ylabel('Displacement')
ax3.legend()
ax3.set_title('Displacement vs Time')

# Adjust layout to prevent overlap
plt.tight_layout()

# Show the plot
plt.show()
