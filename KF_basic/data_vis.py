import pandas as pd
import matplotlib.pyplot as plt

# Read CSV into a DataFrame
df = pd.read_csv('output.csv')

fig, ax = plt.subplots()

# Top plot (subplot 1)
ax.plot(df['Time'], df['Accel_noisy'], label='accel_noisy')
ax.plot(df['Time'], df['Accel_filt'], label='accel_filt')
ax.plot(df['Time'], df['Accel_real'], label='accel_real')
ax.legend()

plt.xlabel('Time')
plt.tight_layout()
plt.show()
