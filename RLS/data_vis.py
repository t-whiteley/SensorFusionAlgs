import pandas as pd
import matplotlib.pyplot as plt

# Read CSV into a DataFrame
df = pd.read_csv('output.csv')

fig, ax = plt.subplots()


# ax.plot(df['Time'], df['s_noisy'], label='s_noisy')
# ax.plot(df['Time'], df['s_filt'], label='s_filt')
# ax.plot(df['Time'], df['s_real'], label='s_real')
ax.plot(df['Time'], df['a_real'], label='a_real')
ax.plot(df['Time'], df['a_filt'], label='a_filt')
ax.legend()

plt.xlabel('Time')
plt.tight_layout()
plt.show()