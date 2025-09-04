import numpy as np
import matplotlib.pyplot as plt

# --- Set Global Font Size ---
plt.rcParams.update({
    'font.size': 18,         # Controls default text sizes
    'axes.titlesize': 20,    # Title font
    'axes.labelsize': 18,    # Axis labels
    'xtick.labelsize': 16,   # Tick labels
    'ytick.labelsize': 16,
    'legend.fontsize': 17    # Legend font
})

# Error range
x = np.linspace(-10, 10, 1000)

# --- Roll Brake Function ---
def x_brake(x):
    return np.clip(10 / ((np.abs(x - 3) * np.abs(x + 3) * 0.195 + 1) * 5), 0.0, 1.0)

plt.figure(figsize=(9, 5))
plt.plot(x, x_brake(x), color='red', label=r'$x_{\mathrm{brake}}$')
plt.title('Selective Braking Function for Roll Axis')
plt.xlabel('Error (x)')
plt.ylabel('Modulation Factor')
plt.grid(True)
plt.ylim(-0.1, 1.1)
plt.xlim(-10, 10)
plt.axvline(x=3, linestyle='--', color='gray', alpha=0.3)
plt.axvline(x=-3, linestyle='--', color='gray', alpha=0.3)
plt.legend()
plt.tight_layout()
plt.show()

# --- Pitch Brake Function ---
def y_brake(x):
    return np.clip(10 / ((np.abs(x - 5) * np.abs(x + 5) * 0.067 + 1) * 5), 0.0, 1.0)

plt.figure(figsize=(9, 5))
plt.plot(x, y_brake(x), color='green', label=r'$y_{\mathrm{brake}}$')
plt.title('Selective Braking Function for Pitch Axis')
plt.xlabel('Error (x)')
plt.ylabel('Modulation Factor')
plt.grid(True)
plt.ylim(-0.1, 1.1)
plt.xlim(-10, 10)
plt.axvline(x=5, linestyle='--', color='gray', alpha=0.3)
plt.axvline(x=-5, linestyle='--', color='gray', alpha=0.3)
plt.legend()
plt.tight_layout()
plt.show()
