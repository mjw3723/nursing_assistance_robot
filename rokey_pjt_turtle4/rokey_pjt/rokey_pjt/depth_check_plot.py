# import matplotlib.pyplot as plt

# # Distance (m) and error (cm) data
# distance = [0.7, 0.8, 0.9, 1.0, 1.1, 1.5, 2.0, 3.0]
# error = [1, 2, 3, 4, 4.5, 7, 13, 30]

# # Plot the graph
# plt.figure(figsize=(8, 5))
# plt.plot(distance, error, marker='o', linestyle='-', color='b', label='Error (cm)')

# # Labels and title
# plt.xlabel('Distance (m)')
# plt.ylabel('Error (cm)')
# plt.title('Error by Distance')
# plt.grid(True)
# plt.legend()
# plt.tight_layout()

# # Show the plot
# plt.show()


import matplotlib.pyplot as plt

# Distance (m) and error (cm) data
distance = [0.7, 0.8, 0.9, 1.0, 1.1, 1.5, 2.0, 3.0]
error = [1, 2, 3, 4, 4.5, 7, 13, 30]

# Plot the graph
plt.figure(figsize=(8, 5))
plt.plot(distance, error, marker='o', linestyle='-', color='b', label='Error (cm)')

# Labels and title
plt.xlabel('Distance (m)')
plt.ylabel('Error (cm)')
plt.title('Error by Distance')
plt.grid(True)
plt.legend()
plt.tight_layout()

# Show the plot
plt.show()
