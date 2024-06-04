import numpy as np
import matplotlib.pyplot as plt


def generate_multisine_positions(dt, time_max):
    time = np.arange(0, time_max, dt)
    velocities = (np.sin(2 * np.pi * 0.1 * time) +
                  np.sin(2 * np.pi * 0.25 * time) +
                  np.sin(2 * np.pi * 0.5 * time) +
                  np.sin(2 * np.pi * 0.5 * time) +
                  np.sin(2 * np.pi * 0.75 * time) +
                  np.sin(2 * np.pi * 1 * time))
    return time, velocities


def generate_multisine_velocities(dt, time_max):
    time = np.arange(0, time_max, dt)
    # Calculate the derivative of the multisine function
    # shift with -pi/2 to start from velocity close to 0 and move in positive direction
    velocities = (0.1 * 2 * np.pi * np.cos(2 * np.pi * 0.1 * time - np.pi/2) +
                  0.25 * 2 * np.pi * np.cos(2 * np.pi * 0.25 * time - np.pi/2) +
                  0.5 * 2 * np.pi * np.cos(2 * np.pi * 0.5 * time - np.pi/2) +
                  0.5 * 2 * np.pi * np.cos(2 * np.pi * 0.5 * time - np.pi/2) +
                  0.75 * 2 * np.pi * np.cos(2 * np.pi * 0.75 * time - np.pi/2) +
                  1 * 2 * np.pi * np.cos(2 * np.pi * 1 * time - np.pi/2))
    return time, velocities


def plot_multisine_positions(time, velocities):
    plt.figure(figsize=(10, 6))
    plt.plot(time, velocities, label='Multisine Waveform - Positions', color='blue')

    # Plot constituent sine waves
    plt.plot(time, np.sin(2 * np.pi * 0.1 * time), label='0.1 Hz', linestyle='--', color='orange')
    plt.plot(time, np.sin(2 * np.pi * 0.25 * time), label='0.25 Hz', linestyle='--', color='green')
    plt.plot(time, np.sin(2 * np.pi * 0.5 * time), label='0.5 Hz', linestyle='--', color='red')
    plt.plot(time, np.sin(2 * np.pi * 0.75 * time), label='0.75 Hz', linestyle='--', color='purple')
    plt.plot(time, np.sin(2 * np.pi * 1 * time), label='1 Hz', linestyle='--', color='brown')

    plt.title('Multisine Waveform and Constituent Sine Waves')
    plt.xlabel('Time')
    plt.ylabel('Amplitude')
    plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.grid(True)
    plt.show()


def plot_multisine_velocities(time, velocities):
    plt.figure(figsize=(10, 6))
    plt.plot(time, velocities, label='Multisine Waveform - Velocities', color='blue')

    # Plot constituent cosine waves
    plt.plot(time, 0.1 * 2 * np.pi * np.cos(2 * np.pi * 0.1 * time - np.pi/2), label='0.1 Hz', linestyle='--', color='orange')
    plt.plot(time, 0.25 * 2 * np.pi * np.cos(2 * np.pi * 0.25 * time - np.pi/2), label='0.25 Hz', linestyle='--', color='green')
    plt.plot(time, 0.5 * 2 * np.pi * np.cos(2 * np.pi * 0.5 * time - np.pi/2), label='0.5 Hz', linestyle='--', color='red')
    plt.plot(time, 0.5 * 2 * np.pi * np.cos(2 * np.pi * 0.5 * time - np.pi/2), label='0.5 Hz', linestyle='--', color='purple')
    plt.plot(time, 0.75 * 2 * np.pi * np.cos(2 * np.pi * 0.75 * time - np.pi/2), label='0.75 Hz', linestyle='--', color='brown')
    plt.plot(time, 1 * 2 * np.pi * np.cos(2 * np.pi * 1 * time - np.pi/2), label='1 Hz', linestyle='--', color='gray')

    plt.title('Velocity from Differentiated Multisine and Constituent Cosine Waves')
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.grid(True)
    plt.show()


def write_velocities_header(velocities):
    with open('velocities.h', 'w') as f:
        f.write('#ifndef VELOCITIES_H\n')
        f.write('#define VELOCITIES_H\n\n')
        f.write('#include <array>\n\n')
        f.write('const std::array<double, {}> velocities = {{\n'.format(len(velocities)))
        for vel in velocities:
            f.write('\t{},\n'.format(vel))
        f.write('};\n\n')
        f.write('#endif // VELOCITIES_H\n')


def main():
    dt = 0.01  # Time step (adjust as needed)
    time_max = 10.0  # Maximum time for the motion

    # Generate multisine velocity waveform
    time, positions = generate_multisine_positions(dt, time_max)
    time, velocities = generate_multisine_velocities(dt, time_max)

    # Plot multisine waveform along with constituent sine waves
    # plot_multisine_positions(time, positions)
    # plot_multisine_velocities(time, velocities)

    # Write the velocities to a header file
    write_velocities_header(velocities)


if __name__ == "__main__":
    main()
