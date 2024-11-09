'''
    vis.py
    This file contains the implementation of the visualization of the robot path planning problem.
'''
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import sys

def plot_maze(file_path):
    try:
        # Initialize an empty list to store the valid rows
        valid_rows = []

        # Read the file line by line and filter out lines with fewer than 50 elements
        with open(file_path, "r") as file:
            # Skip the first 4 lines
            for _ in range(4):
                next(file)
                
            # Process the rest of the file
            for line in file:
                elements = line.strip().split()
                if len(elements) >= 50:  
                    valid_rows.append(elements[:50])  

        # Convert the valid rows to a NumPy array
        maze = np.array(valid_rows, dtype=int)

        # Initialize the plot
        fig, ax = plt.subplots(figsize=(12, 7))
        nrows, ncols = maze.shape

        # Define colors for each tile type
        colors = {0: "white", 1: "black", 2: "red", 3: "white", 4: "yellow", 5: "green"}

        # Plot each tile
        for row in range(nrows):
            for col in range(ncols):
                tile_color = colors.get(
                    maze[row, col], "white"
                )  # Default to white if unknown
                rect = Rectangle(
                    (col, row), 1, 1, edgecolor="black", facecolor=tile_color
                )
                ax.add_patch(rect)

        plt.xlim(0, ncols)
        plt.ylim(0, nrows)
        plt.gca().invert_yaxis()  # Invert y-axis to match array layout
        plt.axis("off")  # Turn off the axis
        plt.show()

    except Exception as e:
        print(f"Failed to read the file: {e}")


def main():
    file_path = sys.argv[1]
    plot_maze(file_path)


if __name__ == "__main__":
    main()
