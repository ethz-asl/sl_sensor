#!/usr/bin/env python

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def l2_distance(residual):
  return np.sqrt(np.sum([i**2 for i in residual]))

def compute_rmse(residuals):
  
  squared_errors = []
  
  for residual in residuals:
    squared_errors.append(l2_distance(residual) ** 2)

  return np.sqrt(np.mean(squared_errors))

def extract_residuals_from_files(filename):

  residuals = []

  with open(filename,'r') as f:
      for line in f:
        values_string = line.split(' ')
        values_float = [float(i) for i in values_string]
        if(len(values_float) == 2):
          residuals.append(values_float)
  
  return residuals

def split_residuals_by_axis(residuals):

  x = []
  y = []

  for residual in residuals:
    x.append(residual[0])
    y.append(residual[1])

  return x, y


def plot_scatter_plot(ax, subplot_directory):
  
  print(subplot_directory)

  residuals = extract_residuals_from_files(subplot_directory)
  
  rmse = compute_rmse(residuals)

  # We do not continue if we fail to get residual values
  if not residuals:
    print("Could not extract residuals from file " + subplot_directory)
    return

  label = Path(subplot_directory).stem + " (RMSE = " + str(round(rmse,3)) + ")"

  (x,y) = split_residuals_by_axis(residuals)

  ax.scatter(x, y, s=1, marker="x", label=label)

  return max([abs(i) for i in x]), max([abs(i) for i in y])


if __name__ == '__main__':

  # Extract Parameters
  parser = argparse.ArgumentParser(description= 'Visualises reprojection errors on scatter diagrams. You can plot multiple datasets on a single plot, and plot multiple plots at the same time')
  parser.add_argument('directories', nargs='+', help='For each plot, specify the title (Spaces in underscores), then the .txt file directories of the datasets, separated by commas. Individual plots are separated by spaces')
  args = parser.parse_args()
  
  all_directories = args.directories

  # Initialise Plot
  fig = plt.figure()  

  number_subplots = len(all_directories)

  for i in range(1,number_subplots+1,1):
    subplot_directories = all_directories[i-1].split(',')

    # Process title
    subplot_title = subplot_directories.pop(0) # First entry is the title of the subplot
    subplot_title = subplot_title.replace('_', ' ') # Replace underscore in title with spaces

    ax = fig.add_subplot(1, number_subplots, i, aspect='equal')

    # Draw circle with 0.5 pixel radius
    circle = plt.Circle((0, 0), 0.5, color='r', fill= False)
    ax.add_patch(circle)

    max_y = 0
    max_x = 0

    for subplot_directory in subplot_directories:
      local_max_x, local_max_y = plot_scatter_plot(ax, subplot_directory)
      max_y = max([max_y, local_max_y])
      max_x = max([max_x, local_max_x])

    sq_size = max([max_x, max_y]) * 1.1

    ax.title.set_text(subplot_title)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1),
          fancybox=True, shadow=True)
    ax.set_xlim(-1.0 * sq_size, sq_size)
    ax.set_ylim(-1.0 * sq_size, sq_size)

  plt.show()
