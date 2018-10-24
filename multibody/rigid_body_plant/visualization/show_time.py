from drake.visualization.director_time import init_visualizer

# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    time_viz = init_visualizer()
