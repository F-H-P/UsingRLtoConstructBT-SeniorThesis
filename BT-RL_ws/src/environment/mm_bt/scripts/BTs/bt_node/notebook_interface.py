"""
A simple simulation environment for running behavior trees
"""

#Imports that define the environment
from bt_node.py_trees_interface import PyTree
import bt_node.behaviors as behaviors
import bt_node.conveyor_kitting as sm
import bt_node.fitness_function as fitness_function

class Environment():
    """ Class defining the environment in which the individual operates """
    def __init__(self, seed=None, verbose=False, fitness_coeff=None):
        self.seed = seed
        self.verbose = verbose
        self.fitness_coeff = fitness_coeff
        # self.world_interface = None
        self.world_interface = sm.Simulation(seed=self.seed)
        self.pytree = None
        self.tree = None

    def get_fitness(self, individual, max_ticks=200, show_world=False, seed=None):
        """ Run the simulation and return the fitness """
        if seed is not None:
            self.seed = seed
        self.world_interface = sm.Simulation(seed=self.seed)
        self.pytree = PyTree(individual[:], behaviors=behaviors, \
            world_interface=self.world_interface, verbose=self.verbose)

        # run the Behavior Tree
        ticks, _ = self.pytree.run_bt(max_ticks=max_ticks, show_world=show_world)

        return fitness_function.compute_fitness(self.world_interface, self.pytree, ticks, self.fitness_coeff)

    def step(self, individual, show_world=False):
        """ Run the simulation and return the fitness """
        if self.world_interface is None:
            self.world_interface = sm.Simulation(seed=self.seed)
        if self.pytree is None:
            self.pytree = PyTree(individual[:], behaviors=behaviors, \
                world_interface=self.world_interface, verbose=self.verbose)

        self.pytree.step_bt(show_world=show_world)

    def plot_individual(self, path, plot_name, individual):
        # pylint: disable=no-self-use
        """ Saves a graphical representation of the individual """
        self.pytree = PyTree(individual[:], behaviors=behaviors)
        self.pytree.save_fig(path, name=plot_name)

    def create_tree(self, individual):
        self.pytree = PyTree(individual[:], behaviors=behaviors, \
            world_interface=self.world_interface, verbose=self.verbose)
        return self.pytree