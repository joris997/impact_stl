import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from utilities.zonotopes import zonotope
from Spec import Spec, Pred
import copy

class World():
    def __init__(self,robustness_type="spatial",specification="catch_throw"):
        self.robustness_type = robustness_type
        self.specification = specification

        if self.robustness_type == "spatial":
            from Spec import spatial_specifications
            spatial_specifications(self,self.specification)
        elif self.robustness_type == "impact":
            from Spec import impact_specifications
            impact_specifications(self,self.specification)        
        else:
            raise ValueError(f"Unknown robustness type: {self.robustness_type}")
        
        # assert that the world now has .spec, .x_lb, .x_ub, .robots, .objects, .areas, .obstacles
        assert hasattr(self,'spec'), "World does not have a specification"
        assert hasattr(self,'x_lb'), "World does not have a lower bound"
        assert hasattr(self,'x_ub'), "World does not have an upper bound"
        assert hasattr(self,'robots'), "World does not have a robot"
        assert hasattr(self,'objects'), "World does not have an object"
        assert hasattr(self,'areas'), "World does not have an area"
        assert hasattr(self,'obstacles'), "World does not have an obstacle"


    def plot(self,ax):
        # draw black square outline of x_lb x_ub, use matplotlib patch
        rect = Rectangle((self.x_lb[0],self.x_lb[1]),self.x_ub[0]-self.x_lb[0],self.x_ub[1]-self.x_lb[1],
                         linewidth=1,edgecolor='k',facecolor='none')
        ax.add_patch(rect)

        for obs in self.obstacles:
            obs.plot(ax,color='r')

        for area in self.areas:
            area.plot(ax,color='b')

        

