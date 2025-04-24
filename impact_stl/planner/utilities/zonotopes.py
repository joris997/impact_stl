import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class zonotope():
    def __init__(self, x=None, G=None, Gdiag=None):
        # assert not both G and Gdiag are not given
        assert not (G is None and Gdiag is None)

        self.x = x
        if G is not None:
            self.G = G
            self.Gdiag = np.diag(G)
        else:
            self.Gdiag = Gdiag
            try:
                self.G = np.diag(Gdiag)
            except:
                self.G = None

    def plot(self, ax, color='r',alpha=0.2):
        if self.Gdiag is not None:
            # make a rectangle from the center x and the widths in each 
            # dimension given by the diagonal of G
            # we plot only 2D
            eps = 1e-4
            ax.add_patch(patches.Rectangle((self.x[0]-self.Gdiag[0],self.x[1]-self.Gdiag[1]),
                                            2*self.Gdiag[0]+eps,2*self.Gdiag[1]+eps,
                                            color=color,alpha=alpha))