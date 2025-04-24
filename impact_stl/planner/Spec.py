import copy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from utilities.zonotopes import zonotope

class Robot():
    # Robot class
    def __init__(self,name,x0,dx0,xf,dxf,nbz=10,
                 dq_lb=np.array([-2,-2]),dq_ub=np.array([2,2])):
        self.name = name
        self.mass = 16.8
        self.x0 = x0
        self.dx0 = dx0
        self.xf = xf
        self.dxf = dxf

        self.nbz = nbz
        self.dq_lb = dq_lb
        self.dq_ub = dq_ub

        # the kind of bezier curve: ['none','pre','post']-impact
        self.ids = self.nbz*['none']
        # other names contains the name of the robot or object
        # that the object or robot is impacting with, only if 
        # self.ids[i] == 'pre' or 'post'!
        self.other_names = self.nbz*['none']


class Object():
    def __init__(self,name,x0,dx0,xf,dxf,nbz=10,t0=0,tf=100,
                 dq_lb=np.array([-2,-2]),dq_ub=np.array([2,2])):
        self.name = name
        self.mass = 16.8
        self.x0 = x0
        self.dx0 = dx0
        self.xf = xf
        self.dxf = dxf

        self.Xfd = zonotope(x=np.append(xf,dxf),
                            Gdiag=np.array([0.5,0.5,0.05,0.05]))

        # create the initial zonotope set, without uncertainty on velocity
        self.X0d = zonotope(x=np.append(x0,dx0),
                            G=np.array([[0,0,0,0],
                                        [0,0,0,0],
                                        [0,0,0,0],
                                        [0,0,0,0]]))
        # array to keep track of Xfs for step
        self.Xfs = []

        self.nbz = nbz
        self.dt = (tf-t0)/self.nbz
        self.hvar = [np.array([[t0+i*self.dt,t0+(i+1)*self.dt]]) for i in range(self.nbz)]
        self.dq_lb = dq_lb
        self.dq_ub = dq_ub

        self.ids = self.nbz*['none']
        self.other_names = self.nbz*['none']

    def evaluate_t(self,t):
        # return the idx of the hvar that contains t
        # also return the phase between [0,1]
        for i,h in enumerate(self.hvar):
            if t >= h[0] and t < h[1]:
                return i, (t-h[0])/(h[1]-h[0])
            
        return i,1


class Area():
    def __init__(self,x_min,x_max):
        self.x_min = x_min
        self.x_max = x_max
        # and convert x_min and x_max to a polytope, Hx <= b
        self.H = np.array([[1,0],
                           [0,1],
                           [-1,0],
                           [0,-1]])
        self.b = np.array([x_max[0],x_max[1],-x_min[0],-x_min[1]])
        self.nfaces = 4

    def plot(self,ax,color='r'):
        # draw a red rectangle
        rect = Rectangle((self.x_min[0],self.x_min[1]),self.x_max[0]-self.x_min[0],self.x_max[1]-self.x_min[1],
                         linewidth=1,edgecolor=color,facecolor=color)
        ax.add_patch(rect)


class Pred():
    def __init__(self,type,I=[0,0],preds=[],io="in"):
        self.type = type
        self.I = I
        self.preds = preds
        self.io = io
        self.rho = None

        self.z_time = None
    
    def get_string(self):
        return f"{self.type}({self.I})"

class Spec():
    def __init__(self,t0,tf):
        self.preds = []
        self.names = []
        self.pred = None

        self.t0 = t0
        self.tf = tf

    def add_pred(self,pred,name):
        # make deep copy of pred
        self.names.append(copy.deepcopy(name))
        self.preds.append(copy.deepcopy(pred))
        print(f"Added pred {pred.get_string()} with name {name}")
        print(f"names: {self.names}")

def spatial_specifications(world,specification):
    if specification == "catch_throw":
        tf = 80
        world.spec = Spec(t0=0,tf=tf)

        v_max = 0.3
        robot1 = Robot(name="snap",
                       x0=np.array([1,1]),
                       dx0=np.array([0,0]),
                       xf=np.array([1,1]),
                       dxf=np.array([0,0]),nbz=4,
                       dq_lb=np.array([-v_max,-v_max]),
                       dq_ub=np.array([v_max,v_max]))
        robot2 = Robot(name="crackle",
                       x0=np.array([1,9]),
                       dx0=np.array([0,0]),
                       xf=np.array([1,9]),
                       dxf=np.array([0,0]),nbz=3,
                       dq_lb=np.array([-v_max,-v_max]),
                       dq_ub=np.array([v_max,v_max]))
        object1 = Object(name="pop",
                         x0=np.array([1,2]),
                         dx0=np.array([0,0]),
                         xf=np.array([1,8]),
                         dxf=np.array([0,0]),nbz=6,
                         dq_lb=np.array([-0.35,-0.35]),
                         dq_ub=np.array([0.35,0.35]))
        
        area1 = Area(x_min=np.array([0.25,7.25]),x_max=np.array([1.75,8.75]))
        mu1 = Pred(type="MU",preds=area1,io="in")
        phi1 = Pred(type="F",I=[0,tf],preds=[mu1])

        obstacle1 = Area(x_min=np.array([-1,0]),x_max=np.array([0,10]))
        mu2 = Pred(type="NEG",preds=obstacle1,io="out")
        phi2 = Pred(type="G",I=[0,tf],preds=[mu2])
        obstacle2 = Area(x_min=np.array([2,0]),x_max=np.array([3,10]))
        mu3 = Pred(type="NEG",preds=obstacle2,io="out")
        phi3 = Pred(type="G",I=[0,tf],preds=[mu3])

        phi_snap = Pred(type="AND",preds=[copy.deepcopy(phi2),copy.deepcopy(phi3)])
        phi_crackle = Pred(type="AND",preds=[copy.deepcopy(phi2),copy.deepcopy(phi3)])
        phi_pop = Pred(type="AND",preds=[phi1,phi2,phi3])

        world.spec.add_pred(phi_snap,name='snap')
        world.spec.add_pred(phi_crackle,name='crackle')
        world.spec.add_pred(phi_pop,name='pop')


        world.dim = 2
        world.robots = [robot1,robot2]
        world.objects = [object1]

        # World bounding box
        world.x_lb = np.array([-1,0])
        world.x_ub = np.array([3,10])

        # Area's of interest
        world.areas = [area1,obstacle1,obstacle2]
        world.obstacles = []

    elif specification == "catch_throw_exp":
        tf = 30
        world.spec = Spec(t0=0,tf=tf)

        v_max = 0.2
        robot1 = Robot(name="crackle",
                        x0=np.array([0.4,0]),
                        dx0=np.array([0,0]),
                        xf=np.array([1.0,0]),
                        dxf=np.array([0,0]),nbz=3,
                        dq_lb=np.array([-v_max,-v_max]),
                        dq_ub=np.array([v_max,v_max]))
        robot2 = Robot(name="pop",
                        x0=np.array([3.5,0]),
                        dx0=np.array([0,0]),
                        xf=np.array([3.5,0]),
                        dxf=np.array([0,0]),nbz=3,
                        dq_lb=np.array([-v_max,-v_max]),
                        dq_ub=np.array([v_max,v_max]))
        object1 = Object(name="snap",
                        x0=np.array([1.2,0]),
                        dx0=np.array([0,0]),
                        xf=np.array([2.5,0]),
                        dxf=np.array([0,0]),
                        nbz=4)
        
        area1 = Area(x_min=np.array([2.1,-0.4]),x_max=np.array([2.9,0.4]))
        mu1 = Pred(type="MU",preds=area1,io="in")
        phi1 = Pred(type="F",I=[0,tf],preds=[mu1])

        obstacle1 = Area(x_min=np.array([0,1.0]),x_max=np.array([4,1.5]))
        mu2 = Pred(type="NEG",preds=obstacle1,io="out")
        phi2 = Pred(type="G",I=[0,tf],preds=[mu2])
        obstacle2 = Area(x_min=np.array([0,-1.0]),x_max=np.array([4,-1.5]))
        mu3 = Pred(type="NEG",preds=obstacle2,io="out")
        phi3 = Pred(type="G",I=[0,tf],preds=[mu3])

        phi_snap = Pred(type="AND",preds=[copy.deepcopy(phi2),copy.deepcopy(phi3)])
        phi_crackle = Pred(type="AND",preds=[copy.deepcopy(phi2),copy.deepcopy(phi3)])
        phi_pop = Pred(type="AND",preds=[phi1])#,phi2,phi3])

        # world.spec.add_pred(phi_snap,name='pop')
        # world.spec.add_pred(phi_crackle,name='crackle')
        world.spec.add_pred(phi_pop,name='snap')

        world.dim = 2
        world.robots = [robot1,robot2]
        world.objects = [object1]

        # World bounding box
        world.x_lb = np.array([0,-1.5])
        world.x_ub = np.array([4,1.5])

        # Area's of interest
        world.areas = [area1,obstacle1,obstacle2]
        world.obstacles = []

    elif specification == "obstacle_avoidance":
        # STL
        area1 = Area(x_min=np.array([2,2]),x_max=np.array([3,3]))
        mu1 = Pred(type="MU",preds=area1,io="in")
        phi1 = Pred(type="G",I=[5,10],preds=[mu1])
        area2 = Area(x_min=np.array([8,8]),x_max=np.array([9,9]))
        mu2 = Pred(type="MU",preds=area2,io="in")
        phi2 = Pred(type="F",I=[10,15],preds=[mu2])

        obs1 = Area(x_min=np.array([3,2]),x_max=np.array([7,4]))
        # mu3 = Pred(type="MU",preds=obs1,io="out")
        phi31 = Pred(type="NEG",preds=obs1) 
        phi3 = Pred(type="G",I=[0,150],preds=[phi31])

        world.spec = Spec(t0=0,tf=150)
        world.spec.add_pred(Pred(type="AND",preds=[phi3]),
                           name='crackle')
        world.spec.add_pred(copy.deepcopy(phi3),
                           name='snap')
        world.spec.add_pred(copy.deepcopy(phi3),
                           name='pop')
        # world.spec.add_pred(phi1)

        robot1 = Robot(name="snap",
                       x0=np.array([9,5]),
                       dx0=np.array([0,0]),
                       xf=np.array([9,1]),
                       dxf=np.array([0,0]),nbz=7)
        
        robot2 = Robot(name="crackle",
                       x0=np.array([1,7]),
                       dx0=np.array([0,0]),
                       xf=np.array([1,1]),
                       dxf=np.array([0,0]),nbz=6)
        
        object1 = Object(name="pop",
                         x0=np.array([5.5,5]),
                         dx0=np.array([0,0]),
                         xf=np.array([5,1]),
                         dxf=np.array([0,0]),nbz=5)

        world.dim = 2
        world.robots = [robot1,robot2]
        world.objects = [object1]

        # World bounding box
        world.x_lb = np.array([0,0])
        world.x_ub = np.array([10,10])

        # Obstacles
        world.obstacles.append(obs1)

        # Area's of interest
        world.areas = [area1,area2]

    elif specification == "pong_stl":
        # STL
        tf = 100
        # bottom left
        area1 = Area(x_min=np.array([1,-1]),x_max=np.array([1.5,-0.5]))
        mu1 = Pred(type="MU",preds=area1,io="in")
        phi1 = Pred(type="F",I=[0,tf],preds=[mu1])
        # bottom right
        area2 = Area(x_min=np.array([2.25,-1]),x_max=np.array([2.75,-0.5]))
        mu2 = Pred(type="MU",preds=area2,io="in")
        phi2 = Pred(type="F",I=[0,tf],preds=[mu2])
        # top left
        area3 = Area(x_min=np.array([1.0,0.0]),x_max=np.array([1.5,0.5]))
        mu3 = Pred(type="MU",preds=area3,io="in")
        phi3 = Pred(type="F",I=[0,tf],preds=[mu3])
        # top right
        area4 = Area(x_min=np.array([2.25,0.5]),x_max=np.array([2.75,1.0]))
        mu4 = Pred(type="MU",preds=area4,io="in")
        phi4 = Pred(type="F",I=[0,tf],preds=[mu4])

        world.spec = Spec(t0=0,tf=100)
        world.spec.add_pred(Pred(type="AND",preds=[phi1,phi2,phi3,phi4]),
                           name='snap')

        robot1 = Robot(name="pop",
                       x0=np.array([0.5,-1]),
                       dx0=np.array([0,0]),
                       xf=np.array([0.5,1]),
                       dxf=np.array([0,0]),nbz=6)
        
        robot2 = Robot(name="crackle",
                       x0=np.array([3.5,-1]),
                       dx0=np.array([0,0]),
                       xf=np.array([3.5,1]),
                       dxf=np.array([0,0]),nbz=6)
        
        object1 = Object(name="snap",
                         x0=np.array([1.25,-0.75]),
                         dx0=np.array([0,0]),
                         xf=None,
                         dxf=np.array([0,0]),nbz=5)

        world.dim = 2
        world.robots = [robot1,robot2]
        world.objects = [object1]

        # World bounding box
        world.x_lb = np.array([0.25,-1.25])
        world.x_ub = np.array([3.75,1.25])

        # Area's of interest
        world.areas = [area1,area2,area3,area4]
        world.obstacles = []

def impact_specifications(world,specification):
    if specification == "catch_throw":
        tf = 80
        world.spec = Spec(t0=0,tf=tf)

        v_max = 0.3
        robot1 = Robot(name="snap",
                        x0=np.array([1,1]),
                        dx0=np.array([0,0]),
                        xf=np.array([1,1]),
                        dxf=np.array([0,0]),nbz=4,
                        dq_lb=np.array([-v_max,-v_max]),
                        dq_ub=np.array([v_max,v_max]))
        robot2 = Robot(name="crackle",
                        x0=np.array([1,9]),
                        dx0=np.array([0,0]),
                        xf=np.array([1,9]),
                        dxf=np.array([0,0]),nbz=4,
                        dq_lb=np.array([-v_max,-v_max]),
                        dq_ub=np.array([v_max,v_max]))
        object1 = Object(name="pop",
                        x0=np.array([1,2]),
                        dx0=np.array([0,0]),
                        xf=np.array([1,8]),
                        dxf=np.array([0,0]),
                        t0=world.spec.t0,tf=world.spec.tf,
                        nbz=7)
        object1.Xfd = zonotope(x=np.append(object1.xf,object1.dxf),
                               Gdiag=np.array([0.5,0.5,0.05,0.05]))
        
        area1 = Area(x_min=np.array([0.25,7.25]),x_max=np.array([1.75,8.75]))
        mu1 = Pred(type="MU",preds=area1,io="in")
        phi1 = Pred(type="F",I=[0,tf],preds=[mu1])

        obstacle1 = Area(x_min=np.array([-1,0]),x_max=np.array([0,10]))
        mu2 = Pred(type="NEG",preds=obstacle1,io="out")
        phi2 = Pred(type="G",I=[0,tf],preds=[mu2])
        obstacle2 = Area(x_min=np.array([2,0]),x_max=np.array([3,10]))
        mu3 = Pred(type="NEG",preds=obstacle2,io="out")
        phi3 = Pred(type="G",I=[0,tf],preds=[mu3])

        phi_snap = Pred(type="AND",preds=[copy.deepcopy(phi2),copy.deepcopy(phi3)])
        phi_crackle = Pred(type="AND",preds=[copy.deepcopy(phi2),copy.deepcopy(phi3)])
        phi_pop = Pred(type="AND",preds=[phi1,phi2,phi3])

        world.spec.add_pred(phi_snap,name='snap')
        world.spec.add_pred(phi_crackle,name='crackle')
        world.spec.add_pred(phi_pop,name='pop')

        world.dim = 2
        world.robots = [robot1,robot2]
        world.objects = [object1]

        # World bounding box
        world.x_lb = np.array([-1,0])
        world.x_ub = np.array([3,10])

        # Area's of interest
        world.areas = [area1,obstacle1,obstacle2]
        world.obstacles = []

    elif specification == "catch_throw_exp":
        tf = 30
        world.spec = Spec(t0=0,tf=tf)

        v_max = 0.3
        robot1 = Robot(name="crackle",
                        x0=np.array([0.4,0]),
                        dx0=np.array([0,0]),
                        xf=np.array([0.4,0]),
                        dxf=np.array([0,0]),nbz=4,
                        dq_lb=np.array([-v_max,-v_max]),
                        dq_ub=np.array([v_max,v_max]))
        robot2 = Robot(name="pop",
                        x0=np.array([3.5,0]),
                        dx0=np.array([0,0]),
                        xf=np.array([3.5,0]),
                        dxf=np.array([0,0]),nbz=3,
                        dq_lb=np.array([-v_max,-v_max]),
                        dq_ub=np.array([v_max,v_max]))
        object1 = Object(name="snap",
                        x0=np.array([1.2,0]),
                        dx0=np.array([0,0]),
                        xf=np.array([2.5,0]),
                        dxf=np.array([0,0]),
                        t0=world.spec.t0,tf=world.spec.tf,
                        nbz=6)
        object1.Xfd = zonotope(x=np.append(object1.xf,object1.dxf),
                               Gdiag=np.array([0.4,0.4,0.1,0.1]))
        
        area1 = Area(x_min=np.array([2.1,-0.4]),x_max=np.array([2.9,0.4]))
        mu1 = Pred(type="MU",preds=area1,io="in")
        phi1 = Pred(type="F",I=[0,tf],preds=[mu1])

        obstacle1 = Area(x_min=np.array([0,1.0]),x_max=np.array([4,1.5]))
        mu2 = Pred(type="NEG",preds=obstacle1,io="out")
        phi2 = Pred(type="G",I=[0,tf],preds=[mu2])
        obstacle2 = Area(x_min=np.array([0,-1.0]),x_max=np.array([4,-1.5]))
        mu3 = Pred(type="NEG",preds=obstacle2,io="out")
        phi3 = Pred(type="G",I=[0,tf],preds=[mu3])

        phi_snap = Pred(type="AND",preds=[copy.deepcopy(phi2),copy.deepcopy(phi3)])
        phi_crackle = Pred(type="AND",preds=[copy.deepcopy(phi2),copy.deepcopy(phi3)])
        phi_pop = Pred(type="AND",preds=[phi1])#,phi2,phi3])

        # world.spec.add_pred(phi_snap,name='pop')
        # world.spec.add_pred(phi_crackle,name='crackle')
        # world.spec.add_pred(phi_pop,name='snap')

        world.dim = 2
        world.robots = [robot1,robot2]
        world.objects = [object1]

        # World bounding box
        world.x_lb = np.array([0,-1.5])
        world.x_ub = np.array([4,1.5])

        # Area's of interest
        world.areas = [area1,obstacle1,obstacle2]
        world.obstacles = []

    elif specification == "obstacle_avoidance":
        # STL
        area1 = Area(x_min=np.array([2,2]),x_max=np.array([3,3]))
        mu1 = Pred(type="MU",preds=area1,io="in")
        phi1 = Pred(type="G",I=[5,10],preds=[mu1])
        area2 = Area(x_min=np.array([8,8]),x_max=np.array([9,9]))
        mu2 = Pred(type="MU",preds=area2,io="in")
        phi2 = Pred(type="F",I=[10,15],preds=[mu2])

        obs1 = Area(x_min=np.array([2.7,1.7]),x_max=np.array([8,4.3]))
        # mu3 = Pred(type="MU",preds=obs1,io="out")
        phi31 = Pred(type="NEG",preds=obs1) 
        phi3 = Pred(type="G",I=[0,150],preds=[phi31])

        world.spec = Spec(t0=0,tf=150)
        world.spec.add_pred(Pred(type="AND",preds=[phi3]),
                           name='crackle')
        world.spec.add_pred(copy.deepcopy(phi3),
                           name='snap')
        world.spec.add_pred(copy.deepcopy(phi3),
                           name='pop')
        # world.spec.add_pred(phi1)

        robot1 = Robot(name="snap",
                       x0=np.array([9,5]),
                       dx0=np.array([0,0]),
                       xf=np.array([9,1]),
                       dxf=np.array([0,0]),nbz=7)
        
        robot2 = Robot(name="crackle",
                       x0=np.array([1,7]),
                       dx0=np.array([0,0]),
                       xf=np.array([1,1]),
                       dxf=np.array([0,0]),nbz=6)
        
        object1 = Object(name="pop",
                         x0=np.array([5.5,5]),
                         dx0=np.array([0,0]),
                         xf=np.array([5,1]),
                         dxf=np.array([0,0]),nbz=5,
                         t0=world.spec.t0,tf=world.spec.tf)
        object1.Xfd = zonotope(x=np.append(object1.xf,object1.dxf),
                                 Gdiag=np.array([0.5,0.5,0.05,0.05]))

        world.dim = 2
        world.robots = [robot1,robot2]
        world.objects = [object1]

        # World bounding box
        world.x_lb = np.array([0,0])
        world.x_ub = np.array([10,10])

        # Obstacles
        world.obstacles = [obs1]

        # Area's of interest
        world.areas = [area1,area2]

    elif specification == "pong_stl":
        # STL
        tf = 100
        # bottom left
        area1 = Area(x_min=np.array([1,-1]),x_max=np.array([1.5,-0.5]))
        mu1 = Pred(type="MU",preds=area1,io="in")
        phi1 = Pred(type="F",I=[0,tf],preds=[mu1])
        # bottom right
        area2 = Area(x_min=np.array([2.5,-1]),x_max=np.array([3,-0.5]))
        mu2 = Pred(type="MU",preds=area2,io="in")
        phi2 = Pred(type="F",I=[0,tf],preds=[mu2])
        # top left
        area3 = Area(x_min=np.array([1.0,0.0]),x_max=np.array([1.5,0.5]))
        mu3 = Pred(type="MU",preds=area3,io="in")
        phi3 = Pred(type="F",I=[0,tf],preds=[mu3])
        # top right
        area4 = Area(x_min=np.array([2.5,0.5]),x_max=np.array([3.0,1.0]))
        mu4 = Pred(type="MU",preds=area4,io="in")
        phi4 = Pred(type="F",I=[0,tf],preds=[mu4])

        world.spec = Spec(t0=0,tf=100)
        world.spec.add_pred(Pred(type="AND",preds=[phi1,phi2,phi3,phi4]),
                           name='snap')

        v_max = 0.3
        robot1 = Robot(name="pop",
                       x0=np.array([0.5,-1]),
                       dx0=np.array([0,0]),
                       xf=np.array([0.5,1]),
                       dxf=np.array([0,0]),nbz=7,
                       dq_lb=np.array([-v_max,-v_max]),
                       dq_ub=np.array([v_max,v_max]))
        
        robot2 = Robot(name="crackle",
                       x0=np.array([3.5,-1]),
                       dx0=np.array([0,0]),
                       xf=np.array([3.5,1]),
                       dxf=np.array([0,0]),nbz=7,
                       dq_lb=np.array([-v_max,-v_max]),
                       dq_ub=np.array([v_max,v_max]))
        
        object1 = Object(name="snap",
                         x0=np.array([1.25,-0.75]),
                         dx0=np.array([0,0]),
                         xf=np.array([2.75,0.75]),
                         dxf=np.array([0,0]),nbz=8,
                         t0=world.spec.t0,tf=world.spec.tf)
        object1.Xfd = zonotope(x=np.append(object1.xf,object1.dxf),
                               Gdiag=np.array([0.25,0.25,0.01,0.01]))

        world.dim = 2
        world.robots = [robot1,robot2]
        world.objects = [object1]

        # World bounding box
        # world.x_lb = np.array([0.0,-1.5])
        # world.x_ub = np.array([4.0,1.5])
        world.x_lb = np.array([-1.0,-2.5])
        world.x_ub = np.array([5.0,2.5])

        # # Obstacles
        world.obstacles = []
        # world.obstacles = [obs1]

        # Area's of interest
        world.areas = [area1,area2,area3,area4]