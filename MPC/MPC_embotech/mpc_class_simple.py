import casadi
import forcespro
import forcespro.nlp
import casadi

# set physical constants
#up to date
lol = 1.59 
WD_front = 0.467
l_f = lol*(1-WD_front)
l_r = lol*WD_front
wing = 0.874
start = -0.3-l_f-wing
final = start+2*75
length = final-start  
factor_bef = 0.52
CdA = 2.0 # for drag (changed)
ClA = 7.0 # for downforce (changed)
pair = 1.225
u_upper=18.0
m = 190.0   # mass of the car
g = 9.81
Iz = 110.0
ds_wanted=0.1
h_cog=0.27
eff=0.85
mi_fric = 0.03
ellipse_array=[]

#for dynamic model
# B=-7.0
# C=1.456
C_tire=0.66
# D= 3000.0
cs=-16419.31
dt_integration=0.025

#compute l_a
umin=4.0
umax=7.0

class State:
    def __init__(self, X, U):
        self.dF = U[0]
        self.dDelta = U[1]
        self.dS = U[2]
        self.X = X[0]
        self.Y = X[1]
        self.phi = X[2]
        self.vx = X[3]
        self.vy = X[4]
        self.r = X[5]
        self.F = X[6]
        self.delta = X[7]
        self.s = X[8]
    def get_validation(self):
        print("dF, X and Y are: ",self.dF, " ", self.X, " ",self.Y)
        
def getFy(Fz,sa):
    ex=1e-7
    P=[1.45673747e+00, -1.94554660e-04,  2.68063018e+00,  3.19197941e+04, 2.58020549e+00, -7.13319396e-04]
    C=P[0]
    D=P[1]*Fz*Fz+P[2]*Fz
    BCD = P[3]*casadi.sin(P[4]*casadi.arctan(P[5]*Fz))
    B=BCD/(C*D)
    F_temp = C_tire*D*casadi.sin(C*casadi.arctan(B*sa))
    return F_temp

def getSasWithState(model_class):
    saf_temp=casadi.arctan((model_class.vy+l_f*model_class.r)/(model_class.vx+1e-3)) - model_class.delta
    sar_temp=casadi.arctan((model_class.vy-l_r*model_class.r)/(model_class.vx+1e-3))
    return saf_temp,sar_temp

def getFzWithState(model_class):
    a_temp = (model_class.F - 0.5*CdA*pair*(model_class.vx)**2)/ m
    dw = (h_cog/lol)*(a_temp/g)*m*g
    Ffz_temp=(l_r/(l_f+l_r))*m*g + 0.25*pair*ClA*(model_class.vx**2) - dw
    Frz_temp=(l_f/(l_f+l_r))*m*g + 0.25*pair*ClA*(model_class.vx**2) + dw
    return Ffz_temp,Frz_temp

def continuous_dynamics(x, u):
    """Defines dynamics of the car, i.e. equality constraints.
    parameters:
    state x = [xPos,yPos,phi, vx, vy, r, F, delta, index]
    input u = [dF,ddelta, dindex]
    """
    state_ = State(x,u)
    temp1=casadi.fmax((state_.vx-umin)/(umax-umin),0)
    l_a=casadi.fmin(temp1,1)
    # saf=casadi.arctan((x[4]+l_f*x[5])/np.sqrt(x[3]**2+1)) - x[7]
    # sar=casadi.arctan((x[4]-l_r*x[5])/np.sqrt(x[3]**2+1))
    saf,sar = getSasWithState(state_) 
    Ffz,Frz = getFzWithState(state_)
    Ffy = getFy(Ffz,saf)
    Fry = getFy(Frz,sar)

    #friction forces
    Fdrag = 0.5*CdA*pair*(state_.vx)**2 + 0.03*(Frz+Ffz)
    print("loaded mdoel with values")
    print("State is",state_)

    #blending with changing lambda
    beta = casadi.arctan(l_r/(l_f + l_r) * casadi.tan(x[7]))
    # xdot = (l_a)*(x[3]*casadi.cos(x[2]) - x[4]*casadi.sin(x[2])) + (1-l_a)*(x[3]*casadi.cos(x[2] + beta))
    # ydot = (l_a)*(x[3]*casadi.sin(x[2]) + x[4]*casadi.cos(x[2])) + (1-l_a)*(x[3]*casadi.sin(x[2] + beta))
    xdot = state_.vx*casadi.cos(state_.phi) - state_.vy*casadi.sin(state_.phi)
    ydot = state_.vx*casadi.sin(state_.phi) + state_.vy*casadi.cos(state_.phi)
    phidot = state_.r
    vxdot = (1-l_a)*((state_.F - Fdrag)/ m) + (l_a)*((state_.F - Fdrag + Ffy*casadi.sin(state_.delta) + m*state_.vy*state_.r)/ m)
    vydot = (1-l_a)*((l_r/(l_r+l_f))*(vxdot*casadi.tan(state_.delta)+state_.vx*(state_.dDelta/(casadi.cos(state_.delta))**2))) + (l_a)*(((-state_.vx*state_.r) + (Fry + Ffy*casadi.cos(state_.delta)))/(1.0*m))
    rdot = (1-l_a)*((1/(l_r+l_f))*(vxdot*casadi.tan(state_.delta)+state_.vx*(state_.dDelta/(casadi.cos(state_.delta))**2))) + (l_a)*((Ffy*l_f*casadi.cos(state_.delta) - Fry*l_r)/(1.0*Iz))
    Fdot= state_.dF
    deltadot = state_.dDelta
    dindexdot = state_.dS
    return casadi.vertcat(xdot,ydot,phidot,vxdot,vydot,rdot,Fdot,deltadot,dindexdot)