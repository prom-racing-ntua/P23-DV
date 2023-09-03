import casadi
import forcespro
import forcespro.nlp


def getEllipseParams(Fz):
    Fz0=1112.0554070627252
    dfz=(Fz-Fz0)/Fz0
    C_tire=0.66
    mx_max=C_tire*(2.21891927-1.36151651e-07*dfz)
    my_max=C_tire*(2.46810824-0.21654031*dfz)
    return mx_max,my_max

def getFy(Fz,sa):
    C_tire = 0.66
    ex = 1e-7
    P=[1.45673747e+00, -1.94554660e-04,  2.68063018e+00,  3.19197941e+04, 2.58020549e+00, -7.13319396e-04]
    C=P[0]
    D=P[1]*Fz*Fz+P[2]*Fz
    BCD = P[3]*casadi.sin(P[4]*casadi.arctan(P[5]*Fz))
    B=BCD/(C*D)
    F_temp = C_tire*D*casadi.sin(C*casadi.arctan(B*sa))
    return F_temp