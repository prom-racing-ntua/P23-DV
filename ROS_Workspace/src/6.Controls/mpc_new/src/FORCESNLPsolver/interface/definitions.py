import numpy
import ctypes

name = "FORCESNLPsolver"
requires_callback = True
lib = "lib/libFORCESNLPsolver.so"
lib_static = "lib/libFORCESNLPsolver.a"
c_header = "include/FORCESNLPsolver.h"
nstages = 30

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  8,   1),    8),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (300,   1),  300),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (120,   1),  120),
 ("reinitialize"        , "dense" , "FORCESNLPsolver_int", ctypes.c_int   , numpy.int32  , (  1,   1),    1),
 ("num_of_threads"      , "dense" , "solver_int32_unsigned", ctypes.c_uint  , numpy.uint32 , (  1,   1),    1)]

# Output                | Type    | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x02"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x03"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x04"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x05"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x06"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x07"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x08"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x09"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x10"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x11"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x12"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x13"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x14"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x15"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x16"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x17"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x18"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x19"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x20"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x21"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x22"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x23"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x24"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x25"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x26"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x27"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x28"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x29"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x30"                 , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10)]

# Info Struct Fields
info = \
[('it', ctypes.c_int),
 ('res_eq', ctypes.c_double),
 ('rsnorm', ctypes.c_double),
 ('pobj', ctypes.c_double),
 ('solvetime', ctypes.c_double),
 ('fevalstime', ctypes.c_double),
 ('QPtime', ctypes.c_double),
 ('QPit', ctypes.c_int),
 ('QPexitflag', ctypes.c_int),
 ('solver_id', ctypes.c_int * 8)
]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0), 
	(10, 8, 0, 4, 9, 9, 0, 0)
]