# Decentralized-based-control-model-for-ENCE672
```python
import gurobipy as gurobipy
from gurobipy import *
import numpy
import scipy
import gurobipy
try:

    # Create a new model
    m = Model("ENCE672")
    m.Params.Timelimit = 3600
    #m.Params.MIPGap = 0.1
    #m.Params.NonConvex = 2
    m.Params.MIPFocus = 2
    m.Params.Method = 2

    # Create variables
    lv = 4 # The length of each vehicle, lv = 4 m
    lt = 1  # The length of each time interval, ls = 1 s
    vi = 6  # The initial speed of each vehicle, vi = 6 m/s
    vmax = 10  # Maximum vehicle velocity, vmax = 10 m/s
    vmin = 0  # Minimum vehicle velocity, vmin = 1 m/s
    am = 2  # Maximum vehicle acceleration rate, am = 2 m/s*s
    dm = -2 # The length of each space section, dm = -2 m/s*s
    ls = 1.0 # The minimum safe distance
    pi = 0.4 # The driver reaction time
    N_P1 = 12 # The number of vehicles entering path P1 within the time horizon
    N_P2 = 12 # The number of vehicles entering path P2 within the time horizon
    N_P3 = 12 # The number of vehicles entering path P3 within the time horizon
    N_P4 = 12 # The number of vehicles entering path P4 within the time horizon
    N_P5 = 12 # The number of vehicles entering path P5 within the time horizon
    N_P6 = 12 # The number of vehicles entering path P6 within the time horizon
    N_P7 = 12 # The number of vehicles entering path P7 within the time horizon
    N_P8 = 12 # The number of vehicles entering path P8 within the time horizon
    TH = 60 # The length of the time horizon
    x_1 = m.addVars(N_P1, TH, vtype=GRB.CONTINUOUS, name="x_1")
    x_2 = m.addVars(N_P2, TH, vtype=GRB.CONTINUOUS, name="x_2")
    x_3 = m.addVars(N_P3, TH, vtype=GRB.CONTINUOUS, name="x_3")
    x_4 = m.addVars(N_P4, TH, vtype=GRB.CONTINUOUS, name="x_4")
    x_5 = m.addVars(N_P5, TH, vtype=GRB.CONTINUOUS, name="x_5")
    x_6 = m.addVars(N_P6, TH, vtype=GRB.CONTINUOUS, name="x_6")
    x_7 = m.addVars(N_P7, TH, vtype=GRB.CONTINUOUS, name="x_7")
    x_8 = m.addVars(N_P8, TH, vtype=GRB.CONTINUOUS, name="x_8")

    v_1 = m.addVars(N_P1, TH, vtype=GRB.CONTINUOUS, name="v_1")
    v_2 = m.addVars(N_P2, TH, vtype=GRB.CONTINUOUS, name="v_2")
    v_3 = m.addVars(N_P3, TH, vtype=GRB.CONTINUOUS, name="v_3")
    v_4 = m.addVars(N_P4, TH, vtype=GRB.CONTINUOUS, name="v_4")
    v_5 = m.addVars(N_P5, TH, vtype=GRB.CONTINUOUS, name="v_5")
    v_6 = m.addVars(N_P6, TH, vtype=GRB.CONTINUOUS, name="v_6")
    v_7 = m.addVars(N_P7, TH, vtype=GRB.CONTINUOUS, name="v_7")
    v_8 = m.addVars(N_P8, TH, vtype=GRB.CONTINUOUS, name="v_8")
    a_1 = m.addVars(N_P1, TH, vtype=GRB.CONTINUOUS, name="a_1")
    # a_1 = m.addVars(N_P1, TH, vtype=GRB.CONTINUOUS, name="a_1") IF the model is feasible, you don't have to declare variable with this code. Making a>=0 can accelerate the solving process
    a_2 = m.addVars(N_P2, TH, vtype=GRB.CONTINUOUS, name="a_2")
    a_3 = m.addVars(N_P3, TH, vtype=GRB.CONTINUOUS, name="a_3")
    a_4 = m.addVars(N_P4, TH, lvtype=GRB.CONTINUOUS, name="a_4")
    a_5 = m.addVars(N_P5, TH, vtype=GRB.CONTINUOUS, name="a_5")
    a_6 = m.addVars(N_P6, TH, vtype=GRB.CONTINUOUS, name="a_6")
    a_7 = m.addVars(N_P7, TH, vtype=GRB.CONTINUOUS, name="a_7")
    a_8 = m.addVars(N_P8, TH, vtype=GRB.CONTINUOUS, name="a_8")

    et_1 = [0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22]
    et_2 = [1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23]
    et_3 = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24]
    et_4 = [3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25]
    et_5 = [4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26]
    et_6 = [5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27]
    et_7 = [6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28]
    et_8 = [7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29]
    dt_1 = m.addVars(N_P1, lb=0.0, ub=TH, vtype=GRB.INTEGER, name="dt_1")
    dt_2 = m.addVars(N_P2, lb=0.0, ub=TH, vtype=GRB.INTEGER, name="dt_2")
    dt_3 = m.addVars(N_P3, lb=0.0, ub=TH, vtype=GRB.INTEGER, name="dt_3")
    dt_4 = m.addVars(N_P4, lb=0.0, ub=TH, vtype=GRB.INTEGER, name="dt_4")
    dt_5 = m.addVars(N_P5, lb=0.0, ub=TH, vtype=GRB.INTEGER, name="dt_5")
    dt_6 = m.addVars(N_P6, lb=0.0, ub=TH, vtype=GRB.INTEGER, name="dt_6")
    dt_7 = m.addVars(N_P7, lb=0.0, ub=TH, vtype=GRB.INTEGER, name="dt_7")
    dt_8 = m.addVars(N_P8, lb=0.0, ub=TH, vtype=GRB.INTEGER, name="dt_8")
    tc_1_up_P1 = m.addMVar(N_P1, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_1_up_P1")
    tc_1_up_P8 = m.addMVar(N_P8, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_1_up_P8")
    tc_1_down_P1 = m.addMVar(N_P1, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_1_down_P1")
    tc_1_down_P8 = m.addMVar(N_P8, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_1_down_P8")
    tc_2_up_P1 = m.addMVar(N_P1, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_2_up_P1")
    tc_2_up_P3 = m.addMVar(N_P3, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_2_up_P3")
    tc_2_down_P1 = m.addMVar(N_P1, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_2_down_P1")
    tc_2_down_P3 = m.addMVar(N_P3, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_2_down_P3")
    tc_3_up_P1 = m.addMVar(N_P1, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_3_up_P1")
    tc_3_up_P7 = m.addMVar(N_P7, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_3_up_P7")
    tc_3_down_P1 = m.addMVar(N_P1, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_3_down_P1")
    tc_3_down_P7 = m.addMVar(N_P7, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_3_down_P7")
    tc_4_up_P1 = m.addMVar(N_P1, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_4_up_P1")
    tc_4_up_P6 = m.addMVar(N_P6, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_4_up_P6")
    tc_4_down_P1 = m.addMVar(N_P1, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_4_down_P1")
    tc_4_down_P6 = m.addMVar(N_P6, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_4_down_P6")
    tc_5_up_P2 = m.addMVar(N_P2, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_5_up_P2")
    tc_5_up_P8 = m.addMVar(N_P8, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_5_up_P8")
    tc_5_down_P2 = m.addMVar(N_P2, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_5_down_P2")
    tc_5_down_P8 = m.addMVar(N_P8, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_5_down_P8")
    tc_6_up_P2 = m.addMVar(N_P2, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_6_up_P2")
    tc_6_up_P5 = m.addMVar(N_P5, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_6_up_P5")
    tc_6_down_P2 = m.addMVar(N_P2, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_6_down_P2")
    tc_6_down_P5 = m.addMVar(N_P5, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_6_down_P5")
    tc_7_up_P2 = m.addMVar(N_P2, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_7_up_P2")
    tc_7_up_P3 = m.addMVar(N_P3, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_7_up_P3")
    tc_7_down_P2= m.addMVar(N_P2, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_7_down_P2")
    tc_7_down_P3= m.addMVar(N_P3, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_7_down_P3")
    tc_8_up_P2 = m.addMVar(N_P2, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_8_up_P2")
    tc_8_up_P4 = m.addMVar(N_P4, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_8_up_P4")
    tc_8_down_P2 = m.addMVar(N_P2, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_8_down_P2")
    tc_8_down_P4 = m.addMVar(N_P4, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_8_down_P4")
    tc_9_up_P3 = m.addMVar(N_P3, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_9_up_P3")
    tc_9_up_P5 = m.addMVar(N_P5, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_9_up_P5")
    tc_9_down_P3 = m.addMVar(N_P3, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_9_down_P3")
    tc_9_down_P5 = m.addMVar(N_P5, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_9_down_P5")
    tc_10_up_P3 = m.addMVar(N_P3, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_10_up_P3")
    tc_10_up_P8 = m.addMVar(N_P8, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_10_up_P8")
    tc_10_down_P3 = m.addMVar(N_P3, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_10_down_P3")
    tc_10_down_P8 = m.addMVar(N_P8, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_10_down_P8")
    tc_11_up_P4 = m.addMVar(N_P4, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_11_up_P4")
    tc_11_up_P7 = m.addMVar(N_P7, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_11_up_P7")
    tc_11_down_P4 = m.addMVar(N_P4, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_11_down_P4")
    tc_11_down_P7 = m.addMVar(N_P7, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_11_down_P7")
    tc_12_up_P4 = m.addMVar(N_P4, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_12_up_P4")
    tc_12_up_P5 = m.addMVar(N_P5, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_12_up_P5")
    tc_12_down_P4 = m.addMVar(N_P4, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_12_down_P4")
    tc_12_down_P5 = m.addMVar(N_P5, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_12_down_P5")
    tc_13_up_P4 = m.addMVar(N_P4, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_13_up_P4")
    tc_13_up_P6 = m.addMVar(N_P6, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_13_up_P6")
    tc_13_down_P4 = m.addMVar(N_P4, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_13_down_P4")
    tc_13_down_P6 = m.addMVar(N_P6, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_13_down_P6")
    tc_14_up_P6 = m.addMVar(N_P6, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_14_up_P6")
    tc_14_up_P7 = m.addMVar(N_P7, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_14_up_P7")
    tc_14_down_P6 = m.addMVar(N_P6, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_14_down_P6")
    tc_14_down_P7 = m.addMVar(N_P7, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_14_down_P7")
    tc_15_up_P6 = m.addMVar(N_P6, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_15_up_P6")
    tc_15_up_P8 = m.addMVar(N_P8, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_15_up_P8")
    tc_15_down_P6 = m.addMVar(N_P6, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_15_down_P6")
    tc_15_down_P8 = m.addMVar(N_P8, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_15_down_P8")
    tc_16_up_P7 = m.addMVar(N_P7, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_16_up_P7")
    tc_16_up_P5 = m.addMVar(N_P5, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_16_up_P5")
    tc_16_down_P7 = m.addMVar(N_P7, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_16_down_P7")
    tc_16_down_P5 = m.addMVar(N_P5, lb = 0,  vtype=GRB.CONTINUOUS, name="tc_16_down_P5")
    y_18 = m.addMVar((N_P1, TH), vtype=GRB.BINARY, name="y_18")
    y_81 = m.addMVar((N_P8, TH), vtype=GRB.BINARY, name="y_81")
    y_13 = m.addMVar((N_P1, TH), vtype=GRB.BINARY, name="y_13")
    y_31 = m.addMVar((N_P3, TH), vtype=GRB.BINARY, name="y_31")
    y_17 = m.addMVar((N_P1, TH), vtype=GRB.BINARY, name="y_17")
    y_71 = m.addMVar((N_P7, TH), vtype=GRB.BINARY, name="y_71")
    y_16 = m.addMVar((N_P1, TH), vtype=GRB.BINARY, name="y_16")
    y_61 = m.addMVar((N_P6, TH), vtype=GRB.BINARY, name="y_61")
    y_28 = m.addMVar((N_P2, TH), vtype=GRB.BINARY, name="y_28")
    y_82 = m.addMVar((N_P8, TH), vtype=GRB.BINARY, name="y_82")
    y_25 = m.addMVar((N_P2, TH), vtype=GRB.BINARY, name="y_25")
    y_52 = m.addMVar((N_P5, TH), vtype=GRB.BINARY, name="y_52")
    y_23 = m.addMVar((N_P2, TH), vtype=GRB.BINARY, name="y_23")
    y_32 = m.addMVar((N_P3, TH), vtype=GRB.BINARY, name="y_32")
    y_24 = m.addMVar((N_P2, TH), vtype=GRB.BINARY, name="y_24")
    y_42 = m.addMVar((N_P4, TH), vtype=GRB.BINARY, name="y_42")
    y_35 = m.addMVar((N_P3, TH), vtype=GRB.BINARY, name="y_35")
    y_53 = m.addMVar((N_P5, TH), vtype=GRB.BINARY, name="y_53")
    y_38 = m.addMVar((N_P3, TH), vtype=GRB.BINARY, name="y_38")
    y_83 = m.addMVar((N_P8, TH), vtype=GRB.BINARY, name="y_83")
    y_47 = m.addMVar((N_P4, TH), vtype=GRB.BINARY, name="y_47")
    y_74 = m.addMVar((N_P7, TH), vtype=GRB.BINARY, name="y_74")
    y_45 = m.addMVar((N_P4, TH), vtype=GRB.BINARY, name="y_45")
    y_54 = m.addMVar((N_P5, TH), vtype=GRB.BINARY, name="y_54")
    y_46 = m.addMVar((N_P4, TH), vtype=GRB.BINARY, name="y_46")
    y_64 = m.addMVar((N_P6, TH), vtype=GRB.BINARY, name="y_64")
    y_67 = m.addMVar((N_P6, TH), vtype=GRB.BINARY, name="y_67")
    y_76 = m.addMVar((N_P7, TH), vtype=GRB.BINARY, name="y_76")
    y_68 = m.addMVar((N_P6, TH), vtype=GRB.BINARY, name="y_68")
    y_86 = m.addMVar((N_P8, TH), vtype=GRB.BINARY, name="y_86")
    y_75 = m.addMVar((N_P7, TH), vtype=GRB.BINARY, name="y_75")
    y_57 = m.addMVar((N_P5, TH), vtype=GRB.BINARY, name="y_57")

    yplus_18 = m.addMVar((N_P1, TH), vtype=GRB.BINARY, name="yplus_18")
    yplus_81 = m.addMVar((N_P8, TH), vtype=GRB.BINARY, name="yplus_81")
    yplus_13 = m.addMVar((N_P1, TH), vtype=GRB.BINARY, name="yplus_13")
    yplus_31 = m.addMVar((N_P3, TH), vtype=GRB.BINARY, name="yplus_31")
    yplus_17 = m.addMVar((N_P1, TH), vtype=GRB.BINARY, name="yplus_17")
    yplus_71 = m.addMVar((N_P7, TH), vtype=GRB.BINARY, name="yplus_71")
    yplus_16 = m.addMVar((N_P1, TH), vtype=GRB.BINARY, name="yplus_16")
    yplus_61 = m.addMVar((N_P6, TH), vtype=GRB.BINARY, name="yplus_61")
    yplus_28 = m.addMVar((N_P2, TH), vtype=GRB.BINARY, name="yplus_28")
    yplus_82 = m.addMVar((N_P8, TH), vtype=GRB.BINARY, name="yplus_82")
    yplus_25 = m.addMVar((N_P2, TH), vtype=GRB.BINARY, name="yplus_25")
    yplus_52 = m.addMVar((N_P5, TH), vtype=GRB.BINARY, name="yplus_52")
    yplus_23 = m.addMVar((N_P2, TH), vtype=GRB.BINARY, name="yplus_23")
    yplus_32 = m.addMVar((N_P3, TH), vtype=GRB.BINARY, name="yplus_32")
    yplus_24 = m.addMVar((N_P2, TH), vtype=GRB.BINARY, name="yplus_24")
    yplus_42 = m.addMVar((N_P4, TH), vtype=GRB.BINARY, name="yplus_42")
    yplus_35 = m.addMVar((N_P3, TH), vtype=GRB.BINARY, name="yplus_35")
    yplus_53 = m.addMVar((N_P5, TH), vtype=GRB.BINARY, name="yplus_53")
    yplus_38 = m.addMVar((N_P3, TH), vtype=GRB.BINARY, name="yplus_38")
    yplus_83 = m.addMVar((N_P8, TH), vtype=GRB.BINARY, name="yplus_83")
    yplus_47 = m.addMVar((N_P4, TH), vtype=GRB.BINARY, name="yplus_47")
    yplus_74 = m.addMVar((N_P7, TH), vtype=GRB.BINARY, name="yplus_74")
    yplus_45 = m.addMVar((N_P4, TH), vtype=GRB.BINARY, name="yplus_45")
    yplus_54 = m.addMVar((N_P5, TH), vtype=GRB.BINARY, name="yplus_54")
    yplus_46 = m.addMVar((N_P4, TH), vtype=GRB.BINARY, name="yplus_46")
    yplus_64 = m.addMVar((N_P6, TH), vtype=GRB.BINARY, name="yplus_64")
    yplus_67 = m.addMVar((N_P6, TH), vtype=GRB.BINARY, name="yplus_67")
    yplus_76 = m.addMVar((N_P7, TH), vtype=GRB.BINARY, name="yplus_76")
    yplus_68 = m.addMVar((N_P6, TH), vtype=GRB.BINARY, name="yplus_68")
    yplus_86 = m.addMVar((N_P8, TH), vtype=GRB.BINARY, name="yplus_86")
    yplus_75 = m.addMVar((N_P7, TH), vtype=GRB.BINARY, name="yplus_75")
    yplus_57 = m.addMVar((N_P5, TH), vtype=GRB.BINARY, name="yplus_57")
    M = 9999
    c_1 = m.addVars(N_P1, TH, vtype=GRB.BINARY, name="c_1")
    c_2 = m.addVars(N_P2, TH, vtype=GRB.BINARY, name="c_2")
    c_3 = m.addVars(N_P3, TH, vtype=GRB.BINARY, name="c_3")
    c_4 = m.addVars(N_P4, TH, vtype=GRB.BINARY, name="c_4")
    c_5 = m.addVars(N_P5, TH, vtype=GRB.BINARY, name="c_5")
    c_6 = m.addVars(N_P6, TH, vtype=GRB.BINARY, name="c_6")
    c_7 = m.addVars(N_P7, TH, vtype=GRB.BINARY, name="c_7")
    c_8 = m.addVars(N_P8, TH, vtype=GRB.BINARY, name="c_8")
    w_1 = m.addVars(N_P1, TH, vtype=GRB.BINARY, name="w_1")
    w_2 = m.addVars(N_P2, TH, vtype=GRB.BINARY, name="w_2")
    w_3 = m.addVars(N_P3, TH, vtype=GRB.BINARY, name="w_3")
    w_4 = m.addVars(N_P4, TH, vtype=GRB.BINARY, name="w_4")
    w_5 = m.addVars(N_P5, TH, vtype=GRB.BINARY, name="w_5")
    w_6 = m.addVars(N_P6, TH, vtype=GRB.BINARY, name="w_6")
    w_7 = m.addVars(N_P7, TH, vtype=GRB.BINARY, name="w_7")
    w_8 = m.addVars(N_P8, TH, vtype=GRB.BINARY, name="w_8")
    o_1 = m.addVars(N_P1, TH, vtype=GRB.BINARY, name="o_1")
    o_2 = m.addVars(N_P2, TH, vtype=GRB.BINARY, name="o_2")
    o_3 = m.addVars(N_P3, TH, vtype=GRB.BINARY, name="o_3")
    o_4 = m.addVars(N_P4, TH, vtype=GRB.BINARY, name="o_4")
    o_5 = m.addVars(N_P5, TH, vtype=GRB.BINARY, name="o_5")
    o_6 = m.addVars(N_P6, TH, vtype=GRB.BINARY, name="o_6")
    o_7 = m.addVars(N_P7, TH, vtype=GRB.BINARY, name="o_7")
    o_8 = m.addVars(N_P8, TH, vtype=GRB.BINARY, name="o_8")
    q_1 = m.addVars(N_P1, TH, vtype=GRB.INTEGER, name="q_1")
    q_2 = m.addVars(N_P2, TH, vtype=GRB.INTEGER, name="q_2")
    q_3 = m.addVars(N_P3, TH, vtype=GRB.INTEGER, name="q_3")
    q_4 = m.addVars(N_P4, TH, vtype=GRB.INTEGER, name="q_4")
    q_5 = m.addVars(N_P5, TH, vtype=GRB.INTEGER, name="q_5")
    q_6 = m.addVars(N_P6, TH, vtype=GRB.INTEGER, name="q_6")
    q_7 = m.addVars(N_P7, TH, vtype=GRB.INTEGER, name="q_7")
    q_8 = m.addVars(N_P8, TH, vtype=GRB.INTEGER, name="q_8")
    m.update()

    # Initial conditions


    for i in range(N_P1):
        m.addConstr(x_1[i, et_1[i]] == 0)
        m.addConstr(v_1[i, et_1[i]] == 6)
        for j in range(TH):
            m.addConstr(x_1[i, j] - 158.4 <= M * o_1[i, j] )
            m.addConstr(x_1[i, j] - 158.4 >= M * (o_1[i, j] - 1))
            m.addConstr((o_1[i, j] == 1) >> (dt_1[i] <= j + 1))
            m.addConstr((o_1[i, j] == 0) >> (dt_1[i] >= j + 1))



    for i in range(N_P2):
        m.addConstr(x_2[i, et_2[i]] == 0)
        m.addConstr(v_2[i, et_2[i]] == 6)
        for j in range(TH):
            m.addConstr(x_2[i, j] - 160 <= M * o_2[i, j] )
            m.addConstr(x_2[i, j] - 160 >= M * (o_2[i, j] - 1))
            m.addConstr((o_2[i, j] == 1) >> (dt_2[i] <= j + 1))
            m.addConstr((o_2[i, j] == 0) >> (dt_2[i] >= j + 1))
    for i in range(N_P3):
        m.addConstr(x_3[i, et_3[i]] == 0)
        m.addConstr(v_3[i, et_3[i]] == 6)
        for j in range(TH):
            m.addConstr(x_3[i, j] - 158.4 <= M * o_3[i, j] )
            m.addConstr(x_3[i, j] - 158.4 >= M * (o_3[i, j] - 1))
            m.addConstr((o_3[i, j] == 1) >> (dt_3[i] <= j + 1))
            m.addConstr((o_3[i, j] == 0) >> (dt_3[i] >= j + 1))
    for i in range(N_P4):
        m.addConstr(x_4[i, et_4[i]] == 0)
        m.addConstr(v_4[i, et_4[i]] == 6)
        for j in range(TH):
            m.addConstr(x_4[i, j] - 160 <= M * o_4[i, j] )
            m.addConstr(x_4[i, j] - 160 >= M * (o_4[i, j] - 1))
            m.addConstr((o_4[i, j] == 1) >> (dt_4[i] <= j + 1))
            m.addConstr((o_4[i, j] == 0) >> (dt_4[i] >= j + 1))
    for i in range(N_P5):
        m.addConstr(x_5[i, et_5[i]] == 0)
        m.addConstr(v_5[i, et_5[i]] == 6)
        for j in range(TH):
            m.addConstr(x_5[i, j] - 158.4 <= M * o_5[i, j] )
            m.addConstr(x_5[i, j] - 158.4 >= M * (o_5[i, j] - 1))
            m.addConstr((o_5[i, j] == 1) >> (dt_5[i] <= j + 1))
            m.addConstr((o_5[i, j] == 0) >> (dt_5[i] >= j + 1))
    for i in range(N_P6):
        m.addConstr(x_6[i, et_6[i]] == 0)
        m.addConstr(v_6[i, et_6[i]] == 6)
        for j in range(TH):
            m.addConstr(x_6[i, j] - 160 <= M * o_6[i, j] )
            m.addConstr(x_6[i, j] - 160 >= M * (o_6[i, j] - 1))
            m.addConstr((o_6[i, j] == 1) >> (dt_6[i] <= j + 1))
            m.addConstr((o_6[i, j] == 0) >> (dt_6[i] >= j + 1))
    for i in range(N_P7):
        m.addConstr(x_7[i, et_7[i]] == 0)
        m.addConstr(v_7[i, et_7[i]] == 6)
        for j in range(TH):
            m.addConstr(x_7[i, j] - 158.4 <= M * o_7[i, j] )
            m.addConstr(x_7[i, j] - 158.4 >= M * (o_7[i, j] - 1))
            m.addConstr((o_7[i, j] == 1) >> (dt_7[i] <= j + 1))
            m.addConstr((o_7[i, j] == 0) >> (dt_7[i] >= j + 1))
    for i in range(N_P8):
        m.addConstr(x_8[i, et_8[i]] == 0)
        m.addConstr(v_8[i, et_8[i]] == 6)
        for j in range(TH):
            m.addConstr(x_8[i, j] - 160 <= M * o_8[i, j] )
            m.addConstr(x_8[i, j] - 160 >= M * (o_8[i, j] - 1))
            m.addConstr((o_8[i, j] == 1) >> (dt_8[i] <= j + 1))
            m.addConstr((o_8[i, j] == 0) >> (dt_8[i] >= j + 1))

    # Time horizon qualification constraint

    for i in range(N_P1):
        m.addConstr(x_1[i, TH - 1] >= 158.4)
    for i in range(N_P2):
        m.addConstr(x_2[i, TH - 1] >= 160)
    for i in range(N_P3):
        m.addConstr(x_3[i, TH - 1] >= 158.4)
    for i in range(N_P4):
        m.addConstr(x_4[i, TH - 1] >= 160)
    for i in range(N_P5):
        m.addConstr(x_5[i, TH - 1] >= 158.4)
    for i in range(N_P6):
        m.addConstr(x_6[i, TH - 1] >= 160)
    for i in range(N_P7):
        m.addConstr(x_7[i, TH - 1] >= 158.4)
    for i in range(N_P8):
        m.addConstr(x_8[i, TH - 1] >= 160)

    # Adding kinematic constraints
    for i in range(N_P1):
        for j in range(et_1[i], TH):
            m.addConstr(vmin <= v_1[i, j])
            m.addConstr(v_1[i, j] <= vmax)
        for k in range(et_1[i], TH-1):
            m.addConstr(v_1[i, k] + a_1[i, k] * lt == v_1[i, k + 1])
            m.addConstr(a_1[i, k] <= am)
            m.addConstr(a_1[i, k] >= dm)
            m.addConstr(x_1[i, k + 1] == x_1[i, k] + v_1[i, k] * lt + 0.5 * a_1[i, k] * lt * lt)

    for i in range(N_P2):
        for j in range(et_2[i], TH):
            m.addConstr(vmin <= v_2[i, j])
            m.addConstr(v_2[i, j] <= vmax)
        for k in range(et_2[i], TH-1):
            m.addConstr(v_2[i, k] + a_2[i, k] * lt == v_2[i, k + 1])
            m.addConstr(a_2[i, k] <= am)
            m.addConstr(a_2[i, k] >= dm)
            m.addConstr(x_2[i, k + 1] == x_2[i, k] + v_2[i, k] * lt + 0.5 * a_2[i, k] * lt * lt)
    for i in range(N_P3):
        for j in range(et_3[i], TH):
            m.addConstr(vmin <= v_3[i, j])
            m.addConstr(v_3[i, j] <= vmax)
        for k in range(et_3[i], TH-1):
            m.addConstr(v_3[i, k] + a_3[i, k] * lt == v_3[i, k + 1])
            m.addConstr(a_3[i, k] <= am)
            m.addConstr(a_3[i, k] >= dm)
            m.addConstr(x_3[i, k + 1] == x_3[i, k] + v_3[i, k] * lt + 0.5 * a_3[i, k] * lt * lt)
    for i in range(N_P4):
        for j in range(et_4[i], TH):
            m.addConstr(vmin <= v_4[i, j])
            m.addConstr(v_4[i, j] <= vmax)
        for k in range(et_4[i], TH-1):
            m.addConstr(v_4[i, k] + a_4[i, k] * lt == v_4[i, k + 1])
            m.addConstr(a_4[i, k] <= am)
            m.addConstr(a_4[i, k] >= dm)
            m.addConstr(x_4[i, k + 1] == x_4[i, k] + v_4[i, k] * lt + 0.5 * a_4[i, k] * lt * lt)
    for i in range(N_P5):
        for j in range(et_5[i], TH):
            m.addConstr(vmin <= v_5[i, j])
            m.addConstr(v_5[i, j] <= vmax)
        for k in range(et_5[i], TH-1):
            m.addConstr(v_5[i, k] + a_5[i, k] * lt == v_5[i, k + 1])
            m.addConstr(a_5[i, k] <= am)
            m.addConstr(a_5[i, k] >= dm)
            m.addConstr(x_5[i, k + 1] == x_5[i, k] + v_5[i, k] * lt + 0.5 * a_5[i, k] * lt * lt)
    for i in range(N_P6):
        for j in range(et_6[i], TH):
            m.addConstr(vmin <= v_6[i, j])
            m.addConstr(v_6[i, j] <= vmax)
        for k in range(et_6[i], TH-1):
            m.addConstr(v_6[i, k] + a_6[i, k] * lt == v_6[i, k + 1])
            m.addConstr(a_6[i, k] <= am)
            m.addConstr(a_6[i, k] >= dm)
            m.addConstr(x_6[i, k + 1] == x_6[i, k] + v_6[i, k] * lt + 0.5 * a_6[i, k] * lt * lt)
    for i in range(N_P7):
        for j in range(et_7[i], TH):
            m.addConstr(vmin <= v_7[i, j])
            m.addConstr(v_7[i, j] <= vmax)
        for k in range(et_7[i], TH-1):
            m.addConstr(v_7[i, k] + a_7[i, k] * lt == v_7[i, k + 1])
            m.addConstr(a_7[i, k] <= am)
            m.addConstr(a_7[i, k] >= dm)
            m.addConstr(x_7[i, k + 1] == x_7[i, k] + v_7[i, k] * lt + 0.5 * a_7[i, k] * lt * lt)
    for i in range(N_P8):
        for j in range(et_8[i], TH):
            m.addConstr(vmin <= v_8[i, j])
            m.addConstr(v_8[i, j] <= vmax)
        for k in range(et_8[i], TH-1):
            m.addConstr(v_8[i, k] + a_8[i, k] * lt == v_8[i, k + 1])
            m.addConstr(a_8[i, k] <= am)
            m.addConstr(a_8[i, k] >= dm)
            m.addConstr(x_8[i, k + 1] == x_8[i, k] + v_8[i, k] * lt + 0.5 * a_8[i, k] * lt * lt)

    # Adding  safe distance constraints
    for i in range(N_P1-1):
        for j in range(et_1[i + 1], TH):
            m.addConstr(x_1[i, j] - x_1[i + 1, j] >= lv + ls)
            m.addConstr(x_1[i, j] - x_1[i + 1, j] >= lv + pi * v_1[i + 1, j])
    for i in range(N_P2-1):
        for j in range(et_2[i + 1], TH):
            m.addConstr(x_2[i, j] - x_2[i + 1, j] >= lv + ls)
            m.addConstr(x_2[i, j] - x_2[i + 1, j] >= lv + pi * v_2[i + 1, j])
    for i in range(N_P3-1):
        for j in range(et_3[i + 1], TH):
            m.addConstr(x_3[i, j] - x_3[i + 1, j] >= lv + ls)
            m.addConstr(x_3[i, j] - x_3[i + 1, j] >= lv + pi * v_3[i + 1, j])
    for i in range(N_P4-1):
        for j in range(et_4[i + 1], TH):
            m.addConstr(x_4[i, j] - x_4[i + 1, j] >= lv + ls)
            m.addConstr(x_4[i, j] - x_4[i + 1, j] >= lv + pi * v_4[i + 1, j])
    for i in range(N_P5-1):
        for j in range(et_5[i + 1], TH):
            m.addConstr(x_5[i, j] - x_5[i + 1, j] >= lv + ls)
            m.addConstr(x_5[i, j] - x_5[i + 1, j] >= lv + pi * v_5[i + 1, j])
    for i in range(N_P6-1):
        for j in range(et_6[i + 1], TH):
            m.addConstr(x_6[i, j] - x_6[i + 1, j] >= lv + ls)
            m.addConstr(x_6[i, j] - x_6[i + 1, j] >= lv + pi * v_6[i + 1, j])
    for i in range(N_P7-1):
        for j in range(et_7[i + 1], TH):
            m.addConstr(x_7[i, j] - x_7[i + 1, j] >= lv + ls)
            m.addConstr(x_7[i, j] - x_7[i + 1, j] >= lv + pi * v_7[i + 1, j])
    for i in range(N_P8-1):
        for j in range(et_8[i + 1], TH):
            m.addConstr(x_8[i, j] - x_8[i + 1, j] >= lv + ls)
            m.addConstr(x_8[i, j] - x_8[i + 1, j] >= lv + pi * v_8[i + 1, j])

    # Adding collision avoidance constraints
    for i in range(N_P1):
        for j in range(N_P8):
            for k in range(TH):
                m.addConstr(y_18[i, k]*M >= x_1[i, k] - 72.633276272)
                m.addConstr((y_18[i, k] - 1) * M <= x_1[i, k] - 72.633276272)
                m.addConstr(yplus_18[i, k] * M >= 82.27614523 - x_1[i, k])
                m.addConstr((yplus_18[i, k] - 1) * M <= 82.27614523 - x_1[i, k])
                m.addConstr(y_81[j, k]*M >= x_8[j, k] - 77.599342077)
                m.addConstr((y_81[j, k] - 1) * M <= x_8[j, k] - 77.599342077)
                m.addConstr(yplus_81[j, k] * M >= 87.1624 - x_8[j, k])
                m.addConstr((yplus_81[j, k] - 1) * M <=  87.1624 - x_8[j, k])
                m.addConstr(y_18[i, k] + yplus_18[i, k] + y_81[j, k] + yplus_81[j, k] <= 3)

    for i in range(N_P1):
        for j in range(N_P3):
            for k in range(TH):
                m.addConstr(y_13[i, k] * M >= x_1[i, k] - 70)
                m.addConstr((y_13[i, k] - 1) * M <= x_1[i, k] - 70)
                m.addConstr(yplus_13[i, k] * M >= 82.6749032 - x_1[i, k] )
                m.addConstr((yplus_13[i, k] - 1) * M <= 82.6749032 - x_1[i, k] )
                m.addConstr(y_31[j, k] * M >= x_3[j, k] - 79.801524854)
                m.addConstr((y_31[j, k] - 1) * M <= x_3[j, k] - 79.801524854)
                m.addConstr(yplus_31[j, k] * M >= 89.83476081 - x_3[j, k])
                m.addConstr((yplus_31[j, k] - 1) * M <= 89.83476081 - x_3[j, k])
                m.addConstr(y_13[i, k] + yplus_13[i, k] + y_31[j, k] + yplus_31[j, k] <= 3)


    for i in range(N_P1):
        for j in range(N_P7):
            for k in range(TH):
                m.addConstr(y_17[i, k]*M >= x_1[i, k] - 79.801524854)
                m.addConstr((y_17[i, k] - 1) * M <= x_1[i, k] - 79.801524854)
                m.addConstr(yplus_17[i, k] * M >= 89.83476081 - x_1[i, k])
                m.addConstr((yplus_17[i, k] - 1) * M <= 89.83476081 - x_1[i, k])
                m.addConstr(y_71[j, k]*M >= x_7[j, k] - 70)
                m.addConstr((y_71[j, k] - 1) * M <= x_7[j, k] - 70)
                m.addConstr(yplus_71[j, k] * M >= 82.6749032 - x_7[j, k])
                m.addConstr((yplus_71[j, k] - 1) * M <= 82.6749032 - x_7[j, k])
                m.addConstr(y_17[i, k] + yplus_17[i, k] + y_71[j, k] + yplus_71[j, k] <= 3)

    for i in range(N_P1):
        for j in range(N_P6):
            for k in range(TH):
                m.addConstr(y_16[i, k]*M >= x_1[i, k] - 78.314117135)
                m.addConstr((y_16[i, k] - 1) * M <= x_1[i, k] - 78.314117135)
                m.addConstr(yplus_16[i, k] * M >=  89.83476081 - x_1[i, k])
                m.addConstr((yplus_16[i, k] - 1) * M <= 89.83476081 - x_1[i, k])
                m.addConstr(y_61[j, k]*M >= x_6[j, k] - 76.83755342)
                m.addConstr((y_61[j, k] - 1) * M <= x_6[j, k] - 76.83755342)
                m.addConstr(yplus_61[j, k] * M >= 87.16244658 - x_6[j, k])
                m.addConstr((yplus_61[j, k] - 1) * M <= 87.16244658 - x_6[j, k])
                m.addConstr(y_16[i, k] + yplus_16[i, k] + y_61[j, k] + yplus_61[j, k] <= 3)

    for i in range(N_P2):
        for j in range(N_P8):
            for k in range(TH):
                m.addConstr(y_28[i, k]*M >= x_2[i, k] - 73)
                m.addConstr((y_28[i, k] - 1) * M <= x_2[i, k] - 73)
                m.addConstr(yplus_28[i, k] * M >=  80.5 - x_2[i, k])
                m.addConstr((yplus_28[i, k] - 1) * M <= 80.5 - x_2[i, k])
                m.addConstr(y_82[j, k]*M >= x_8[j, k] - 83.5)
                m.addConstr((y_82[j, k] - 1) * M <= x_8[j, k] - 83.5)
                m.addConstr(yplus_82[j, k] * M >= 91 - x_8[j, k])
                m.addConstr((yplus_82[j, k] - 1) * M <= 91 - x_8[j, k])
                m.addConstr(y_28[i, k] + yplus_28[i, k] + y_82[j, k] + yplus_82[j, k] <= 3)

    for i in range(N_P2):
        for j in range(N_P5):
            for k in range(TH):
                m.addConstr(y_25[i, k]*M >= x_2[i, k] - 76.83755342)
                m.addConstr((y_25[i, k] - 1) * M <= x_2[i, k] - 76.83755342)
                m.addConstr(yplus_25[i, k] * M >=  87.16244658 - x_2[i, k])
                m.addConstr((yplus_25[i, k] - 1) * M <= 87.16244658- x_2[i, k])
                m.addConstr(y_52[j, k]*M >= x_5[j, k] - 78.314117135)
                m.addConstr((y_52[j, k] - 1) * M <= x_5[j, k] - 78.314117135)
                m.addConstr(yplus_52[j, k] * M >= 89.83476081 - x_5[j, k])
                m.addConstr((yplus_52[j, k] - 1) * M <= 89.83476081 - x_5[j, k])
                m.addConstr(y_25[i, k] + yplus_25[i, k] + y_52[j, k] + yplus_52[j, k] <= 3)

    for i in range(N_P2):
        for j in range(N_P3):
            for k in range(TH):
                m.addConstr(y_23[i, k]*M >= x_2[i, k] - 77.599342077)
                m.addConstr((y_23[i, k] - 1) * M <= x_2[i, k] - 77.599342077)
                m.addConstr(yplus_23[i, k] * M >=  87.1624 - x_2[i, k])
                m.addConstr((yplus_23[i, k] - 1) * M <= 87.1624- x_2[i, k])
                m.addConstr(y_32[j, k]*M >= x_3[j, k] - 72.633276272)
                m.addConstr((y_32[j, k] - 1) * M <= x_3[j, k] - 72.633276272)
                m.addConstr(yplus_32[j, k] * M >= 82.27614523 - x_3[j, k])
                m.addConstr((yplus_32[j, k] - 1) * M <= 82.27614523 - x_3[j, k])
                m.addConstr(y_23[i, k] + yplus_23[i, k] + y_32[j, k] + yplus_32[j, k] <= 3)

    for i in range(N_P2):
        for j in range(N_P4):
            for k in range(TH):
                m.addConstr(y_24[i, k]*M >= x_2[i, k] - 83.5)
                m.addConstr((y_24[i, k] - 1) * M <= x_2[i, k] - 83.5)
                m.addConstr(yplus_24[i, k] * M >=  91 - x_2[i, k])
                m.addConstr((yplus_24[i, k] - 1) * M <= 91 - x_2[i, k])
                m.addConstr(y_42[j, k]*M >= x_4[j, k] - 73)
                m.addConstr((y_42[j, k] - 1) * M <= x_4[j, k] - 73)
                m.addConstr(yplus_42[j, k] * M >= 80.5 - x_4[j, k])
                m.addConstr((yplus_42[j, k] - 1) * M <= 80.5 - x_4[j, k])
                m.addConstr(y_24[i, k] + yplus_24[i, k] + y_42[j, k] + yplus_42[j, k] <= 3)

    for i in range(N_P3):
        for j in range(N_P5):
            for k in range(TH):
                m.addConstr(y_35[i, k]*M >= x_3[i, k] - 70)
                m.addConstr((y_35[i, k] - 1) * M <= x_3[i, k] - 70)
                m.addConstr(yplus_35[i, k] * M >=  82.6749032  - x_3[i, k])
                m.addConstr((yplus_35[i, k] - 1) * M <= 82.6749032  - x_3[i, k])
                m.addConstr(y_53[j, k]*M >= x_5[j, k] - 79.801524854)
                m.addConstr((y_53[j, k] - 1) * M <= x_5[j, k] - 79.801524854)
                m.addConstr(yplus_53[j, k] * M >= 89.83476081 - x_5[j, k])
                m.addConstr((yplus_53[j, k] - 1) * M <= 89.83476081 - x_5[j, k])
                m.addConstr(y_35[i, k] + yplus_35[i, k] + y_53[j, k] + yplus_53[j, k] <= 3)

    for i in range(N_P3):
        for j in range(N_P8):
            for k in range(TH):
                m.addConstr(y_38[i, k]*M >= x_3[i, k] - 78.314117135)
                m.addConstr((y_38[i, k] - 1) * M <= x_3[i, k] - 78.314117135)
                m.addConstr(yplus_38[i, k] * M >=  89.83476081  - x_3[i, k])
                m.addConstr((yplus_38[i, k] - 1) * M <= 89.83476081  - x_3[i, k])
                m.addConstr(y_83[j, k]*M >= x_8[j, k] - 76.83755342)
                m.addConstr((y_83[j, k] - 1) * M <= x_8[j, k] - 76.83755342)
                m.addConstr(yplus_83[j, k] * M >= 87.16244658 - x_8[j, k])
                m.addConstr((yplus_83[j, k] - 1) * M <= 87.16244658 - x_8[j, k])
                m.addConstr(y_38[i, k] + yplus_38[i, k] + y_83[j, k] + yplus_83[j, k] <= 3)

    for i in range(N_P4):
        for j in range(N_P7):
            for k in range(TH):
                m.addConstr(y_47[i, k]*M >= x_4[i, k] - 76.83755342)
                m.addConstr((y_47[i, k] - 1) * M <= x_4[i, k] - 76.83755342)
                m.addConstr(yplus_47[i, k] * M >=  87.16244658  - x_4[i, k])
                m.addConstr((yplus_47[i, k] - 1) * M <= 87.16244658  - x_4[i, k])
                m.addConstr(y_74[j, k]*M >= x_7[j, k] - 78.314117135)
                m.addConstr((y_74[j, k] - 1) * M <= x_7[j, k] - 78.314117135)
                m.addConstr(yplus_74[j, k] * M >= 89.83476081 - x_7[j, k])
                m.addConstr((yplus_74[j, k] - 1) * M <= 89.83476081 - x_7[j, k])
                m.addConstr(y_47[i, k] + yplus_47[i, k] + y_74[j, k] + yplus_74[j, k] <= 3)

    for i in range(N_P4):
        for j in range(N_P5):
            for k in range(TH):
                m.addConstr(y_45[i, k]*M >= x_4[i, k] - 77.599342077)
                m.addConstr((y_45[i, k] - 1) * M <= x_4[i, k] - 77.599342077)
                m.addConstr(yplus_45[i, k] * M >=  87.1624  - x_4[i, k])
                m.addConstr((yplus_45[i, k] - 1) * M <= 87.1624  - x_4[i, k])
                m.addConstr(y_54[j, k]*M >= x_5[j, k] - 72.633276272)
                m.addConstr((y_54[j, k] - 1) * M <= x_5[j, k] - 72.633276272)
                m.addConstr(yplus_54[j, k] * M >= 82.27614523 - x_5[j, k])
                m.addConstr((yplus_54[j, k] - 1) * M <= 82.27614523 - x_5[j, k])
                m.addConstr(y_45[i, k] + yplus_45[i, k] + y_54[j, k] + yplus_54[j, k] <= 3)

    for i in range(N_P4):
        for j in range(N_P6):
            for k in range(TH):
                m.addConstr(y_46[i, k]*M >= x_4[i, k] - 83.5)
                m.addConstr((y_46[i, k] - 1) * M <= x_4[i, k] - 83.5)
                m.addConstr(yplus_46[i, k] * M >=  91  - x_4[i, k])
                m.addConstr((yplus_46[i, k] - 1) * M <= 91  - x_4[i, k])
                m.addConstr(y_64[j, k]*M >= x_6[j, k] - 73)
                m.addConstr((y_64[j, k] - 1) * M <= x_6[j, k] - 73)
                m.addConstr(yplus_64[j, k] * M >= 80.5 - x_6[j, k])
                m.addConstr((yplus_64[j, k] - 1) * M <= 80.5 - x_6[j, k])
                m.addConstr(y_46[i, k] + yplus_46[i, k] + y_64[j, k] + yplus_64[j, k] <= 3)

    for i in range(N_P6):
        for j in range(N_P7):
            for k in range(TH):
                m.addConstr(y_67[i, k]*M >= x_6[i, k] - 77.599342077)
                m.addConstr((y_67[i, k] - 1) * M <= x_6[i, k] - 77.599342077)
                m.addConstr(yplus_67[i, k] * M >=  87.1624  - x_6[i, k])
                m.addConstr((yplus_67[i, k] - 1) * M <= 87.1624  - x_6[i, k])
                m.addConstr(y_76[j, k]*M >= x_7[j, k] - 72.633276272)
                m.addConstr((y_76[j, k] - 1) * M <= x_7[j, k] - 72.633276272)
                m.addConstr(yplus_76[j, k] * M >= 82.27616723 - x_7[j, k])
                m.addConstr((yplus_76[j, k] - 1) * M <= 82.27616723 - x_7[j, k])
                m.addConstr(y_67[i, k] + yplus_67[i, k] + y_76[j, k] + yplus_76[j, k] <= 3)

    for i in range(N_P6):
        for j in range(N_P8):
            for k in range(TH):
                m.addConstr(y_68[i, k]*M >= x_6[i, k] - 83.5)
                m.addConstr((y_68[i, k] - 1) * M <= x_6[i, k] - 83.5)
                m.addConstr(yplus_68[i, k] * M >=  91  - x_6[i, k])
                m.addConstr((yplus_68[i, k] - 1) * M <= 91  - x_6[i, k])
                m.addConstr(y_86[j, k]*M >= x_8[j, k] - 73)
                m.addConstr((y_86[j, k] - 1) * M <= x_8[j, k] - 73)
                m.addConstr(yplus_86[j, k] * M >= 80.5 - x_8[j, k])
                m.addConstr((yplus_86[j, k] - 1) * M <= 80.5 - x_8[j, k])
                m.addConstr(y_68[i, k] + yplus_68[i, k] + y_86[j, k] + yplus_86[j, k] <= 3)

    for i in range(N_P7):
        for j in range(N_P5):
            for k in range(TH):
                m.addConstr(y_75[i, k] * M >= x_7[i, k] - 79.801524854)
                m.addConstr((y_75[i, k] - 1) * M <= x_7[i, k] - 79.801524854)
                m.addConstr(yplus_75[i, k] * M >= 89.83476081 - x_7[i, k])
                m.addConstr((yplus_75[i, k] - 1) * M <= 89.83476081 - x_7[i, k])
                m.addConstr(y_57[j, k] * M >= x_5[j, k] - 70)
                m.addConstr((y_57[j, k] - 1) * M <= x_5[j, k] - 70)
                m.addConstr(yplus_57[j, k] * M >= 82.6749032 - x_5[j, k])
                m.addConstr((yplus_57[j, k] - 1) * M <= 82.6749032 - x_5[j, k])
                m.addConstr(y_75[i, k] + yplus_75[i, k] + y_57[j, k] + yplus_57[j, k] <= 3)
    m.setObjectiveN(dt_1[0] , index= 0, weight = 5, priority = 5)
    m.setObjectiveN(dt_2[0] , index= 1, weight = 1, priority = 5)
    m.setObjectiveN(dt_1[1],  index=2,  weight = 1, priority = 5)
    m.setObjectiveN(dt_3[0] , index= 3, weight = 1, priority = 5)
    m.setObjectiveN(dt_2[1], index=4, weight=1, priority=5)
    m.setObjectiveN(dt_4[0] , index= 5, weight = 1, priority = 5)
    m.setObjectiveN(dt_3[1], index=6, weight=1, priority=5)
    m.setObjectiveN(dt_1[2], index=7, weight=1, priority=5)
    m.setObjectiveN(dt_5[0] , index= 8, weight = 1, priority = 5)
    m.setObjectiveN(dt_4[1], index=9, weight=1, priority=5)
    m.setObjectiveN(dt_2[2], index=10, weight=1, priority=5)
    m.setObjectiveN(dt_6[0] , index= 11, weight = 1, priority = 5)
    m.setObjectiveN(dt_5[1], index=12, weight=1, priority=5)
    m.setObjectiveN(dt_3[2], index=13, weight=1, priority=5)
    m.setObjectiveN(dt_1[3], index=14, weight=5, priority=5)
    m.setObjectiveN(dt_7[0] , index= 15, weight = 1, priority = 5)
    m.setObjectiveN(dt_6[1], index=16, weight=1, priority=5)
    m.setObjectiveN(dt_4[2], index=17, weight=1, priority=5)
    m.setObjectiveN(dt_2[3], index=18, weight=5, priority=5)
    m.setObjectiveN(dt_8[0] , index= 19, weight = 1, priority = 5)
    m.setObjectiveN(dt_7[1], index=20, weight=1, priority=5)
    m.setObjectiveN(dt_5[2], index=21, weight=1, priority=5)
    m.setObjectiveN(dt_3[3], index=22, weight=5, priority=5)
    m.setObjectiveN(dt_1[4], index=23, weight=5, priority=5)
    m.setObjectiveN(dt_8[1], index=24, weight=1, priority=5)
    m.setObjectiveN(dt_6[2], index=25, weight=1, priority=5)
    m.setObjectiveN(dt_4[3], index=26, weight=5, priority=5)
    m.setObjectiveN(dt_2[4], index=27, weight=5, priority=5)
    m.setObjectiveN(dt_7[2], index=28, weight=1, priority=5)
    m.setObjectiveN(dt_5[3], index=29, weight=5, priority=5)
    m.setObjectiveN(dt_3[4], index=30, weight=5, priority=5)
    m.setObjectiveN(dt_1[5], index=31, weight=5, priority=5)
    m.setObjectiveN(dt_8[2], index=32, weight=1, priority=5)
    m.setObjectiveN(dt_6[3], index=33, weight=5, priority=5)
    m.setObjectiveN(dt_4[4], index=34, weight=5, priority=5)
    m.setObjectiveN(dt_2[5], index=35, weight=5, priority=5)
    m.setObjectiveN(dt_7[3], index=36, weight=5, priority=5)
    m.setObjectiveN(dt_5[4], index=37, weight=5, priority=5)
    m.setObjectiveN(dt_3[5], index=38, weight=5, priority=5)
    m.setObjectiveN(dt_1[6], index=39, weight=5, priority=5)
    m.setObjectiveN(dt_8[3], index=40, weight=5, priority=5)
    m.setObjectiveN(dt_6[4], index=41, weight=5, priority=5)
    m.setObjectiveN(dt_4[5], index=42, weight=5, priority=5)
    m.setObjectiveN(dt_2[6], index=43, weight=5, priority=5)
    m.setObjectiveN(dt_7[4], index=44, weight=5, priority=5)
    m.setObjectiveN(dt_5[5], index=45, weight=5, priority=5)
    m.setObjectiveN(dt_3[6], index=46, weight=5, priority=5)
    m.setObjectiveN(dt_1[7], index=47, weight=5, priority=5)
    m.setObjectiveN(dt_8[4], index=48, weight=5, priority=5)
    m.setObjectiveN(dt_6[5], index=49, weight=5, priority=5)
    m.setObjectiveN(dt_4[6], index=50, weight=5, priority=5)
    m.setObjectiveN(dt_2[7], index=51, weight=5, priority=5)
    m.setObjectiveN(dt_7[5], index=52, weight=5, priority=5)
    m.setObjectiveN(dt_5[6], index=53, weight=5, priority=5)
    m.setObjectiveN(dt_3[7], index=54, weight=5, priority=5)
    m.setObjectiveN(dt_1[8], index=55, weight=5, priority=5)
    m.setObjectiveN(dt_8[5], index=56, weight=5, priority=5)
    m.setObjectiveN(dt_6[6], index=57, weight=5, priority=5)
    m.setObjectiveN(dt_4[7], index=58, weight=5, priority=5)
    m.setObjectiveN(dt_2[8], index=59, weight=5, priority=5)
    m.setObjectiveN(dt_7[6], index=60, weight=5, priority=5)
    m.setObjectiveN(dt_5[7], index=61, weight=5, priority=5)
    m.setObjectiveN(dt_3[8], index=62, weight=5, priority=5)
    m.setObjectiveN(dt_1[9], index=63, weight=5, priority=5)
    m.setObjectiveN(dt_8[6], index=64, weight=5, priority=5)
    m.setObjectiveN(dt_6[7], index=65, weight=5, priority=5)
    m.setObjectiveN(dt_4[8], index=66, weight=5, priority=5)
    m.setObjectiveN(dt_2[9], index=67, weight=5, priority=5)
    m.setObjectiveN(dt_7[7], index=68, weight=5, priority=5)
    m.setObjectiveN(dt_5[8], index=69, weight=5, priority=5)
    m.setObjectiveN(dt_3[9], index=70, weight=5, priority=5)
    m.setObjectiveN(dt_1[10], index=71, weight=5, priority=5)
    m.setObjectiveN(dt_8[7], index=72, weight=5, priority=5)
    m.setObjectiveN(dt_6[8], index=73, weight=5, priority=5)
    m.setObjectiveN(dt_4[9], index=74, weight=5, priority=5)
    m.setObjectiveN(dt_2[10], index=75, weight=5, priority=5)
    m.setObjectiveN(dt_7[8], index=76, weight=5, priority=5)
    m.setObjectiveN(dt_5[9], index=77, weight=5, priority=5)
    m.setObjectiveN(dt_3[10], index=78, weight=5, priority=5)
    m.setObjectiveN(dt_1[11], index=79, weight=5, priority=5)
    m.setObjectiveN(dt_8[8], index=80, weight=5, priority=5)
    m.setObjectiveN(dt_6[9], index=81, weight=5, priority=5)
    m.setObjectiveN(dt_4[10], index=82, weight=5, priority=5)
    m.setObjectiveN(dt_2[11], index=83, weight=5, priority=5)
    m.setObjectiveN(dt_7[9], index=84, weight=5, priority=5)
    m.setObjectiveN(dt_5[10], index=85, weight=5, priority=5)
    m.setObjectiveN(dt_3[11], index=86, weight=5, priority=5)
    m.setObjectiveN(dt_8[9], index=87, weight=5, priority=5)
    m.setObjectiveN(dt_6[10], index=88, weight=5, priority=5)
    m.setObjectiveN(dt_4[11], index=89, weight=5, priority=5)
    m.setObjectiveN(dt_7[10], index=90, weight=5, priority=5)
    m.setObjectiveN(dt_5[11], index=91, weight=5, priority=5)
    m.setObjectiveN(dt_8[10], index=92, weight=5, priority=5)
    m.setObjectiveN(dt_6[11], index=93, weight=5, priority=5)
    m.setObjectiveN(dt_7[11], index=94, weight=5, priority=5)
    m.setObjectiveN(dt_8[11], index=95, weight=5, priority=5)
    m.update()
    m.optimize()
    total =0.0
    for i in range(N_P1 + N_P2 + N_P3 + N_P4 + N_P5 + N_P6 + N_P7 + N_P8):
        m.setParam(gurobipy.GRB.Param.ObjNumber, i)
        total+=m.ObjNVal
        print(m.ObjNVal)
    print(total-sum(et_1 + et_2 + et_3 + et_4 + et_5 + et_6 + et_7 + et_8))
    #for v in m.getVars():
        #print('%s %g' % (v.varName, v.x))

except GurobiError as e:
    print('Error code ' + str(e.errno) + ": " + str(e))

except AttributeError:
    # print('Obj: %g' % m.ObjVal)
    print('Encountered an attribute error')


```
