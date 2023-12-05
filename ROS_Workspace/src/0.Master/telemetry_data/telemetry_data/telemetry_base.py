import customtkinter  as ctk
import time
import tkinter as tk
from math import pi

'''
-> Velocity         / *
-> Acceleration     / *
-> Lap              / *
-> Status           / *
-> Torque           / *
-> Steering         / *
-> Brake Pressure   / *
-> Node Error       / *
-> Ellipses???      / *6,3,1
'''



class TelemetryApp(ctk.CTk):
    def __init__(self) -> None:
        super().__init__()
        
        self.title("Telemetry - Data")
        # height = self.winfo_screenheight()
        # width = self.winfo_screenwidth()
        # self.geometry("%dx%d"%(width, height))
        self.configure(fg_color = "gray10")
        
        
        self.grid_rowconfigure(0, weight = 1, uniform='c')
        for i in range(0,3):
            self.grid_columnconfigure(i, weight = 1, uniform = "c")
        for j in range(1,5):
            self.grid_rowconfigure(j, weight = 3, uniform = "c")
            
        self.state_ = StateFrame(self)
        self.state_.grid(row = 0, column = 0, sticky = "news", padx = 3, pady = 3, columnspan = 3)
        self.velocity = VelocityFrame(self)
        self.velocity.grid(row = 1, column = 0, sticky = "news", padx = 3, pady = 3)
        self.accel = AccelFrame(self)
        self.accel.grid(row = 1, column = 1, sticky = "news", padx = 3, pady = 3)
        self.lap = LapFrame(self)
        self.lap.grid(row = 1, column = 2, sticky = "news", padx = 3, pady = 3)
        self.torque = TorqueFrame(self, -186.2, +186.2)
        self.torque.grid(row = 2, column = 0, sticky = "news", padx = 3, pady = 3)
        self.steer = SteerFrame(self, -31.2, +31.2)
        self.steer.grid(row = 2, column = 1, sticky = "news", padx = 3, pady = 3)
        self.brake = BrakeFrame(self)
        self.brake.grid(row = 2, column = 2, sticky = "news", padx = 3, pady = 3)
        self.error = ErrorFrame(self)
        self.error.grid(row = 3, column = 0, sticky = "news", padx = 3, pady = 3, rowspan = 2)
        self.ell_fl = EllipseFrame(self, 1, "FL")
        self.ell_fl.grid(row = 3, column = 1, sticky = "news", padx = 3, pady = 3)
        self.ell_fr = EllipseFrame(self, 0, "FR")
        self.ell_fr.grid(row = 3, column = 2, sticky = "news", padx = 3, pady = 3)
        self.ell_rl = EllipseFrame(self, 1, "RL")
        self.ell_rl.grid(row = 4, column = 1, sticky = "news", padx = 3, pady = 3)
        self.ell_rr = EllipseFrame(self, 0, "RR")
        self.ell_rr.grid(row = 4, column = 2, sticky = "news", padx = 3, pady = 3)
       
class VelocityFrame(ctk.CTkFrame):
    def __init__(self, master) -> None:
        super().__init__(master)
        self.configure(bg_color = "transparent")
        self.configure(fg_color = "gray20")
        
        self.grid_rowconfigure(0, weight = 1, uniform = "c")
        self.grid_rowconfigure(1, weight = 1, uniform = "c")
        self.grid_rowconfigure(2, weight = 1, uniform = "c")
        self.grid_rowconfigure(3, weight = 1, uniform = "c")
        
        self.grid_columnconfigure(0, weight = 1, uniform = 'c')
        self.grid_columnconfigure(1, weight = 2, uniform = 'c')
                
        self.speedbar = ctk.CTkProgressBar(self, width = 30, height = 120, corner_radius = 0, border_width = 1, border_color = 'black', fg_color = "gray39", progress_color = "SeaGreen3", orientation = "vertical")
        self.speedbar.grid(row = 0, column = 0, rowspan = 4, sticky = "ns", pady = 10)
        self.speedbar.set(0.0)
        
        self.actual_l = ctk.CTkLabel(self, text = "Actual", text_color = "gray85", font = ("CTkFont", 10), fg_color = "gray30", bg_color="transparent", corner_radius = 10)
        self.actual_l.grid(row = 0, column = 1, padx = 5, pady = 5)
        self.actual_v = ctk.CTkLabel(self, text = "0 m/s", text_color = "white", font = ("CTkFont", 30))
        self.actual_v.grid(row = 1, column = 1, sticky = "nswe", padx = 5, pady = 5)
        
        self.target_v = ctk.CTkLabel(self, text = "0 m/s", text_color = "white", font = ("CTkFont", 20))
        self.target_v.grid(row = 2, column = 1, sticky = "nswe", padx = 5, pady = 5)
        self.target_l = ctk.CTkLabel(self, text = "Target", text_color = "gray85", font = ("CTkFont", 10), fg_color = "gray30", bg_color="transparent", corner_radius = 10)
        self.target_l.grid(row = 3, column = 1, padx = 5, pady = 5)
        
        self.target = 0
        self.actual = 0
        
        
    def set_target(self, target) -> None:
        if target!=self.target:
            self.target = target
            self.target_v.configure(text = "%.2f m/s"%target)
        
    def set_actual(self, speed) -> None:
        if speed != self.actual:
            self.actual = speed
            self.actual_v.configure(text = "%.2f m/s"%speed)
            self.speedbar.set(speed / 10)
            if speed>10: 
                self.actual_v.configure(text_color = "red")
                self.speedbar.configure(progress_color = "red")

class AccelFrame(ctk.CTkFrame):
    def __init__(self, master) -> None:
        super().__init__(master)
        
        self.configure(bg_color = "transparent")
        self.configure(fg_color = "gray20")
        
        self.grid_rowconfigure(0, weight = 1, uniform = 'c')
        self.grid_rowconfigure(1, weight = 3, uniform = 'c')
        
        self.grid_columnconfigure(0, weight = 3, uniform = 'c')
        self.grid_columnconfigure(1, weight = 2, uniform = 'c')
        
        a = 110
        
        self.canvas = tk.Canvas(self, background="gray20", bd = 0, height= a, width = a, highlightthickness = 0)
        self.canvas.grid(row = 1, column = 0, padx = 5, pady = 5, sticky = "")
        
        self.ax = ctk.CTkLabel(self, text = "+0.0 g", text_color = "gray85", font = ("CTkFont", 15), fg_color = "gray30", bg_color="transparent", corner_radius = 5)
        self.ax.grid(row = 0, column = 0, padx = 5, pady = 5)
        
        self.ay = ctk.CTkLabel(self, text = "+0.0 g", text_color = "gray85", font = ("CTkFont", 15), fg_color = "gray30", bg_color="transparent", corner_radius = 5)
        self.ay.grid(row = 1, column = 1, padx = 5, pady = 5)
        
        coord3 = 0, 0, a, a
        self.l3_e = self.canvas.create_arc(coord3, start = -45, extent = 90, fill = "gray30")
        self.l3_n = self.canvas.create_arc(coord3, start = 45, extent = 90, fill = "gray30")
        self.l3_w = self.canvas.create_arc(coord3, start = 135, extent = 90, fill = "gray30")
        self.l3_s = self.canvas.create_arc(coord3, start = 225, extent = 90, fill = "gray30")
        
        coord2 = a/8, a/8, 7*a/8, 7*a/8
        self.l2_e = self.canvas.create_arc(coord2, start = -45, extent = 90, fill = "gray30")
        self.l2_n = self.canvas.create_arc(coord2, start = 45, extent = 90, fill = "gray30")
        self.l2_w = self.canvas.create_arc(coord2, start = 135, extent = 90, fill = "gray30")
        self.l2_s = self.canvas.create_arc(coord2, start = 225, extent = 90, fill = "gray30")
        
        coord1 = a/4, a/4, 3*a/4, 3*a/4
        self.l1_e = self.canvas.create_arc(coord1, start = -45, extent = 90, fill = "gray30")
        self.l1_n = self.canvas.create_arc(coord1, start = 45, extent = 90, fill = "gray30")
        self.l1_w = self.canvas.create_arc(coord1, start = 135, extent = 90, fill = "gray30")
        self.l1_s = self.canvas.create_arc(coord1, start = 225, extent = 90, fill = "gray30")
        
        coord0 = 3*a/8, 3*a/8, 5*a/8, 5*a/8
        self.l0 = self.canvas.create_oval(coord0, fill = "red")
        
        self.a_x = 0
        self.a_y = 0
        
    def update_ax(self, a_x) -> None:
        if a_x != self.a_x:
            self.a_x = a_x
            ag = a_x / 9.81
            
            self.canvas.itemconfigure(self.l1_n, fill = "gray30")
            self.canvas.itemconfigure(self.l2_n, fill = "gray30")
            self.canvas.itemconfigure(self.l3_n, fill = "gray30")
            self.canvas.itemconfigure(self.l1_s, fill = "gray30")
            self.canvas.itemconfigure(self.l2_s, fill = "gray30")
            self.canvas.itemconfigure(self.l3_s, fill = "gray30")
            
            if ag>0:
                self.ax.configure(text = "+%.2f g"%ag)
                if a_x>0.25:
                    self.canvas.itemconfigure(self.l1_n, fill = "red2")
                if a_x>1:
                    self.canvas.itemconfigure(self.l2_n, fill = "red3")
                if a_x>2:
                    self.canvas.itemconfigure(self.l3_n, fill = "red4")
                    
            else:
                self.ax.configure(text = "%.2f g"%ag)
                if a_x<-0.25:
                    self.canvas.itemconfigure(self.l1_s, fill = "red2")
                if a_x<-1:
                    self.canvas.itemconfigure(self.l2_s, fill = "red3")
                if a_x<-2:
                    self.canvas.itemconfigure(self.l3_s, fill = "red4")
        
    def update_ay(self, a_y) -> None:
        if a_y!=self.a_y:
            self.a_y = a_y
            ag = a_y / 9.81
            
            self.canvas.itemconfigure(self.l1_e, fill = "gray30")
            self.canvas.itemconfigure(self.l2_e, fill = "gray30")
            self.canvas.itemconfigure(self.l3_e, fill = "gray30")
            self.canvas.itemconfigure(self.l1_w, fill = "gray30")
            self.canvas.itemconfigure(self.l2_w, fill = "gray30")
            self.canvas.itemconfigure(self.l3_w, fill = "gray30")
            
            if ag>0:
                self.ay.configure(text = "+%.2f g"%ag)
                if a_y>0.25:
                    self.canvas.itemconfigure(self.l1_e, fill = "red2")
                if a_y>1:
                    self.canvas.itemconfigure(self.l2_e, fill = "red3")
                if a_y>2:
                    self.canvas.itemconfigure(self.l3_e, fill = "red4")
                    
            else:
                self.ay.configure(text = "%.2f g"%ag)
                if a_y<-0.25:
                    self.canvas.itemconfigure(self.l1_w, fill = "red2")
                if a_y<-1:
                    self.canvas.itemconfigure(self.l2_w, fill = "red3")
                if a_y<-2:
                    self.canvas.itemconfigure(self.l3_w, fill = "red4")

class LapFrame(ctk.CTkFrame):
    def __init__(self, master) -> None:
        super().__init__(master)
        
        self.configure(bg_color = "transparent")
        self.configure(fg_color = "gray20")
        
        self.grid_rowconfigure(0, weight = 1, uniform = 'c')
        self.grid_rowconfigure(1, weight = 1, uniform = 'c')
        
        self.grid_columnconfigure(0, weight = 1, uniform = 'c')
        self.grid_columnconfigure(1, weight = 1, uniform = 'c')
        
        self.lab = ctk.CTkLabel(self, text = "LAP", text_color = "white", font = ("CTkFont", 20), fg_color = "gray30", bg_color="transparent", corner_radius = 10)
        self.lab.grid(row = 0, column = 0, columnspan = 2, padx = 20, pady = 20, sticky = "s")
        
        self.lap = 0
        self.count = ctk.CTkLabel(self, text = "0", text_color = "white", font = ("CTkFont", 50), fg_color = "transparent", bg_color="transparent", corner_radius = 10)
        self.count.grid(row = 1, column = 0, columnspan = 1, padx = 5, pady = 5, sticky = "e")
        
        self.lap_count = -1
        self.total = ctk.CTkLabel(self, text = "/0", text_color = "white", font = ("CTkFont", 25), fg_color = "transparent", bg_color="transparent", corner_radius = 10)
        self.total.grid(row = 1, column = 1, columnspan = 1, padx = 5, pady = 5, sticky = "w")

        self.set_max(2)
        
    def set_max(self, lap_count) -> None:
        self.lap_count = lap_count - 1
        self.total.configure(text = "/ "+str(lap_count - 1))
        
    def set_lap(self, lap) -> None:
        if self.lap != lap:
            self.lap = lap
            self.count.configure(text = str(lap))
            if lap==self.lap_count:
                self.count.configure(text_color = "green4")
            elif lap==self.lap_count+1:
                self.count.configure(text_color = "navy")
            elif lap>self.lap_count+1:
                self.count.configure(text_color = "red")
    
class TorqueFrame(ctk.CTkFrame):
    def __init__(self, master, mn, mx) -> None:
        super().__init__(master)
        
        self.mn = mn
        self.mx = mx
        
        self.configure(bg_color = "transparent")
        self.configure(fg_color = "gray20")
        
        self.grid_rowconfigure(0, weight = 1, uniform = "c")
        self.grid_rowconfigure(1, weight = 1, uniform = "c")
        self.grid_rowconfigure(2, weight = 1, uniform = "c")
        self.grid_rowconfigure(3, weight = 1, uniform = "c")
        
        self.grid_columnconfigure(0, weight = 1, uniform = 'c')
        self.grid_columnconfigure(1, weight = 4, uniform = 'c')
                
        self.bar_acc = ctk.CTkProgressBar(self, width = 20, height = 60, corner_radius = 0, border_width = 1, border_color = 'gray40', fg_color = "gray40", progress_color = "green2", orientation = "vertical")
        self.bar_acc.grid(row = 0, column = 0, rowspan = 2, sticky = "ns", pady = (10,0))
        self.bar_acc.set(0.0)
        self.bar_dec = ctk.CTkProgressBar(self, width = 20, height = 60, corner_radius = 0, border_width = 1, border_color = 'gray40', fg_color = "red2", progress_color = "gray40", orientation = "vertical")
        self.bar_dec.grid(row = 2, column = 0, rowspan = 2, sticky = "sn", pady = (0,10))
        self.bar_dec.set(1.0)
        
        self.actual_l = ctk.CTkLabel(self, text = "Actual", text_color = "gray85", font = ("CTkFont", 10), fg_color = "gray30", bg_color="transparent", corner_radius = 10)
        self.actual_l.grid(row = 0, column = 1, padx = 5, pady = 5)
        self.actual_v = ctk.CTkLabel(self, text = "0 Nm", text_color = "white", font = ("CTkFont", 30))
        self.actual_v.grid(row = 1, column = 1, sticky = "nswe", padx = 5, pady = 5)
        
        self.target_v = ctk.CTkLabel(self, text = "0 Nm", text_color = "white", font = ("CTkFont", 20))
        self.target_v.grid(row = 2, column = 1, sticky = "nswe", padx = 5, pady = 5)
        self.target_l = ctk.CTkLabel(self, text = "Target", text_color = "gray85", font = ("CTkFont", 10), fg_color = "gray30", bg_color="transparent", corner_radius = 10)
        self.target_l.grid(row = 3, column = 1, padx = 5, pady = 5)
        
        self.target = 0
        self.actual = 0
        
    def set_target(self, target) -> None:
        if target!=self.target:
            self.target = target
            self.target_v.configure(text = "%.2f Nm"%target)
        
    def set_actual(self, trq) -> None:
        if trq!=self.actual:
            self.actual = trq
            self.actual_v.configure(text = "%.2f Nm"%trq)
            if trq>0:
                self.bar_acc.set(trq/self.mx)
                self.bar_dec.set(1)
            else:
                self.bar_dec.set(1-trq/self.mn)
                self.bar_acc.set(0)
            
class SteerFrame(ctk.CTkFrame):
    def __init__(self, master, mn, mx) -> None:
        super().__init__(master)
        
        self.mn = mn
        self.mx = mx
        
        self.configure(bg_color = "transparent")
        self.configure(fg_color = "gray20")
        
        self.grid_rowconfigure(0, weight = 1, uniform = "c")
        self.grid_rowconfigure(1, weight = 1, uniform = "c")
        self.grid_rowconfigure(2, weight = 1, uniform = "c")
        
        self.grid_columnconfigure(0, weight = 1, uniform = "c")
        self.grid_columnconfigure(1, weight = 1, uniform = "c")
                
        self.bar_acc = ctk.CTkProgressBar(self, width = 60, height = 20, corner_radius = 0, border_width = 1, border_color = 'gray40', fg_color = "gold3", progress_color = "gray40", orientation = "horizontal")
        self.bar_acc.grid(column = 0, row = 0, columnspan = 1, sticky = "we", padx = (20, 0))
        self.bar_acc.set(1.0)
        self.bar_dec = ctk.CTkProgressBar(self, width = 60, height = 20, corner_radius = 0, border_width = 1, border_color = 'gray40', fg_color = "gray40", progress_color = "gold3", orientation = "horizontal")
        self.bar_dec.grid(column = 1, row = 0, columnspan = 2, sticky = "we", padx = (0, 20))
        self.bar_dec.set(0.0)
        
        self.actual_l = ctk.CTkLabel(self, text = "Actual", text_color = "gray85", font = ("CTkFont", 10), fg_color = "gray30", bg_color="transparent", corner_radius = 5)
        self.actual_l.grid(column = 0, row = 2, padx = 5, pady = 5)
        self.actual_v = ctk.CTkLabel(self, text = "0°", text_color = "white", font = ("CTkFont", 30))
        self.actual_v.grid(column = 0, row = 1, sticky = "nswe", padx = 5, pady = 5)
        
        self.target_v = ctk.CTkLabel(self, text = "0°", text_color = "white", font = ("CTkFont", 20))
        self.target_v.grid(column = 1, row = 1, sticky = "nswe", padx = 5, pady = 5)
        self.target_l = ctk.CTkLabel(self, text = "Target", text_color = "gray85", font = ("CTkFont", 10), fg_color = "gray30", bg_color="transparent", corner_radius = 5)
        self.target_l.grid(column = 1, row = 2, padx = 5, pady = 5)
        
        self.target = 0
        self.actual = 0
        
    def set_target(self, target) -> None:
        if target!=self.target:
            self.target = target
            target = target 
            self.target_v.configure(text = "%.2f°"%target)
        
    def set_actual(self, steer) -> None:
        if steer!=self.actual:
            self.actual = steer
            steer = steer 
            self.actual_v.configure(text = "%.2f°"%steer)
            if steer>0:
                self.bar_dec.set(steer/self.mx)
                self.bar_acc.set(1)
            else:
                self.bar_acc.set(1-steer/self.mn)
                self.bar_dec.set(0)            
        
class BrakeFrame(ctk.CTkFrame):
    def __init__(self, master) -> None:
        super().__init__(master)
        self.configure(bg_color = "transparent")
        self.configure(fg_color = "gray20")
        
        self.grid_columnconfigure(0, weight = 2, uniform = "c")
        self.grid_columnconfigure(1, weight = 2, uniform = "c")
        self.grid_columnconfigure(2, weight = 2, uniform = "c")
        
        self.grid_rowconfigure(0, weight = 1, uniform = "c")
        self.grid_rowconfigure(1, weight = 2, uniform = "c")
        self.grid_rowconfigure(2, weight = 1, uniform = "c")
        self.grid_rowconfigure(3, weight = 2, uniform = "c")
        
        self.last_target = 20
        self.target = 0
        self.target_l = ctk.CTkLabel(self, text = "Target", text_color = "gray85", font = ("CTkFont", 10), fg_color = "gray30", bg_color="transparent", corner_radius = 5)
        self.target_l.grid(row = 0, column = 0, padx = 0, pady = 5, rowspan = 2)
        
        self.target_v = ctk.CTkLabel(self, text = "0", text_color = "white", font = ("CTkFont", 30))
        self.target_v.grid(row = 2, column = 0, sticky = "n", padx = 0, pady = 5, rowspan = 2)
        
        self.actual_lf = ctk.CTkLabel(self, text = "Front", text_color = "gray85", font = ("CTkFont", 10), fg_color = "gray30", bg_color="transparent", corner_radius = 5)
        self.actual_lf.grid(row = 0, column = 1, padx = 0, pady = 5)

        self.actual_vf = ctk.CTkLabel(self, text = "0", text_color = "white", font = ("CTkFont", 20))
        self.actual_vf.grid(row = 1, column = 1, sticky = "nswe", padx = 0, pady = 5)
        
        self.actual_lr = ctk.CTkLabel(self, text = "Rear", text_color = "gray85", font = ("CTkFont", 10), fg_color = "gray30", bg_color="transparent", corner_radius = 5)
        self.actual_lr.grid(row = 2, column = 1, padx = 0, pady = 5)

        self.actual_vr = ctk.CTkLabel(self, text = "0", text_color = "white", font = ("CTkFont", 20))
        self.actual_vr.grid(row = 3, column = 1, sticky = "nswe", padx = 0, pady = 5)
        
        self.bar_front = ctk.CTkProgressBar(self, width = 30, height = 60, corner_radius = 0, bg_color = "transparent", fg_color = "gray10", progress_color = "red", orientation = "vertical")
        self.bar_front.grid(row = 0, column = 2, rowspan = 2, sticky = "ns", pady = 5)
        self.bar_front.set(0)
        
        self.bar_rear = ctk.CTkProgressBar(self, width = 30, height = 60, corner_radius = 0, bg_color = "transparent", fg_color = "gray10", progress_color = "red", orientation = "vertical")
        self.bar_rear.grid(row = 2, column = 2, rowspan = 2, sticky = "ns", pady = 5)
        self.bar_rear.set(0)
        
        self.target = 0
        self.actual_f = 0
        self.actual_r = 0
        
    def set_target(self, t) -> None:
        if t!=self.target:
            self.target = t
            if t > 0:
                self.last_target = t
            self.target_v.configure(text = "%.1f"%t)
            if abs(t)>0.1:
                self.bar_front.configure(fg_color = "gray30")
                self.bar_rear.configure(fg_color = "gray30")
            else:
                self.bar_front.configure(fg_color = "gray10")
                self.bar_rear.configure(fg_color = "gray10")
        
    def set_front(self, p) -> None:
        if p!=self.actual_f:
            self.actual_f = p
            self.actual_vf.configure(text = "%.1f"%p)
            self.bar_front.set(p / self.last_target)
        
    def set_rear(self, p) -> None:
        if p!=self.actual_r:
            self.actual_r = p
            self.actual_vr.configure(text = "%.1f"%p)
            self.bar_rear.set(p / self.last_target)
        
    def set_both(self, f, r) -> None:
        self.set_front(f)
        self.set_rear(r)
        
class ErrorFrame(ctk.CTkFrame):
    def __init__(self, master) -> None:
        super().__init__(master)
        self.configure(bg_color = "transparent")
        self.configure(fg_color = "gray20")

        self.grid_columnconfigure(0, weight = 3, uniform = "c")
        self.grid_columnconfigure(1, weight = 1, uniform = "c")
        self.grid_columnconfigure(2, weight = 1, uniform = "c")
        
        self.grid_rowconfigure(0, weight = 1, uniform = "c")
        self.grid_rowconfigure(1, weight = 1, uniform = "c")
        self.grid_rowconfigure(2, weight = 1, uniform = "c")
        self.grid_rowconfigure(3, weight = 1, uniform = "c")
        self.grid_rowconfigure(4, weight = 1, uniform = "c")
        self.grid_rowconfigure(5, weight = 1, uniform = "c")
        self.grid_rowconfigure(6, weight = 1, uniform = "c")
        self.grid_rowconfigure(7, weight = 1, uniform = "c")
        self.grid_rowconfigure(8, weight = 1, uniform = "c")
        self.grid_rowconfigure(9, weight = 1, uniform = "c")
        self.grid_rowconfigure(10, weight = 1, uniform = "c")

        #   --------------------------
        #           Labels
        #   -------------------------
        self.inference = ctk.CTkLabel(self, text = "Inference", text_color = "white", bg_color = "transparent", fg_color = "transparent", font = ("CTkFont", 15), anchor = "w")
        self.inference.grid(row = 0, column = 0, sticky = "news", padx = 10)
        
        self.vel_est = ctk.CTkLabel(self, text = "Velocity Estimation", text_color = "white", bg_color = "transparent", fg_color = "transparent", font = ("CTkFont", 15), anchor = "w")
        self.vel_est.grid(row = 1, column = 0, sticky = "news", padx = 10)
        
        self.slam = ctk.CTkLabel(self, text = "SLAM", text_color = "white", bg_color = "transparent", fg_color = "transparent", font = ("CTkFont", 15), anchor = "w")
        self.slam.grid(row = 2, column = 0, sticky = "news", padx = 10)
        
        self.mpc = ctk.CTkLabel(self, text = "MPC", text_color = "white", bg_color = "transparent", fg_color = "transparent", font = ("CTkFont", 15), anchor = "w")
        self.mpc.grid(row = 3, column = 0, sticky = "news", padx = 10)
        
        self.pidpp = ctk.CTkLabel(self, text = "PID PP", text_color = "white", bg_color = "transparent", fg_color = "transparent", font = ("CTkFont", 15), anchor = "w")
        self.pidpp.grid(row = 4, column = 0, sticky = "news", padx = 10)
        
        self.pathp = ctk.CTkLabel(self, text = "Path Planning", text_color = "white", bg_color = "transparent", fg_color = "transparent", font = ("CTkFont", 15), anchor = "w")
        self.pathp.grid(row = 5, column = 0, sticky = "news", padx = 10)
        
        self.cam_l = ctk.CTkLabel(self, text = "Cam Left", text_color = "white", bg_color = "transparent", fg_color = "transparent", font = ("CTkFont", 15), anchor = "w")
        self.cam_l.grid(row = 6, column = 0, sticky = "news", padx = 10)
        
        self.cam_r = ctk.CTkLabel(self, text = "Cam Right", text_color = "white", bg_color = "transparent", fg_color = "transparent", font = ("CTkFont", 15), anchor = "w")
        self.cam_r.grid(row = 7, column = 0, sticky = "news", padx = 10)
        
        self.vn_200 = ctk.CTkLabel(self, text = "VN-200", text_color = "white", bg_color = "transparent", fg_color = "transparent", font = ("CTkFont", 15), anchor = "w")
        self.vn_200.grid(row = 8, column = 0, sticky = "news", padx = 10)
        
        self.vn_300 = ctk.CTkLabel(self, text = "VN-300", text_color = "white", bg_color = "transparent", fg_color = "transparent", font = ("CTkFont", 15), anchor = "w")
        self.vn_300.grid(row = 9, column = 0, sticky = "news", padx = 10)
        
        self.ins_mode = ctk.CTkLabel(self, text = "INS Mode", text_color = "white", bg_color = "transparent", fg_color = "transparent", font = ("CTkFont", 15), anchor = "w")
        self.ins_mode.grid(row = 10, column = 0, sticky = "news", padx = 10)
        
        #   --------------------------
        #           Errors
        #   -------------------------
        self.inference_status = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray30", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.inference_status.grid(row = 0, column = 1, padx = 10, pady = 5)
        
        self.vel_est_status = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray30", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.vel_est_status.grid(row = 1, column = 1, padx = 10, pady = 5)
        
        self.slam_status = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray30", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.slam_status.grid(row = 2, column = 1, padx = 10, pady = 5)
        
        self.mpc_status = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray30", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.mpc_status.grid(row = 3, column = 1, padx = 10, pady = 5)
        
        self.pidpp_status = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray30", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.pidpp_status.grid(row = 4, column = 1, padx = 10, pady = 5)
        
        self.pathp_status = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray30", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.pathp_status.grid(row = 5, column = 1, padx = 10, pady = 5)
        
        self.cam_l_status = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray30", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.cam_l_status.grid(row = 6, column = 1, padx = 10, pady = 5)
        
        self.cam_r_status = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray30", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.cam_r_status.grid(row = 7, column = 1, padx = 10, pady = 5)
        
        self.vn_200_status = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray30", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.vn_200_status.grid(row = 8, column = 1, padx = 10, pady = 5)
        
        self.vn_300_status = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray30", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.vn_300_status.grid(row = 9, column = 1, padx = 10, pady = 5)
        
        self.ins_mode_status = ctk.CTkLabel(self, text = "0", text_color = "white", bg_color = "transparent", fg_color = "gray30", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.ins_mode_status.grid(row = 10, column = 1, padx = 5, pady = 5)

        #   --------------------------
        #         Transitions
        #   -------------------------
        self.inference_trans = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray20", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.inference_trans.grid(row = 0, column = 2, padx = 10, pady = 5)
        
        self.vel_est_trans = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray20", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.vel_est_trans.grid(row = 1, column = 2, padx = 10, pady = 5)
        
        self.slam_trans = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray20", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.slam_trans.grid(row = 2, column = 2, padx = 10, pady = 5)
        
        self.mpc_trans = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray20", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.mpc_trans.grid(row = 3, column = 2, padx = 10, pady = 5)
        
        self.pidpp_trans = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray20", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.pidpp_trans.grid(row = 4, column = 2, padx = 10, pady = 5)
        
        self.pathp_trans = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray20", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.pathp_trans.grid(row = 5, column = 2, padx = 10, pady = 5)
        
        self.cam_l_trans = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray20", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.cam_l_trans.grid(row = 6, column = 2, padx = 10, pady = 5)
        
        self.cam_r_trans = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray20", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.cam_r_trans.grid(row = 7, column = 2, padx = 10, pady = 5)
        
        self.errors = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
        self.ins = -1
        self.id = 0
        self.status = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
       
    def update(self, error, ins_mode, trans_id, status) -> None:
        if error[0] != self.errors[0]:
            self.errors[0] = error[0]
            self.inference.configure(text_color = self.color_l(error[0]))
            self.inference_status.configure(fg_color = self.color(error[0]))
        
        if error[1] != self.errors[1]:
            self.errors[1] = error[1]    
            self.vel_est.configure(text_color = self.color_l(error[1]))
            self.vel_est_status.configure(fg_color = self.color(error[1]))
        
        if error[2] != self.errors[2]:
            self.errors[2] = error[2]      
            self.slam.configure(text_color = self.color_l(error[2]))
            self.slam_status.configure(fg_color = self.color(error[2]))
        
        if error[3] != self.errors[3]:
            self.errors[3] = error[3]  
            self.mpc.configure(text_color = self.color_l(error[3]))
            self.mpc_status.configure(fg_color = self.color(error[3]))
        
        if error[4] != self.errors[4]:
            self.errors[4] = error[4]  
            self.pidpp.configure(text_color = self.color_l(error[4]))
            self.pidpp_status.configure(fg_color = self.color(error[4]))
        
        if error[5] != self.errors[5]:
            self.errors[5] = error[5]  
            self.pathp.configure(text_color = self.color_l(error[5]))
            self.pathp_status.configure(fg_color = self.color(error[5]))
        
        if error[6] != self.errors[6]:
            self.errors[6] = error[6]  
            self.cam_l.configure(text_color = self.color_l(error[6]))
            self.cam_l_status.configure(fg_color = self.color(error[6]))
        
        if error[7] != self.errors[7]:
            self.errors[7] = error[7]  
            self.cam_r.configure(text_color = self.color_l(error[7]))
            self.cam_r_status.configure(fg_color = self.color(error[7]))
        
        if error[8] != self.errors[8]:
            self.errors[8] = error[8]  
            self.vn_200.configure(text_color = self.color_l(error[8]))
            self.vn_200_status.configure(fg_color = self.color(error[6]))
        
        if error[9] != self.errors[9]:
            self.errors[9] = error[9]  
            self.vn_300.configure(text_color = self.color_l(error[9]))
            self.vn_300_status.configure(fg_color = self.color(error[7]))
            
        if self.ins!=ins_mode:
            self.ins = ins_mode
            self.ins_mode_status.configure(text = str(ins_mode))

        else:
            for entry in [[self.inference_trans, self.inference, 2], 
                          [self.vel_est_trans, self.vel_est, 3], 
                          [self.slam_trans,self.slam, 4], 
                          [self.mpc_trans, self.mpc, 7], 
                          [self.pidpp_trans, self.pidpp, 6], 
                          [self.pathp_trans, self.pathp, 5], 
                          [self.cam_l_trans, self.cam_l, 1], 
                          [self.cam_r_trans, self.cam_r, 0]]:
                if self.status[entry[2]] != status[entry[2]]:
                    entry[0].configure(fg_color = self.color_t(status[entry[2]]))
                    self.status[entry[2]] = status[entry[2]]
        self.id = trans_id
        # else:
        #     if(trans_id not in [0, 10]):
        #         self.errors = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
        #         self.ins_mode = -1
            
        #     if(trans_id not in [0, 10, self.id]):
        #         for button in [self.inference_status, self.vel_est_status, self.slam_status, self.mpc_status, self.pidpp_status, self.pathp_status, self.cam_l_status, self.cam_r_status, self.vn_200_status, self.vn_300_status]:
        #             button.configure(fg_color = 'gray10')

        #     for entry in [[self.inference_status, self.inference, 2], 
        #                   [self.vel_est_status, self.vel_est, 3], 
        #                   [self.slam_status,self.slam, 4], 
        #                   [self.mpc_status, self.mpc, 7], 
        #                   [self.pidpp_status, self.pidpp, 6], 
        #                   [self.pathp_status, self.pathp, 5], 
        #                   [self.cam_l_status, self.cam_l, 1], 
        #                   [self.cam_r_status, self.cam_r, 0]]:
        #         if self.status[entry[2]] != status[entry[2]]:
        #             entry[0].configure(fg_color = self.color_t(status[entry[2]]))
        #             self.status[entry[2]] = status[entry[2]]
                
            

            
    def color(self, error) -> str:
        if error==0: return "green4"
        else: return "red3"
        
    def color_l(self, error) -> str:
        if error==0: return "white"
        else: return "red3"

    def color_t(self, status) -> str:
        if status==0: return "gray20"
        elif status==1: return "yellow"
        elif status==2: return "green4"
        else: return "red3"

class EllipseFrame(ctk.CTkFrame):
    def __init__(self, master, left, title) -> None:
        super().__init__(master)
        
        self.configure(bg_color = "transparent")
        self.configure(fg_color = "gray20")
        
        self.ell_col = 1 - left #if left=1 ell_col=0
        
        self.grid_rowconfigure(0, weight = 1, uniform = 'c')
        self.grid_rowconfigure(1, weight = 3, uniform = 'c')
        
        self.grid_columnconfigure(self.ell_col, weight = 3, uniform = 'c')
        self.grid_columnconfigure(1 - self.ell_col, weight = 1, uniform = 'c')
        
        self.a = 120
        
        self.canvas = tk.Canvas(self, background="gray20", bd = 0, height= self.a, width = self.a, highlightthickness = 0)
        self.canvas.grid(row = 0, rowspan = 2, column = self.ell_col, padx = 5, pady = 5, sticky = "")
        
        self.lab = ctk.CTkLabel(self, text = title, text_color = "white", bg_color = "transparent", fg_color = "transparent", font = ("CTkFont", 20))
        self.lab.grid(row = 0, column = 1-self.ell_col, sticky = "news", padx = 10, pady = 10)
        
        self.status = ctk.CTkLabel(self, text = "", text_color = "white", bg_color = "transparent", fg_color = "gray30", font = ("CTkFont", 15), anchor = "w", corner_radius = 20)
        self.status.grid(row = 1, column = 1-self.ell_col, padx = 10, pady = 5)
        
        self.xaxis = self.canvas.create_line(0, self.a/2, self.a, self.a/2, arrow = tk.LAST, width = 2)
        self.yaxis = self.canvas.create_line(self.a/2, self.a, self.a/2, 0, arrow = tk.LAST, width = 2)
        
        self.fxmax = 3500
        self.fymax = 3500
        self.ell = self.canvas.create_oval(0, 0, 0, 0, outline = "red", width  =2, fill = "")
        self.point = self.canvas.create_oval(0, 0, 0, 0, outline = "black", width  =1, fill = "blue")
        
        self.mx, self.my, self.fx, self.fy, self.fz = 0, 0, 0, 0, 0
        self.slip = False
                
    def update(self, fx, fy, fz, mx, my) -> None:
        if self.fz!=fz or self.my!=my or self.mx!=mx:
            self.fz, self.my, self.mx = fz, my, mx
            e = (self.fymax - my*fz)*(self.a/(2*self.fymax))
            d = (self.fxmax - mx*fz)*(self.a/(2*self.fxmax))
            c = (self.fxmax + mx*fz)*(self.a/(2*self.fxmax))
            f = (self.fymax + my*fz)*(self.a/(2*self.fymax))
            
            coords = d,e,c,f
            #print(mx, my, fz)
            #self.canvas.itemconfig(self.ell, __x0 = d, __y0 = e, __x1 = c, __y1 = f)
            self.canvas.coords(self.ell, d, e, c, f)
        
        if fx!=self.fx or fy!=self.fy:
            self.fx, self.fy = fx, fy
            x = (self.fxmax + fx)*(self.a/(2*self.fxmax))
            y = (self.fymax - fy)*(self.a/(2*self.fymax))
            
            #self.canvas.itemconfig(self.point, __x0 = x-1, __y0 = y-1, __x1 = x+1, __y1 = y+1)
            self.canvas.coords(self.point, x-3, y-3, x+3, y+3)
            
        slip = (fx/mx)**2 + (fy/my)**2 > fz**2
        if slip and not self.slip:
            self.slip = True
            self.status.configure(fg_color = "red")
        elif not slip and self.slip:
            self.slip = False
            self.status.configure(fg_color = "gray30")

class StateFrame(ctk.CTkFrame):
    def __init__(self, master) -> None:
        super().__init__(master)
        
        self.configure(bg_color = "transparent")
        self.configure(fg_color = "gray20")
        
        self.columnconfigure((0,1,2), weight = 1, uniform = "c")
        self.rowconfigure(0, weight = 2, uniform = 'c')
        self.rowconfigure(1, weight = 3, uniform = 'c')
        
        self.dv_l = ctk.CTkLabel(self, text = "DV Status", text_color = "gray85", font = ("CTkFont", 10), fg_color = "gray30", bg_color="transparent", corner_radius = 5)
        self.dv_l.grid(row = 0, column = 0, padx = 5, pady = 5)
        
        self.aut_l = ctk.CTkLabel(self, text = "Autonomous Status", text_color = "gray85", font = ("CTkFont", 10), fg_color = "gray30", bg_color="transparent", corner_radius = 5)
        self.aut_l.grid(row = 0, column = 1, padx = 5, pady = 5)
        
        self.miss_l = ctk.CTkLabel(self, text = "Mission", text_color = "gray85", font = ("CTkFont", 10), fg_color = "gray30", bg_color="transparent", corner_radius = 5)
        self.miss_l.grid(row = 0, column = 2, padx = 5, pady = 5)
        
        self.dv_v = ctk.CTkLabel(self, text = "STARTUP", text_color = "white", font = ("CTkFont", 20))
        self.dv_v.grid(row = 1, column = 0, sticky = "nswe", padx = 5, pady = 5)

        self.aut_v = ctk.CTkLabel(self, text = "AS OFF", text_color = "white", font = ("CTkFont", 20))
        self.aut_v.grid(row = 1, column = 1, sticky = "nswe", padx = 5, pady = 5)

        self.miss_v = ctk.CTkLabel(self, text = "UNLOCKED", text_color = "white", font = ("CTkFont", 20))
        self.miss_v.grid(row = 1, column = 2, sticky = "nswe", padx = 5, pady = 5)
        
        self.dv, self.aut, self.miss = 0, 0, 0
        
    def change_dv(self, state) -> None:
        if state==self.dv:
            return
        self.dv = state
        
        if state==0:
            self.dv_v.configure(text = "STARTUP", text_color = "white")
        elif state==1:
            self.dv_v.configure(text = "LV_ON", text_color = "white")
        elif state==2:
            self.dv_v.configure(text = "MISSION SELECTED", text_color = "white")
        elif state==3:
            self.dv_v.configure(text = "READY", text_color = "white")
        elif state==4:
            self.dv_v.configure(text = "DRIVING", text_color = "blue")
        elif state==5:
            self.dv_v.configure(text = "PROBLEM", text_color = "red")
        elif state==6:
            self.dv_v.configure(text = "MISSION FINISHED", text_color = "green")
        else:
            self.dv_v.configure(text = "invalid", text_color = "red")
            
    def change_as(self, state) -> None:
        if state==self.aut:
            return
        self.aut = state
        
        if state==1:
            self.aut_v.configure(text = "OFF", text_color = "white")
        elif state==2:
            self.aut_v.configure(text = "READY", text_color = "white")
        elif state==3:
            self.aut_v.configure(text = "DRIVING", text_color = "blue")
        elif state==4:
            self.aut_v.configure(text = "FINISHED", text_color = "green")
        elif state==5:
            self.aut_v.configure(text = "EMERGENCY", text_color = "red")
        else:
            self.aut_v.configure(text = "invalid", text_color = "red")
            
    def change_miss(self, state) -> None:
        if state==self.miss:
            return
        self.miss = state
        
        if state==0:
            self.miss_v.configure(text = "UNLOCKED", text_color = "white")
        elif state==1:
            self.miss_v.configure(text = "ACCELERATION", text_color = "white")
        elif state==2:
            self.miss_v.configure(text = "SKIDPAD", text_color = "white")
        elif state==3:
            self.miss_v.configure(text = "AUTOCROSS", text_color = "white")
        elif state==4:
            self.miss_v.configure(text = "TRACKDRIVE", text_color = "white")
        elif state==5:
            self.miss_v.configure(text = "EBS TEST", text_color = "white")
        elif state==6:
            self.miss_v.configure(text = "INSPECTION", text_color = "white")
        elif state==7:
            self.miss_v.configure(text = "MANUAL", text_color = "white")
        else:
            self.miss_v.configure(text = "invalid", text_color = "red")

    def change_all(self, dv, AS, miss) -> None:
        self.change_dv(dv)
        self.change_as(AS)
        self.change_miss(miss)
