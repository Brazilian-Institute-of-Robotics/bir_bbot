library(deSolve)
library(ggplot2)
library(gridExtra)

K_lqr = matrix(c( -6440.61259723522, -1365.97124527816, -0.681977774441287, -305.181366019499, 
                  -6440.60932513392, -1365.97055186717,  0.681977415077895, -305.181329377807), nrow = 2)

inputs_ini = c(0,0)
inputs_hist = list(inputs_ini)

# Define system of ODE
func = function(t, y, parms){
  inputs = -1*K_lqr%*%y
  inputs_hist[[length(inputs_hist)+1]] = as.numeric(inputs)
  p = parms
  
  list(c(
        -0.0002223981974*(20.0*inputs[1] + 20.0*inputs[2] - 8.0*y[1] - y[2]*(-0.3677016*y[2]*sin(y[4]) - 0.4) + 0.3677016*y[3]**2*sin(y[4]))/(0.0003380111666064*cos(y[4])**2 - 0.00063181192858645) + 0.000919254*(-inputs[1] - inputs[2] + 0.4*y[1] - 0.02*y[2] + 0.08595260896*y[3]**2*sin(y[4])*cos(y[4]) + 3.607152696*sin(y[4]))*cos(y[4])/(0.0003380111666064*cos(y[4])**2 - 0.00063181192858645),
        0.000919254*(20.0*inputs[1] + 20.0*inputs[2] - 8.0*y[1] - y[2]*(-0.3677016*y[2]*sin(y[4]) - 0.4) + 0.3677016*y[3]**2*sin(y[4]))*cos(y[4])/(0.0003380111666064*cos(y[4])**2 - 0.00063181192858645) - 0.00710226*(-inputs[1] - inputs[2] + 0.4*y[1] - 0.02*y[2] + 0.08595260896*y[3]**2*sin(y[4])*cos(y[4]) + 3.607152696*sin(y[4]))/(0.0003380111666064*cos(y[4])**2 - 0.00063181192858645),
        0.005*(-1.431*inputs[1] + 1.431*inputs[2] - 0.3677016*y[1]*y[3]*sin(y[4]) - 0.08595260896*y[2]*y[3]*sin(y[4])*cos(y[4]) - y[3]*(0.08595260896*y[2]*sin(y[4])*cos(y[4]) + 0.04095522))/(0.0004297630448*sin(y[4])**2 + 4.98118377493e-5),
        y[2]
        ))
}

y_ini = c(0, 0.09, 0, 0.15)
inputs_ini = c(0,0)

t0 = 0
t1 = 10
dt = 0.01
time = seq(t0,t1,dt)

sim = ode(y = y_ini, func = func, parms = 0, times = time, method = "ode45")
colnames(sim) <- c("time","x_vel","Pitch_vel","Yaw_vel","Pitch")
sim_data = as.data.frame(sim)

x_pos = list(0)
for (i in 2:length(sim_data$time)){
  len = length(x_pos)
  x_pos[[len+1]] = x_pos[[len]] + sim_data[i,2]*dt
}
sim_data$"x_pos" = as.numeric(x_pos)

# -- PLOTS --
x_vel_plot <- ggplot(sim_data, aes(x = time, y = x_vel)) +
  ggtitle("X vel") + 
  geom_point() + 
  geom_line()

pitch_vel_plot <- ggplot(sim_data, aes(x = time, y = Pitch_vel)) +
  ggtitle("Pitch vel") + 
  geom_point() + 
  geom_line()

x_pos_plot <- ggplot(sim_data, aes(x = time, y = x_pos)) +
  ggtitle("X pos") + 
  geom_point() + 
  geom_line()

pitch_plot <- ggplot(sim_data, aes(x = time, y = Pitch)) +
  ggtitle("Pitch pos") + 
  geom_point() + 
  geom_line()

grid.arrange(pitch_plot,x_pos_plot,pitch_vel_plot,x_vel_plot, ncol = 2, nrow = 2)
