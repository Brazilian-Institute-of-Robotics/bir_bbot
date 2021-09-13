library(deSolve)
library(ggplot2)
library(gridExtra)

K_lqr = matrix(c( -7202.31604897679, -1527.38910154423, -0.689224825521283, -312.474638458303,
                  -7202.31533546446, -1527.38895033865,  0.689224755173669, -312.474633215354), nrow = 2)

K_lqr = 0.01*rbind(c(-7202.31604897679, -1527.38910154423, -0.689224825521283, -312.474638458303),
               c(-7202.31533546446, -1527.38895033865,  0.689224755173669, -312.474633215354))

# K_lqr = -rbind(c( 23.9356745074183, 2.96610851661515, 0.562945763802608, 509.661063400201),
#               c( 23.9359380999371, 2.96609855829759,  -0.562950732073306,  509.66508686494))

# # Define system of ODE for visc = 0.01
# func = function(t, y, parms){
#   
#   inputs = -1*K_lqr%*%y
#   
#   list(c(
#         -0.0002223981974*(20.0*inputs[1] + 20.0*inputs[2] - 8.0*y[1] - y[2]*(-0.3677016*y[2]*sin(y[4]) - 0.4) + 0.3677016*y[3]**2*sin(y[4]))/(0.0003380111666064*cos(y[4])**2 - 0.00063181192858645) + 0.000919254*(-inputs[1] - inputs[2] + 0.4*y[1] - 0.02*y[2] + 0.08595260896*y[3]**2*sin(y[4])*cos(y[4]) + 3.607152696*sin(y[4]))*cos(y[4])/(0.0003380111666064*cos(y[4])**2 - 0.00063181192858645),
#         0.000919254*(20.0*inputs[1] + 20.0*inputs[2] - 8.0*y[1] - y[2]*(-0.3677016*y[2]*sin(y[4]) - 0.4) + 0.3677016*y[3]**2*sin(y[4]))*cos(y[4])/(0.0003380111666064*cos(y[4])**2 - 0.00063181192858645) - 0.00710226*(-inputs[1] - inputs[2] + 0.4*y[1] - 0.02*y[2] + 0.08595260896*y[3]**2*sin(y[4])*cos(y[4]) + 3.607152696*sin(y[4]))/(0.0003380111666064*cos(y[4])**2 - 0.00063181192858645),
#         0.005*(-1.431*inputs[1] + 1.431*inputs[2] - 0.3677016*y[1]*y[3]*sin(y[4]) - 0.08595260896*y[2]*y[3]*sin(y[4])*cos(y[4]) - y[3]*(0.08595260896*y[2]*sin(y[4])*cos(y[4]) + 0.04095522))/(0.0004297630448*sin(y[4])**2 + 4.98118377493e-5),
#         y[2]
#         ))
# }

# Define system of ODE for visc = 0.01
func = function(t, y, parms){

  inputs = -1*K_lqr%*%y

  list(c(
    0.08895927896*(20.0*inputs[1] + 20.0*inputs[2] - 8.0*y[1] - y[2]*(-0.3677016*y[2]*sin(y[4]) - 0.4) + 0.3677016*y[3]**2*sin(y[4]))/(0.25272477143458 - 0.13520446664256*cos(y[4])**2) - 0.3677016*(-inputs[1] - inputs[2] + 0.4*y[1] - 0.02*y[2] + 0.08595260896*y[3]**2*sin(y[4])*cos(y[4]) + 3.607152696*sin(y[4]))*cos(y[4])/(0.25272477143458 - 0.13520446664256*cos(y[4])**2),
    -0.3677016*(20.0*inputs[1] + 20.0*inputs[2] - 8.0*y[1] - y[2]*(-0.3677016*y[2]*sin(y[4]) - 0.4) + 0.3677016*y[3]**2*sin(y[4]))*cos(y[4])/(0.25272477143458 - 0.13520446664256*cos(y[4])**2) + 2.840904*(-inputs[1] - inputs[2] + 0.4*y[1] - 0.02*y[2] + 0.08595260896*y[3]**2*sin(y[4])*cos(y[4]) + 3.607152696*sin(y[4]))/(0.25272477143458 - 0.13520446664256*cos(y[4])**2),
    (-1.431*inputs[1] + 1.431*inputs[2] - 0.3677016*y[1]*y[3]*sin(y[4]) - 0.08595260896*y[2]*y[3]*sin(y[4])*cos(y[4]) - y[3]*(0.08595260896*y[2]*sin(y[4])*cos(y[4]) + 0.04095522))/(0.08595260896*sin(y[4])**2 + 0.00996236754986),
    y[2]
  ))
}

# Define system of ODE for visc = 0.1
# func = function(t, y, parms){
# 
#   inputs = -1*K_lqr%*%y
# 
#   list(c(
#     0.08895927896*(20.0*inputs[1] + 20.0*inputs[2] - 80.0*y[1] - y[2]*(-0.3677016*y[2]*sin(y[4]) - 4.0) + 0.3677016*y[3]**2*sin(y[4]))/(0.25272477143458 - 0.13520446664256*cos(y[4])**2) - 0.3677016*(-inputs[1] - inputs[2] + 4.0*y[1] - 0.2*y[2] + 0.08595260896*y[3]**2*sin(y[4])*cos(y[4]) + 3.607152696*sin(y[4]))*cos(y[4])/(0.25272477143458 - 0.13520446664256*cos(y[4])**2),
#     -0.3677016*(20.0*inputs[1] + 20.0*inputs[2] - 80.0*y[1] - y[2]*(-0.3677016*y[2]*sin(y[4]) - 4.0) + 0.3677016*y[3]**2*sin(y[4]))*cos(y[4])/(0.25272477143458 - 0.13520446664256*cos(y[4])**2) + 2.840904*(-inputs[1] - inputs[2] + 4.0*y[1] - 0.2*y[2] + 0.08595260896*y[3]**2*sin(y[4])*cos(y[4]) + 3.607152696*sin(y[4]))/(0.25272477143458 - 0.13520446664256*cos(y[4])**2),
#     (-1.431*inputs[1] + 1.431*inputs[2] - 0.3677016*y[1]*y[3]*sin(y[4]) - 0.08595260896*y[2]*y[3]*sin(y[4])*cos(y[4]) - y[3]*(0.08595260896*y[2]*sin(y[4])*cos(y[4]) + 0.4095522))/(0.08595260896*sin(y[4])**2 + 0.00996236754986),
#     y[2]
#   ))
# }

y_ini = c(0, 0.02, 0, 0.6)

t0 = 0
t1 = 10
dt = 0.01
time = seq(t0,t1,dt)
y_ini = c(0, 0.02, 0, 0.6)
sim_data = matrix(nrow = 1001,ncol = 5)
inputs_data = matrix(c(0,0),nrow = 1001,ncol = 2)

# for(i in 1:1000){
#   ctime = c(time[i],time[i+1]) #current time
sim = ode(y = y_ini, func = func, parms = NULL, times = time, method = "radau")
#   sim_data[i+1,] <- sim[2,]
#   inputs_data[i+1,] <- -1*K_lqr%*%sim_data[i+1,2:5]
# }

sim_data <- as.data.frame(sim)
colnames(sim_data) <- c("time","x_vel","Pitch_vel","Yaw_vel","Pitch")

# -- Integrate for the X pos plot
x_pos = list(0)
for (i in 2:length(sim_data$time)){
  len = length(x_pos)
  x_pos[[len+1]] = x_pos[[len]] + sim_data[i,2]*dt
}
sim_data$"x_pos" = as.numeric(x_pos)

# -- calculate input history
for(i in 1:1001){
  inputs_data[i,] <- -K_lqr%*%as.numeric(sim_data[i,2:5])
}
colnames(inputs_data) <- c("T_L","T_R")
inputs_data <- as.data.frame(inputs_data)

# -- PLOTS --
x_vel_plot <- ggplot(sim_data, aes(x = time, y = x_vel)) +
  ggtitle("X vel") + 
  # geom_point() + 
  geom_line()

pitch_vel_plot <- ggplot(sim_data, aes(x = time, y = Pitch_vel)) +
  ggtitle("Pitch vel") + 
  # geom_point() + 
  geom_line()

x_pos_plot <- ggplot(sim_data, aes(x = time, y = x_pos)) +
  ggtitle("X pos") + 
  # geom_point() + 
  geom_line()

pitch_plot <- ggplot(sim_data, aes(x = time, y = Pitch)) +
  ggtitle("Pitch pos") + 
  # geom_point() + 
  geom_line()

torq_L_plot <- ggplot(inputs_data, aes(x = time)) +
  ggtitle("Torque Left") + 
  # geom_point() + 
  geom_line(aes(y = T_L))

torq_R_plot <- ggplot(inputs_data, aes(x = time)) +
  ggtitle("Torque Rigth") + 
  # geom_point() + 
  geom_line(aes(y = T_R))

grid.arrange(pitch_plot,x_pos_plot,pitch_vel_plot,x_vel_plot,torq_L_plot,torq_R_plot, ncol = 2, nrow = 3)
