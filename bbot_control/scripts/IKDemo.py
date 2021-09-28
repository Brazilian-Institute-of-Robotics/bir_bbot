from IKSolve import IKSolve

# Servo Home Values (degrees) - ***YOURS WILL DIFFER***
# ru_home = 50   # Right leg upper joint home position (servo_angle[0])
# rl_home = 109   # Right leg lower joint home position (servo_angle[1])
# lu_home = 52   # Left leg upper joint home position (servo_angle[2])
# ll_home = 23   # Left leg lower joint home position (servo_angle[3])
 
ik_handler = IKSolve(ru_home=0, rl_home=0,  # Pass home positions to IK class
                     lu_home=0, ll_home=0) # Home position from CAD

while True:
    x = float(input('x:'))
    y = float(input('y:'))
    servo_angle = ik_handler.translate_xy(x, y, flip=False)  # servo_angles[0] refers to the right leg upper servo etc
    print(servo_angle)
