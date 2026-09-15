import numpy as np
from src.motor_model import MotorParams, state_space
from src.lqr import augment_with_integrator, is_controllable, lqr_gain

p = MotorParams()
A, B, C = state_space(p)
A_aug, B_aug = augment_with_integrator(A, B, C)

print("Controllable:", is_controllable(A_aug, B_aug))

Q = np.diag([1.0, 0.0, 1.0])
R = np.array([[1.0]])
K = lqr_gain(A_aug, B_aug, Q, R)
print("K =", K.round(3))
print("Closed-loop poles:", np.linalg.eigvals(A_aug - B_aug @ K).round(3))