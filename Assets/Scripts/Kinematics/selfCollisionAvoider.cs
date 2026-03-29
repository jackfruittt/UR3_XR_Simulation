/// A null-space solver for self collision avoidance
/// 
/// Reference:
/// https://www.youtube.com/watch?v=3FDV8af9XLg
/// 
/// Khatib (1986): Real-time obstacle avoidance for manipulators and mobile robots
/// Aoki (2019): Model Predictive Control with Reach-Avoid-Stay Specifications
///              https://mizuhoaoki.com/projects/phd_dissertation_mizuhoaoki.pdf
/// 
/// Notes:
///  --- General Solution ---
///  J*q_dot = r_dot  (underdetermined when n > m, i.e. more joints than task DOF)
///  General solution = particular + homogeneous:
///  q_dot = J_pinv * r_dot  +  (I - J_pinv * J) * q_dot_0
///          |___________|       |___________________|
///          satisfies           any vector in null(J), does not disturb the task
///          J*q_dot = r_dot     q_dot_0 is a free secondary objective
///
///  --- Projector P = (I - J_pinv * J) ---
///  - Idempotent:   P^2 = P
///  - P * J_pinv = 0  (J_pinv lives in col(J^T), orthogonal to null(J))
///  - Does not affect the particular solution: P * J_pinv * r_dot = 0
///  - J_pinv * r_dot is orthogonal to P * q_dot_0
///
///  --- More general form (arbitrary inverses) ---
///  q_dot = K_1 * r_dot + (I - K_2 * J) * q_dot_0
///  K_1 must be a right inverse of J (J*K_1 = I_m) to satisfy J*q_dot = r_dot
///  K_2 can be any generalised inverse of J (J*K_2*J = J)
///  LQO (below) shows J_pinv is the optimal choice for K_1 = K_2
///
///  --- Linear Quadratic Optimisation: which q_dot_0 and which inverse are optimal? ---
///  The null-space general solution has two free choices: which inverse to use (K_1, K_2)
///  and what secondary objective to project (q_dot_0). LQO answers both optimally:
///  choosing K_1 = K_2 = J_pinv minimises ||q_dot||^2 (minimum norm), and projecting
///  q_dot_0 = -grad_H into the null space minimises a secondary cost H(q_dot) subject
///  to the task constraint — i.e. the null-space projection IS the LQO solution.
///  min  H(q_dot) = 0.5*(q_dot - q_dot_0)^T * W * (q_dot - q_dot_0)
///  s.t. J * q_dot = r_dot
///  q_dot in R^n, r_dot in R^m, W > 0 (symmetric positive definite), rank(J) = m
///  q_dot_0 is a "privileged" joint velocity (secondary objective, e.g. collision avoidance)
///
///  Step 1 - Form the Lagrangian (converts constrained -> unconstrained):
///  L(q_dot, lambda) = H(q_dot) + lambda^T * (J*q_dot - r_dot)
///
///  Step 2 - Necessary conditions (set partial derivatives to zero):
///  (1)  dL/dq_dot  = W*(q_dot - q_dot_0) + J^T * lambda = 0
///  (2)  dL/dlambda = J*q_dot - r_dot = 0
///
///  Step 3 - Solve for lambda (eliminate q_dot):
///  From (1):  q_dot = q_dot_0 - W^-1 * J^T * lambda
///  Sub into (2):  J*q_dot_0 - J*W^-1*J^T*lambda = r_dot
///                 lambda = (J*W^-1*J^T)^-1 * (J*q_dot_0 - r_dot)
///  (J*W^-1*J^T is m x m, invertible since W > 0 and rank(J) = m)
///
///  Step 4 - Substitute lambda back to get q_dot*:
///  q_dot* = q_dot_0 + W^-1*J^T*(J*W^-1*J^T)^-1 * (r_dot - J*q_dot_0)
///
///  Step 5 - Recognise the weighted pseudoinverse:
///  J_pinv_W = W^-1 * J^T * (J*W^-1*J^T)^-1
///
///  Step 6 - Expand into particular + null-space terms:
///  q_dot* = J_pinv_W * r_dot  +  (I - J_pinv_W * J) * q_dot_0
///           |_______________|     |______________________|
///           min weighted norm         null-space projection
///           solution (q_dot_0 = 0)    of secondary objective
///
///  Special case W = I:
///  J_pinv_W = J^T*(J*J^T)^-1 = J_pinv   (Moore-Penrose pseudoinverse)
///  q_dot* = J_pinv * r_dot + (I - J_pinv*J) * q_dot_0  <-- recovers the general solution above
///  J_pinv is therefore the unique minimiser of ||q_dot||^2 subject to J*q_dot = r_dot
///
///  --- Conclusion ---
///  q_dot = J_pinv * r_dot + (I - J_pinv*J) * q_dot_0
///  is not a heuristic - it is the provably optimal solution to the LQO problem (W=I):
///  the particular term satisfies the task with minimum joint effort,
///  and the null-space term pursues q_dot_0 (e.g. collision avoidance gradient)
///  without disturbing the task. J_pinv is the unique inverse that achieves this.
///
/// PART 2: ARTIFICIAL POTENTIAL FIELDS FOR COLLISION AVOIDANCE (Khatib 1986)
///
