import sys
import lcm
import numpy as np

import matplotlib.pyplot as plt

from lcmtypes import pose_xyt_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: read-log <logfile>\n")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1], "r")

true_ts = []
true_xs = []
true_ys = []
true_thetas = []


slam_ts = []
slam_xs = []
slam_ys = []
slam_thetas = []

solution_ts = []
solution_xs = []
solution_ys = []
solution_thetas = []

for event in log:
    # print(event.channel)
    if event.channel == "TRUE_POSE":
        msg = pose_xyt_t.decode(event.data)
        true_ts.append(msg.utime/ 1e6)
        true_xs.append(msg.x)
        true_ys.append(msg.y)
        true_thetas.append(msg.theta)

      

    if event.channel == "SLAM_POSE":
        msg = pose_xyt_t.decode(event.data)
        slam_ts.append(msg.utime/ 1e6)
        slam_xs.append(msg.x)
        slam_ys.append(msg.y)
        slam_thetas.append(msg.theta)

    if event.channel == "SOLUTION_POSE":
        msg = pose_xyt_t.decode(event.data)
        solution_ts.append(msg.utime/ 1e6)
        solution_xs.append(msg.x)
        solution_ys.append(msg.y)
        solution_thetas.append(msg.theta)

# shift the zero-t
t0 = true_ts[0]
true_ts = [t - t0 for t in true_ts]
slam_ts = [t - t0 for t in slam_ts]
solution_ts = [t - t0 for t in solution_ts]


# plots
fig, ax = plt.subplots(3, 1,figsize=(6,10))

ax[0].plot(true_ts, true_xs, label="TRUE_POSE")
# ax[0].plot(solution_ts, solution_xs, label="SOLUTION_POSE")
ax[0].plot(slam_ts, slam_xs, label="SLAM_POSE")
ax[0].set_ylabel("x [m]")

ax[1].plot(true_ts, true_ys, label="TRUE_POSE")
# ax[1].plot(solution_ts, solution_ys, label="SOLUTION_POSE")
ax[1].plot(slam_ts, slam_ys, label="SLAM_POSE")
ax[1].set_ylabel("y [m]")

ax[2].plot(true_ts, np.unwrap(true_thetas), label="TRUE_POSE")
# ax[2].plot(solution_ts, solution_thetas, label="SOLUTION_POSE")
ax[2].plot(slam_ts, np.unwrap(slam_thetas), label="SLAM_POSE")
ax[2].set_ylabel("θ [rad]")

plt.legend()
plt.savefig("plot_200_particles.png")




## interpolate and then compute the rms

interp_ts = true_ts

slam_interp_xs = np.interp(interp_ts, slam_ts, slam_xs)
slam_interp_ys = np.interp(interp_ts, slam_ts, slam_ys)
slam_interp_thetas = np.interp(interp_ts, slam_ts, slam_thetas)


def rmse(e):
  return np.sqrt(np.mean([ee**2 for ee in e]))

def wrap_error(x, y):
  return np.arctan2(np.sin(x-y), np.cos(x-y))



print("Max Error: xs: ", np.max(np.abs(slam_interp_xs - true_xs)))
print("Max Error: ys: ", np.max(np.abs(slam_interp_ys - true_ys)))
print("Max Error: thetas: ", np.max(np.abs([wrap_error(s[0], s[1]) for s in zip(slam_interp_thetas, true_thetas)])))

print("RMSE: xs: %f" % rmse(slam_interp_xs - true_xs))
print("RMSE: ys: %f" % rmse(slam_interp_ys - true_ys))
print("RMSE: thetas: %f" % rmse([wrap_error(s[0], s[1]) for s in zip(slam_interp_thetas, true_thetas)]))

