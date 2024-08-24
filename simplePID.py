import numpy as np

class PID:
    def __init__(self, Kp, Ki, Kd, target):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target
        self.error = 0
        self.prev_error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.output = 0

    def update(self, measurement, dt):
        self.error = self.target - measurement
        self.integral_error += self.error * dt
        self.derivative_error = (self.error - self.prev_error) / dt
        self.output = self.Kp * self.error + self.Ki * self.integral_error + self.Kd * self.derivative_error
        self.prev_error = self.error
        return self.output
    
    def set_target(self, target):
        self.target = target


def main():
    iterations = 100
    target = [10, 20, 500]
    Kp = 0.3
    Ki = 0.04
    Kd = 0.01
    pids = [PID(Kp, Ki, Kd, target[i]) for i in range(len(target))]
    measurements = np.empty((len(target), iterations))
    measurement = np.zeros(len(target))
    dt = 0.05
    for i in range(iterations):
        if i == iterations / 2:
            for j in range(len(target)):
                pids[j].set_target(target[j] * 2)
        for j in range(len(target)):
            measurement[j] += pids[j].update(measurement[j], dt)
            measurements[j][i] = measurement[j]

    #print last measurement
    print(measurements[:,-1])

if __name__ == '__main__':
    main()

