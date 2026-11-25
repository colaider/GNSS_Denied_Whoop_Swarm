import numpy as np

class Network:
    def __init__(self, sizes=(4, 5, 5), seed=0):
        rng = np.random.default_rng(seed)
        
        self.W = [rng.normal(0, np.sqrt(2 / n_in), (n_out, n_in)) for n_in, n_out in zip(sizes[:-1], sizes[1:])]
        self.b = [np.zeros(n_out) for n_out in sizes[1:]]

    def forward(self, x):
        hs = [x]
        for W, b in zip(self.W, self.b): hs.append(np.tanh(W @ hs[-1] + b))
        return np.append(hs[-1], 1.0), hs


    def backward(self, hs, theta, y, yHat):
        delta = -2.0 * (y - yHat)                   
        gTheta = delta * np.append(hs[-1], 1.0)    

        g = delta * theta[:-1]                     
        gW, gb = [None] * len(self.W), [None] * len(self.b)

        for l in reversed(range(len(self.W))):
            ga = g * (1.0 - hs[l + 1] ** 2)        
            gW[l], gb[l] = np.outer(ga, hs[l]), ga              
            g = self.W[l].T @ ga   

        return gW, gb, gTheta


    def train(self, theta, X, Y, lr=0.01, epochs=100):
        for epoch in range(epochs):
            for x, y in zip(X, Y):
                phi, hs = self.forward(x)
                gW, gb, gT = self.backward(hs, theta, y, theta @ phi)
                for l in range(len(self.W)):
                    self.W[l] -= lr * gW[l]
                    self.b[l] -= lr * gb[l]
                theta -= lr * gT
        return theta

    
        
