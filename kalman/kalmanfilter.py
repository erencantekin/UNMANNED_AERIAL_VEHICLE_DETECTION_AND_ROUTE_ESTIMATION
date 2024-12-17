
import cv2
import numpy as np
import numpy.linalg as la



def kalman(x_esti,P,A,Q,B,u,z,H,R):

    x_pred = A @ x_esti + B @ u;         

    P_pred  = A @ P @ A.T + Q;

    zp = H @ x_pred

    if z is None:
        return x_pred, P_pred, zp

    epsilon = z - zp

    k = P_pred @ H.T @ la.inv(H @ P_pred @ H.T +R)

    x_esti = x_pred + k @ epsilon;
    P  = (np.eye(len(P))-k @ H) @ P_pred;
    return x_esti, P, zp


class KalmanFilter:
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)                              
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)    
    

    def predict(self, coordX, coordY):
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()       
        statePre = self.kf.statePre
        statePost = self.kf.statePost
        Q = self.kf.processNoiseCov
        measurementNoiseCov = self.kf.measurementNoiseCov
        errorCovPre = self.kf.errorCovPre
        errorCovPost = self.kf.errorCovPost
        H = self.kf.measurementMatrix
        
        x, y = int(predicted[0]), int(predicted[1])
        return (x, y), statePre.T[0], statePost.T[0], errorCovPre 

    def kal(self, mu, P, B, u, z):
        A = self.kf.transitionMatrix
        statePre = self.kf.statePre
        Q = self.kf.processNoiseCov
        H = self.kf.measurementMatrix
        R = self.kf.measurementNoiseCov

        x_pred = A @ mu + B @ u
        P_pred = A @ P @ A.T + Q / 4 
        zp = H @ x_pred

        if z is None:
            return x_pred, P_pred

        epsilon = z - zp

        k = P_pred @ H.T @ la.inv(H @ P_pred @ H.T +R)

        x_esti = x_pred + k @ epsilon
        P  = (np.eye(len(P))-k @ H) @ P_pred
        return x_esti, P