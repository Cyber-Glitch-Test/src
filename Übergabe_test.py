import numpy as np
import rospy
import tf
import math
from filterpy.kalman import KalmanFilter

class KalmanFilter3D:
    def __init__(self, dt=1, process_noise=1e-4, measurement_noise=1e-1):
        """
        Kalman-Filter für eine 3D-Position (x, y, z).

        :param dt: Zeitschritt
        :param process_noise: Unsicherheit im Modell
        :param measurement_noise: Messrauschen
        """
        self.kf = KalmanFilter(dim_x=6, dim_z=3)  # 6 Zustände (Position + Geschwindigkeit für x, y, z), 3 Messungen (x, y, z)
        
        # Zustandsübergangsmatrix
        self.kf.F = np.array([[1, dt, 0,  0,  0,  0],  
                              [0,  1, 0,  0,  0,  0],
                              [0,  0, 1, dt,  0,  0],
                              [0,  0, 0,  1,  0,  0],
                              [0,  0, 0,  0,  1, dt],
                              [0,  0, 0,  0,  0,  1]])

        # Messmatrix
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],  
                              [0, 0, 1, 0, 0, 0],  
                              [0, 0, 0, 0, 1, 0]])

        # Prozessrauschen
        self.kf.Q = np.eye(6) * process_noise

        # Messrauschen
        self.kf.R = np.eye(3) * measurement_noise

        # Kovarianzmatrix
        self.kf.P = np.eye(6) * 500

        # Anfangszustand
        self.kf.x = np.zeros((6, 1))

    def update(self, measurement):
        """ Aktualisiert den Kalman-Filter mit einer neuen Messung. """
        z = np.array(measurement).reshape(3, 1)  # Messung (x, y, z)
        self.kf.predict()
        self.kf.update(z)
        return self.kf.x[:3].flatten()  # Gibt die gefilterte Position zurück (x, y, z)

# Kalman-Filter für Schulter, Ellbogen und Hand erstellen
kf_shoulder = KalmanFilter3D()
kf_elbow = KalmanFilter3D()
kf_hand = KalmanFilter3D()

class get_Hum_metrics:
    def __init__(self):
        self.upperarm_length = 0
        self.forearm_length = 0
        self.shoulder_coords = [0, 0, 0]
        self.elbow_coords = [0, 0, 0]
        self.hand_coords = [0, 0, 0]
        self.calc_arm_length()

    def camera_listener(self):
        try:
            time = rospy.Time(0)
            listener = tf.TransformListener()
            listener.waitForTransform("world", "right_shoulder", time, rospy.Duration(1.0))
            listener.waitForTransform("world", "right_elbow", time, rospy.Duration(1.0))
            listener.waitForTransform("world", "right_hand", time, rospy.Duration(1.0))

            shoulder_trans, _ = listener.lookupTransform("world", "right_shoulder", time)
            elbow_trans, _ = listener.lookupTransform("world", "right_elbow", time)
            hand_trans, _ = listener.lookupTransform("world", "right_hand", time)

            # Messungen durch den Kalman-Filter glätten
            self.shoulder_coords = kf_shoulder.update(shoulder_trans)
            self.elbow_coords = kf_elbow.update(elbow_trans)
            self.hand_coords = kf_hand.update(hand_trans)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF Error: {e}")

    def calc_arm_length(self):
        self.camera_listener()
        self.upperarm_length = self.calc_euclidean_distance(self.shoulder_coords, self.elbow_coords)
        self.forearm_length = self.calc_euclidean_distance(self.hand_coords, self.elbow_coords)

    def calc_euclidean_distance(self, point1, point2):
        distance = sum((p1 - p2) ** 2 for p1, p2 in zip(point1, point2))
        return math.sqrt(distance)

