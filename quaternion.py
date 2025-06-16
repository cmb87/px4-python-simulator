import numpy as np


class Quaternion(object):
 
    @staticmethod
    def quat2Euler(Z):
        """Transformation function

        Parameters
        ----------
        Z : param
            Quaternions

        Returns
        -------
        param
            Description
        """
        a, b, c, d = Z[0], Z[1], Z[2], Z[3]

        PHI = np.zeros(3)
        PHI[0] = np.arctan2(2 * (c * d + a * b), (a ** 2 - b ** 2 - c ** 2 + d ** 2))
        PHI[1] = np.arcsin(2 * (a * c - b * d))
        PHI[2] = np.arctan2(2 * (b * c + a * d), (a ** 2 + b ** 2 - c ** 2 - d ** 2))
        return PHI

    @staticmethod
    def euler2quat(PHI):
  
        sa, sb, sc = np.sin(0.5 * PHI[0]), np.sin(0.5 * PHI[1]), np.sin(0.5 * PHI[2])
        ca, cb, cc = np.cos(0.5 * PHI[0]), np.cos(0.5 * PHI[1]), np.cos(0.5 * PHI[2])

        Z = np.zeros(4)
        Z[0] = cc * cb * ca + sc * sb * sa
        Z[1] = cc * cb * sa - sc * sb * ca
        Z[2] = cc * sb * ca + sc * cb * sa
        Z[3] = sc * cb * ca - cc * sb * sa
        return Z

    @staticmethod
    def Mfg(Z):
        """Rotates from system stationary system (g)  to  body system (f)

        Parameters
        ----------
        Z : param
            Quaternionen

        Returns
        -------
        param
            Rotation matrix, Mfg [3x3]
        """



        a, b, c, d = Z[0], Z[1], Z[2], Z[3]

        Mfg = [
            [
                (a ** 2 + b ** 2 - c ** 2 - d ** 2),
                2 * (b * c + a * d),
                2 * (b * d - a * c),
            ],
            [
                2 * (b * c - a * d),
                (a ** 2 - b ** 2 + c ** 2 - d ** 2),
                2 * (c * d + a * b),
            ],
            [
                2 * (b * d + a * c),
                2 * (c * d - a * b),
                (a ** 2 - b ** 2 - c ** 2 + d ** 2),
            ],
        ]
        return np.asarray(Mfg)


    @staticmethod
    def transform(xb, x0, z0):

        M = Quaternion.Mfg(z0).T
        return x0 + np.dot(M, xb)
