#!/usr/bin/python

import numpy as np
from numpy import rad2deg, arcsin, sqrt, tan
import sys
from .interface import BaseInterface
from ophyd import Component as Cpt
from .device import GroupDevice
from pcdsdevices.epics_motor import IMS
from ophyd.signal import EpicsSignal, EpicsSignalRO
from pcdsdevices.pseudopos import PseudoSingleInterface
from pcdsdevices.positioner import FuncPositioner

alpha_r = 63 * np.pi/180 #rail angle
r = 2960.0 #first rail distance from sample to rail rod at 27deg
R = 6735.0
alpha_cos = np.cos(27 * np.pi/180)


def ThetaToMotors(theta, samz_offset=0):
    theta = theta * np.pi/180 #change to radian
    x1 = (r) * np.sin(theta)/(np.sin(alpha_r) * np.sin(alpha_r+theta))
    x2 = (R) * np.sin(theta)/(np.sin(alpha_r) * np.sin(alpha_r+theta))
    dz = (r)/np.sin(alpha_r)-x1 * np.sin(alpha_r)/np.sin(theta)
    if theta == 0:
        dz = 0
    return x1, x2, dz


def y1ToGamma(y1):
    gamma = np.arctan((y1)/(r))
    gamma = gamma * 180/np.pi
    return gamma


def y2ToGamma(y2):
    gamma = np.arctan((y2)/(R))
    gamma = gamma * 180/np.pi
    return gamma


def GammaToMotors(gamma):
    gamma = gamma * np.pi/180
    y1 = (r) * np.tan(gamma)
    y2 = (R) * np.tan(gamma)
    dy = y2 - y1
    return y1, y2, dy


def x1ToTheta(x1, samz_offset=0):
    theta = np.arctan(x1 * (np.sin(alpha_r))**2/(r-samz_offset * alpha_cos-x1 * np.sin(2 * alpha_r)/2))
    theta = theta * 180/np.pi
    return theta


def x2ToTheta(x2, samz_offset=0):
    theta = np.arctan(x2 * (np.sin(alpha_r))**2/(R-samz_offset * alpha_cos-x2 * np.sin(2 * alpha_r)/2))
    theta = theta * 180/np.pi
    return theta


def xTox12(x):
    x12 = x/np.sin(alpha_r)
    return x12


def xToz(x):
    z = x/np.tan(alpha_r)
    return z


def MotorsTox(x1, x2, z):
    x_x1 = x1 * np.sin(alpha_r)
    x_x2 = x2 * np.sin(alpha_r)
    x_z = z * np.tan(alpha_r)
    return x_x1, x_x2, x_z


def ThetaToMotors_print(theta):
    x1, x2, z = ThetaToMotors(theta)
    print("Move x1 to %+.4f" % x1)
    print("Move x2 to %+.4f" % x1)
    print("Move z to %+.4f" % z)


class LADM(BaseInterface, GroupDevice):
    """
    Class to control the LADM. Includes Single motors and beamstops.
    
    Parameters
    ----------
    prefix : str
        Base PV for the LADM

    name : str
        Alias for the device   
    """
    tab_component_names = True
    
    #Motors
    x1_us = Cpt(IMS, ':MMS:01', kind='normal') #X1 Upstream
    x2_ds = Cpt(IMS, ':MMS:04', kind='normal') #X2 Downstream
    y1_us = Cpt(IMS, ':MMS:03', kind='normal') #Y1 Upstream
    y2_ds = Cpt(IMS, ':MMS:05', kind='normal') #Y2 Dowstream
    z_us  = Cpt(IMS, ':MMS:02', kind='normal')  #Z Upstream
    bs_6_r = Cpt(IMS, ':MMS:12', kind='normal')
    bs_6_t = Cpt(IMS, ':MMS:11', kind='normal')
    bs_2_r = Cpt(IMS, ':MMS:13', kind='normal')
    bs_2_t = Cpt(IMS, ':MMS:14', kind='normal')
    bs_10_r = Cpt(IMS, ':MMS:15', kind='normal')
    bs_10_t = Cpt(IMS, ':MMS:16', kind='normal')

    theta_pv = EpicsSignal('XCS:VARS:LAM:Theta', name = 'LADM_theta')
    gamma_pv = EpicsSignal('XCS:VARS:LAM:Gamma', name='LADM_gamma')
    motors = {
                "x1": x1_us,
                "y1": y1_us,
                "x2": x2_ds,
                "z": z_us
             }

    def __init__(self):#, x1=x1_us, y1=y1_us, x2=x2_ds, y2=y2_ds, z=z_us, theta_pv=theta_pv, gamma_pv=gamma_pv):
#        self.x1 = x1 #EpicsMotors
#        self.x2 = x2
#        self.y1 = y1
#        self.y2 = y2
#        self.z = z
        self.theta = None #VirtualMotor
        self.XT = None #VirtualMotor
        self.gamma = None
        self.__lowlimX = None
        self.__hilimX = None
#        self.motors = {
#            "x1": x1_us,
#            "y1": y1_us,
#            "x2": x2_ds,
#            "y2": y2_ds,
#            "z": z_us
#            }
#        self._theta_pv = theta_pv #EpicsSignal
#        self._gamma_pv = gamma_pv #EpicsSignal

        super().__init__(self, LADM, prefix=prefix, name=name)
    def __theta_movement(self, theta, samz_offset=0):
        x1, x2, z = ThetaToMotors(theta, samz_offset)
        z_now = z_us.wm()
        try:
            if z_now < z:
                print("Moving z to %+4.f" % z)
                z_us.mv(z)
                z_us.wait()
                print("Moving x1 to to %+4.f and x2 to %.4f\n" % (x1, x2))
                x1_us.mv(x1); x2_us.mv(x2)
                self.waitAll()
            else:
                print("Moving x1 to %+4.f and x2 to %.4f\n" % (x1, x2))
        except KeyboardInterrupt:
            self.stop()
        finally:
            theta_pv.put(self.wmTheta())

    def status(self):
        str = "**LADM Status**\n "
        str += "\t%10s\t%10s\t%10s\n" % ("Motor", "User", "Dial")
        str += "\t%10s\t%+10.4f\t%+10.4s\n" % ("theta", self.theta.wm(), "-")
        str += "\t%10s\t%+10.4f\t%+10.4s\n" % ("XT", self.XT.wm(), "-")
        #keys = self.motors.keys()
        #keys.sorted()
        for key,value in motors.items():
            m = motors[key]
            str += "\t%10s\t%+10.4f\t%+10.4f\n" % (key, m.wm(), m.dial_position.get())
        print(str)

    def ThetaToMotors_print(self, theta):
        x1, x2, z = ThetaToMotors(theta)
        print("move x1 to %+.4f" % x1)
        print("move x2 to %+.4f" % x2)
        print("move  z to %+.4f" % z)

    def moveTheta(self, theta, samz_offset=0):
        theta_now = self.theta.wm()
        if theta_now is np.nan:
            theta1 = x1ToTheta(x1_us.wm(), samz_offset)
            theta2 = x2ToTheta(x2_ds.wm(), samz_offset)
            x1_th1, x2_th1, z_th1 = ThetaToMotors(theta1)
            x1_th2, x2_th2, z_th2 = ThetaToMotors(theta2)
            str = ("theta(x1)= %.4f \n  Should move x2 to %.4f \n  Should move z to %.4f\n" % (theta1, x2_th1, z_th1))
            str += ("theta(x2)= %.4f \n  Should move x1 to %.4f \n  Should move z to %.4f\n\n" % (theta2, x1_th2, z_th2))
            str += self.status()
            print(str)
        else:
            if abs(theta-theta_now) <= 28:
                self.__theta_movement(theta, samz_offset)
            else:
                theta_1 = (theta + theta_now)/2
                self.__theta_movement(theta_1, samz_offset)
                self.theta.wait()
                self.__theta_movement(theta, samz_offset)

    def waitAll(self):
        z_us.wait(); x1_us.wait(); x2_ds.wait()

    def wmTheta(self, samz_offset=0):
        theta1 = x1ToTheta(x1_us.wm(), samz_offset)
        theta2 = x2ToTheta(x2_ds.wm(), samz_offset)
        tolerance = .01
        if samz_offset > 0:
            thetar = np.arctan((x2_ds.wm() - x1_us.wm())/(R-r))
            theta = thetar * 180/np.pi
            ca_samz_offset = -((x1_us.wm() * (np.sin(alpha_r))**2 / np.tan(thetar)) - r + x1_us.wm() * np.sin(2*alpha_r)/2)/alpha_cos
            str = " %.4f (by x1) / %.4f (by x2) / sample z at %.4f  \n" % (theta1, theta2, samz_offset)
            str += "or \n theta %.4f at sample z offset %.4f " % (theta, ca_samz_offset)
            print(str)
        elif abs(theta1 - theta2) < tolerance:
            return theta1
        else:
            return np.nan

    def wmGamma(self):
        gamma1 = y1ToGamma(y1_us.wm())
        gamma2 = y1ToGamma(y2_ds.wm())
        str = "gamma = %.4f (y1= %.4f / y2= %.4f)" % (gamma2-gamma1, y1_us.wm(), y2_ds.wm())
        print(str)

    def mvrGamma(self):
        y1, y2 = GammaToMotors(theta)
        gamma1 = y1ToGamma(y1_us.wm())
        gamma2 = y1ToGamma(y2_ds.wm())
        y1_us.mvr(y1)
        y2_ds.mvr(y2)

    def setTheta(self, value):
        """ set x1, x2 and  z for Theta value   """
        x1, x2, z = ThetaToMotors(value)
        x1_us.set(x1)
        x2_ds.set(x2)
        z_us.set(z)

    def moveX(self,x):
        """ whole ladm move horizontally and keep same Z distance from diff  """
        if ((x <= self.__lowlimX) or (x >= self.__hilimX)):
            print("Asked to move %s outside limit, aborting" % (self.XT.name)) #need to study logprint and rewrite an alternative. Need to test functionality of .pvname in xcs3.

        else:
            try:
                x1 = xTox12(x)
                x2 = xTox12(x)
                z = xToz(x)
                z_now = z_us.wm()
                if z > z_now:
                    print("Moving z to o %+4.f\n" % z)
                    z_us.mv(z)
                    z_us.wait()
                    print("Moving x1 to %+4.f and x2 to %.4f\n" % (x1, x2))
                    x1_us.mv(x1); x2_ds.mv(x2)
                else:
                    print("Moving x1 to %+4.f and x2 to %.4f\n" % (x1, x2))
                    x1_us.mv(x1); x2_ds.mv(x2)
                    x1_us.wait(); x2_ds.wait()
                    print("Moving z to %+4.f\n" % z)
            except KeyboardInterrupt:
                self.stop()

    def tweakH(self, x):
        """ whole ladm move horizontally and keep same Z distance from diff  """
        try:
            x1 = xTox12(x)
            x2 = xTox12(x)
            z = xToz(x)
            z_now = z_us.wm()
            if z > z_now:
                print("Moving z to %+4.f\n" % z)
                z_us.mvr(z - z_now)
                z_us.wait()
                print("Moving x1 to %+4.f and x2 to %.4f\n" % (x1, x2))
                x1_us.mvr(x1); x2_ds.mvr(x2)
            else:
                print("Moving x1 to %+4.f and x2 to %.4f\n" % (x1, x2))
                x1.mvr(x1); x2_ds.mvr(x2)
                x1_us.wait(); x2_ds.wait()
                print("Moving z to %+4.f\n" % z)
                z_us.mvr(z - z_now)
        except KeyboardInterrupt:
            self.stop()

    def mvrV(self, y):
        """ ladm move vertically  """
        try:
            y1_us.mvr(y)
            y2_ds.mvr(y)
        except KeyboardInterrupt:
            self.stop()

    def wmX(self):
        x1 = x1_us.wm()
        x2 = x2_ds.wm()
        z = z_us.wm()
        x_x1, x_x2, x_z = MotorsTox(x1, x2, z)
        print(x_x1, x_x2, x_z)
        db_x1 = EpicsSignal(x1_us.prefix+'.RDBD', name='db_x1').get()
        db_x2 = EpicsSignal(x2_ds.prefix+'.RDBD', name='db_x2').get()
        tolerance = db_x1 + db_x2
        if (abs(x_x1 - x_x2) <= tolerance):
            z_theo = xToz(x_x1)
            db_z = EpicsSignal(z_us.prefix+'.RDBD', name='db_z').get()
            if abs(z - z_theo) < 2 * db_z:
                return x_x1
            else:
                return np.nan
        else:
            return np.nan

    def ca_theta(self, cal_theta, samz_offset=0):
        """ calculation relative x and z postion at certain theta
        ca_theta(theta,samz_offset(downstream offset / mm)) """
        cal_thetarad = cal_theta * np.pi/180
        cal_x1 = (r-samz_offset * alpha_cos) * np.sin(cal_thetarad)/(np.sin(alpha_r) * np.sin(alpha_r + cal_thetarad))
        cal_x2 = (R - samz_offset * alpha_cos) * np.sin(cal_thetarad)/(np.sin(alpha_r) * np.sin(alpha_r + cal_thetarad))
        cal_dz = (r - samz_offset * alpha_cos)/np.sin(alpha_r) - cal_x1 * np.sin(alpha_r)/np.sin(cal_thetarad)
        str = "      theta     =     %3.2f \n "      % cal_theta
        str += "      x1        =     %3.2f \n"       % cal_x1 
        str += "      x2        =     %3.2f \n"       % cal_x2
        str += "      delta z   =     %3.2f \n"       % cal_dz
        str += "sample z offset =     %3.2f \n"       % samz_offset
        print(str)

    def _setX(self, value):
        x1 = xTox12(value)
        x2 = xTox12(value)
        z = xToz(value)
        x1_us.set(x1)
        x2_ds.set(x2)
        z_us.set(z)

    def _set_lowlimX(self, value):
        self.__lowlimX = value

    def _set_hilimX(self, value):
        self.__hilimX = value

    def _get_lowlimX(self):
        return self.__lowlimX

    def _get_hilimX(self):
        return self.__hilimX

    def stop(self):
        x1_us.stop()
        x2_ds.stop()
        y1_us.stop()
        y2_ds.stop()
        z_us.stop()

    def __repr__(self):
        return self.status()
