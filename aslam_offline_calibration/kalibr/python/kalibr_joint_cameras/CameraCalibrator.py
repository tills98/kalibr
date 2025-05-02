from __future__ import print_function #handle print in 2.x python
import sm
from sm import PlotCollection
from kalibr_common import ConfigReader as cr
import aslam_cv as acv
import aslam_cameras_april as acv_april
import aslam_cv_backend as acvb
import aslam_backend as aopt
import incremental_calibration as ic
import kalibr_camera_calibration as kcc

from matplotlib.backends.backend_pdf import PdfPages
import mpl_toolkits.mplot3d.axes3d as p3
import cv2
import numpy as np
import pylab as pl
import math
import gc
import sys


#available models
cameraModels = { 'pinhole-radtan': acvb.DistortedPinhole,
                 'pinhole-equi':   acvb.EquidistantPinhole,
                 'pinhole-fov':    acvb.FovPinhole,
                 'omni-none':      acvb.Omni,
                 'omni-radtan':    acvb.DistortedOmni,
                 'eucm-none':      acvb.ExtendedUnified,
                 'ds-none':        acvb.DoubleSphere}


#DV group IDs
CALIBRATION_GROUP_ID = 0
TRANSFORMATION_GROUP_ID = 1
LANDMARK_GROUP_ID = 2

class CameraCalibrationTarget(object):
    """
    A class to hold the camera and its target (for one image) where it was initially calibrated for.
    """
    def __init__(self, cameraGeometry: kcc.CameraGeometry, T_tc_guess: np.ndarray, target_points: np.ndarray, observations: np.ndarray):
        """
        :param cameraGeometry: the camera geometry
        :param T_tc_guess: the guess for the transformation from target to camera
        :param target_points: the target points in the camera frame
        :param observations: the observations of the target points in the 2D image
        """
        self.camera = cameraGeometry
        self.T_tc_guess = T_tc_guess
        self.target_points = target_points
        self.observations = observations

        self.build_expressions()

    def build_expressions(self):
        """
        Build the expressions for the target points in the camera frame.
        :return:
        """
        #build the expressions for the target points
        self.P_t_dv = list()
        self.P_t_ex = list()

        for i in range(0, len(self.target_points)):
            p_t_dv = aopt.HomogeneousPointDv(sm.toHomogeneous(self.target_points[i]))
            p_t_ex = p_t_dv.toExpression()
            self.P_t_dv.append(p_t_dv)
            self.P_t_ex.append(p_t_ex)

    def getPoint(self, i):
        return self.P_t_ex[i]


class JointCalibrationTargetOptimizationProblem(ic.CalibrationOptimizationProblem):
    @classmethod
    def fromTargetViewObservations(cls, cameras, targets, useBlakeZissermanMest=True):
        """
        Create a new optimization problem for the given target view observations in one image.

        :param cameras: list of cameras
        :param targets: list of targets for all cameras in one image (|num_camera|)
        :param baselines: list of baselines
        :param T_tc_guess: guess for the transformation from target to camera
        :param rig_observations: list of observations for the cameras
        :param useBlakeZissermanMest: use Blake-Zisserman M-estimator
        :return: a new optimization problem
        """
        # create a new optimization problem
        rval = JointCalibrationTargetOptimizationProblem()        

        # store the arguements in case we want to rebuild a modified problem
        rval.cameras = cameras
        rval.targets = targets
        
        target = targets[0]
        camera = cameras[0]

        # 1. Create a design variable for first camera pose
        T_target_camera = sm.Transformation(target.T_tc_guess)
        rval.dv_T_target_camera = aopt.TransformationDv(T_target_camera) 
        for i in range(0, rval.dv_T_target_camera.numDesignVariables()):
            rval.addDesignVariable(rval.dv_T_target_camera.getDesignVariable(i), TRANSFORMATION_GROUP_ID)

        # 2. add landmark DVs of all targets
        for p in target.P_t_dv: # P_t_dv = [p_t_dv] where p_t_dv = aopt.HomogeneousPointDv(points(i)), the corner points in its 3D world (x, y, 0)
            rval.addDesignVariable(p, LANDMARK_GROUP_ID)

        # 3. add camera DVs
        camera.setDvActiveStatus(True, True, False)
        rval.addDesignVariable(camera.dv.distortionDesignVariable(), CALIBRATION_GROUP_ID)
        rval.addDesignVariable(camera.dv.projectionDesignVariable(), CALIBRATION_GROUP_ID)
        rval.addDesignVariable(camera.dv.shutterDesignVariable(), CALIBRATION_GROUP_ID)

        # 4. add all observations for this view
        rval.rerrs = dict()
        rerr_cnt = 0

        T_target_cam = rval.dv_T_target_camera.expression # no .inverse() here, because we want to go from points on the target to the camera

        # \todo pass in the detector uncertainty somehow.
        cornerUncertainty = 1.0
        R = np.eye(2) * cornerUncertainty * cornerUncertainty
        invR = np.linalg.inv(R)

        rval.rerrs = list()

        # iterate the target points
        for i in range(0, len(target.P_t_ex)):
            p_target = target.P_t_ex[i]
            y = target.observations[i]
            
            rerr_cnt += 1
            rerr = camera.model.reprojectionError(y, invR, T_target_cam * p_target, camera.dv)
            rerr.idx = rerr_cnt

            if useBlakeZissermanMest:
                mest = aopt.BlakeZissermanMEstimator(2.0)
                rerr.setMEstimatorPolicy(mest)

            rval.addErrorTerm(rerr)
            rval.rerrs.append(rerr)

        # # ---- old code ----
        # # 1. Create a design variable for all camera poses
        # rval.dv_Ts_target_camera = list()
        # for target in targets:
        #     T_target_camera = sm.Transformation(target.T_tc_guess)
            
        #     dv_T_target_camera = aopt.TransformationDv(T_target_camera)
        #     rval.dv_Ts_target_camera.append(dv_T_target_camera)

        #     # add the design variable Transformation matrix to the problem
        #     for i in range(0, dv_T_target_camera.numDesignVariables()):
        #         rval.addDesignVariable(dv_T_target_camera.getDesignVariable(i), TRANSFORMATION_GROUP_ID)
        
        # # 2. add landmark DVs of all targets
        # for target in targets:
        #     for p in target.P_t_dv: # P_t_dv = [p_t_dv] where p_t_dv = aopt.HomogeneousPointDv(points(i)), the corner points in its 3D world (x, y, 0)
        #         rval.addDesignVariable(p, LANDMARK_GROUP_ID)
        
        # # 3. add camera DVs
        # for camera in cameras:
        #     # if not camera.isGeometryInitialized:
        #     #     raise RuntimeError('The camera geometry is not initialized. Please initialize with initGeometry() or initGeometryFromDataset()')
        #     camera.setDvActiveStatus(True, True, False)
        #     rval.addDesignVariable(camera.dv.distortionDesignVariable(), CALIBRATION_GROUP_ID)
        #     rval.addDesignVariable(camera.dv.projectionDesignVariable(), CALIBRATION_GROUP_ID)
        #     rval.addDesignVariable(camera.dv.shutterDesignVariable(), CALIBRATION_GROUP_ID)
        
        # #4. add all observations for this view
        # rval.rerrs = dict()
        # rerr_cnt = 0

        # for target_id, target in enumerate(targets):
        #     T_cam_target = rval.dv_Ts_target_camera[target_id].expression  # no .inverse() here, because we want to go from points on the target to the camera
        #     # T_target_camera = target.T_tc_guess

        #     # \todo pass in the detector uncertainty somehow.
        #     cornerUncertainty = 1.0
        #     R = np.eye(2) * cornerUncertainty * cornerUncertainty
        #     invR = np.linalg.inv(R)

        #     rval.rerrs[target_id] = list()

        #     # iterate the target points
        #     for i in range(0, len(target.P_t_ex)):
        #         p_target = target.P_t_ex[i]
        #         y = target.observations[i]
                
        #         # iterate over all cameras
        #         for camera in cameras:
        #             rerr_cnt += 1
        #             rerr = camera.model.reprojectionError(y, invR, T_cam_target * p_target, camera.dv)
        #             rerr.idx = rerr_cnt

        #             if useBlakeZissermanMest:
        #                 mest = aopt.BlakeZissermanMEstimator(2.0)
        #                 rerr.setMEstimatorPolicy(mest)

        #             rval.addErrorTerm(rerr)
        #             rval.rerrs[target_id].append(rerr)

        
        sm.logDebug("Adding a view with {0} error terms.".format(rerr_cnt))
        return rval



class CameraCalibration(object):
    def __init__(self, cameras, verbose=False, useBlakeZissermanMest=True):
        """
        :param cameras: list of kcc.CameraGeometry objects
        :param targets: list of CameraCalibrationTarget objects
        :param useBlakeZissermanMest: use Blake-Zisserman M-estimator
        :param verbose: verbose output
        """
        self.cameras = cameras
        self.useBlakeZissermanMest = useBlakeZissermanMest
        #create the incremental estimator
        self.estimator = ic.IncrementalEstimator(CALIBRATION_GROUP_ID)
        self.linearSolverOptions = self.estimator.getLinearSolverOptions()
        self.optimizerOptions = self.estimator.getOptimizerOptions()
        self.verbose = verbose

        #storage for the used views
        self.views = list()
        

    def addTargetView(self, targets, force=False):
        """
        called for every timestamp with all observations at the given timestamp and the T_targetToCamera_guess

        :param targets: list of CameraCalibrationTarget objects, targets for each camera in the current image
        :param force: save design variables in case the batch is rejected
        :return:
        """
        #create the problem for this batch and try to add it
        batch_problem = JointCalibrationTargetOptimizationProblem.fromTargetViewObservations(self.cameras, targets, useBlakeZissermanMest=self.useBlakeZissermanMest)
        self.estimator_return_value = self.estimator.addBatch(batch_problem, force)
        
        if self.estimator_return_value.numIterations >= self.optimizerOptions.maxIterations:
            sm.logError("Did not converge in maxIterations... restarting...")
            raise kcc.OptimizationDiverged
        
        success = self.estimator_return_value.batchAccepted
        if success:
            sm.logDebug("The estimator accepted this batch")
            self.views.append(batch_problem)
        else:
            sm.logDebug("The estimator did not accept this batch")
        return success