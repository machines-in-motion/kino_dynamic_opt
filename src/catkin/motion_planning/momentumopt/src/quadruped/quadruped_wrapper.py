import os

import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *

class QuadrupedWrapper(RobotWrapper):

    def __init__(self, urdf, dt=0.01, q=None):
        package_dirs = [os.path.dirname(os.path.dirname(os.path.abspath(__file__)))]
        ## For pinocchio_v2
        # RobotWrapper.BuildFromURDF(urdf, package_dirs=package_dirs, root_joint=se3.JointModelFreeFlyer())
        # RobotWrapper.__init__(self)
        self.robot = RobotWrapper.BuildFromURDF(urdf, package_dirs=package_dirs, root_joint=se3.JointModelFreeFlyer())
        # Create data again after setting frames
        self.model = self.robot.model
        self.data = self.robot.model.createData()
        if not q is None:
            self.set_configuration(q)
        else:
            self.q = None
        self.M_com = None
        self.dt = 0.01

        self.effs = ["BR", "BL", "FR", "FL"]  # order is important
        self.joints_list = ["HFE", "KFE", "END"]

        self.colors = {"BL": "r", "BR": "y", "FL": "b", "FR": "g"}

        self.initDisplay(loadModel=True)
        self.robot.viewer.gui.addFloor('world/floor')

        self.set_init_config()

        self.init_jacobians_and_trafos()

    def set_init_config(self):
        model = self.model
        data = self.data
        NQ = model.nq
        NV = model.nv
        self.q, self.dq, self.ddq, tau = zero(NQ), zero(NV), zero(NV), zero(NV)

        # The free flyer has an idenitity placement
        identity_placement = np.matrix(se3.utils.se3ToXYZQUAT(se3.SE3.Identity())).T
        print("shape:", NQ)
        self.q[:np.shape(identity_placement)[0]] = identity_placement

        self.num_ctrl_joints = self.q.shape[0] - identity_placement.shape[0]

        # Set initial configuration
        angle = np.deg2rad(60.0)
        q_dummy = np.zeros(self.num_ctrl_joints)
        q_dummy[:int(self.num_ctrl_joints / 2)] = - angle
        q_dummy[:int(self.num_ctrl_joints / 2):2] = 0.5 * angle
        q_dummy[int(self.num_ctrl_joints / 2):] = angle
        q_dummy[int(self.num_ctrl_joints / 2)::2] = - 0.5 * angle

        q_reshape = np.reshape(q_dummy, (self.num_ctrl_joints, 1))

        q_reshape[0, 0] = - angle / 2
        q_reshape[1, 0] = angle
        q_reshape[2, 0] = - angle / 2
        q_reshape[3, 0] = angle

        self.q[7:] = q_reshape

        print(self.q)
        self.set_configuration(self.q)
        self.display(self.q)

    def set_configuration(self, q):
        self.q = q
        se3.forwardKinematics(self.model, self.data, self.q)

    def set_velocity(self, dq):
        self.dq = dq

    def set_acceleration(self, ddq):
        self.ddq = ddq

    def update_configuration(self, delta_q):
        self.q = se3.integrate(self.model, self.q, delta_q)

    def get_difference(self, q_1, q_2):
        return se3.difference(self.model, q_1, q_2)

    def get_jacobian(self, name, dofs=None, internal=True):
        if not self.model.existFrame(name) and not name == "COM":
            raise ValueError("Joint %s is not available." %name)
        if name == "universe" or name == "root_joint":
            raise ValueError("Joint %s is not available." %name)

        index = self.model.getFrameId(name)
        range_ = None
        if dofs == "TRANSLATION":
            range_ = range(3)
        elif dofs == "ROTATION":
            if name == "COM":
                raise ValueError("No rotation for COM available")
            range_ = range(3, 6)
        else:
            range_ = range(6)

        if internal:
            if name == "COM":
                def eval_jac_internal_com():
                    return self.Jcom(self.q)
                return eval_jac_internal_com
            else:
                def eval_jac_internal():
                    return se3.frameJacobian(self.model, self.data, self.q, index, se3.ReferenceFrame.LOCAL)[range_, :]
                return eval_jac_internal
        else:
            if name == "COM":
                return self.Jcom
            else:
                def eval_jac_at_q(q):
                    return se3.frameJacobian(self.model, self.data, q, index, se3.ReferenceFrame.LOCAL)[range_, :]
                return eval_jac_at_q

    def get_centroidal_momentum(self):
        def eval_centroidal_momentum():
            self.centroidalMomentum(self.q, self.dq)
            centroidal_momentum_matrix = self.data.Ag
            return centroidal_momentum_matrix

        return eval_centroidal_momentum

    def get_d_centroidal_momentum(self):
        def eval_d_centroidal_momentum():
            self.centroidalMomentum(self.q, self.dq)
            d_centroidal_momentum_matrix = self.data.dAg
            return d_centroidal_momentum_matrix

        return eval_d_centroidal_momentum

    def get_transformation(self, name, dofs=None):
        if not self.model.existFrame(name) and not name == "COM":
            raise ValueError("Transformation for %s is not available." %name)
        if name == "universe" or name == "root_joint":
            raise ValueError("Transformation for %s is not available." %name)

        def transformation():
            index = self.model.getFrameId(name)
            if dofs == "TRANSLATION":
                return self.data.oMf[index].translation
            elif dofs == "ROTATION":
                if name == "COM":
                    raise ValueError("No rotation for COM available")
                return self.data.oMf[index].rotation
            else:
                return self.data.oMf[index]

        def transformation_com():
            return self.com(self.q)

        if name == "COM":
            return transformation_com
        else:
            return transformation

    def get_desired_velocity(self, goal, transformation_func, dofs=None):
        def eval_vel(delta_t):
            if dofs == "TRANSLATION":
                return (goal - transformation_func()) / delta_t
            elif dofs is None:
                return se3.log(transformation_func().inverse() * goal).vector / delta_t
            else:
                raise ValueError("Implementation for %s not available" %dofs)

        return eval_vel

    def init_jacobians_and_trafos(self):
        # Create dictionary for jacobians
        self.jacobians_dict = {}
        # Create jacobian for COM
        self.jacobians_dict["COM"] = self.get_jacobian("COM")
        self.jacobians_dict["base_link"] = self.get_jacobian("base_link")
        # Create dictionary for transformations
        self.transformations_dict = {}
        # Create transformation for COM
        self.transformations_dict["COM"] = self.get_transformation("COM")
        self.transformations_dict["COM_GOAL"] = self.transformations_dict["COM"]().copy()
        self.transformations_dict["base_link"] = self.get_transformation("base_link")
        self.transformations_dict["base_link_GOAL"] = self.transformations_dict["base_link"]().copy()

        # Hip, knee and end effector jacobians and transformations
        for eff in self.effs:
            for joint in self.joints_list:
                joint_identifier = eff + "_" + joint
                self.jacobians_dict[joint_identifier] = self.get_jacobian(joint_identifier, "TRANSLATION")
                self.transformations_dict[joint_identifier] = self.get_transformation(joint_identifier, "TRANSLATION")

            # Create transformations for goal states
            self.transformations_dict[eff + "_END_GOAL"] = self.transformations_dict[eff + "_END"]().copy()

        self.centroidal_momentum = self.get_centroidal_momentum()
        self.d_centroidal_momentum = self.get_d_centroidal_momentum()

    def initDisplay(self,loadModel):
        self.robot.initDisplay(loadModel=loadModel)

        self.id_frame_base = self.model.getFrameId("base_link")
        self.id_frame_bl_end = self.model.getFrameId("BL_END")
        self.id_frame_bl_hfe = self.model.getFrameId("BL_HFE")
        self.id_frame_bl_kfe = self.model.getFrameId("BL_KFE")
        self.id_frame_fl_end = self.model.getFrameId("FL_END")
        self.id_frame_fl_hfe = self.model.getFrameId("FL_HFE")
        self.id_frame_fl_kfe = self.model.getFrameId("FL_KFE")
        self.id_frame_br_end = self.model.getFrameId("BR_END")
        self.id_frame_br_hfe = self.model.getFrameId("BR_HFE")
        self.id_frame_br_kfe = self.model.getFrameId("BR_KFE")
        self.id_frame_fr_end = self.model.getFrameId("FR_END")
        self.id_frame_fr_hfe = self.model.getFrameId("FR_HFE")
        self.id_frame_fr_kfe = self.model.getFrameId("FR_KFE")

        self.robot.viewer.gui.addSphere('world/blhip',.03,[1.0,0.0,0.0, 1.])
        self.robot.viewer.gui.addSphere('world/blknee',.03,[1.0,0.0,0.0, 1.])
        self.robot.viewer.gui.addSphere('world/bleff',.03,[1.0,0.0,0.0, 1.])
        self.robot.viewer.gui.addSphere('world/brhip',.03,[1.0,0.0,0.0, 1.])
        self.robot.viewer.gui.addSphere('world/brknee',.03,[1.0,0.0,0.0, 1.])
        self.robot.viewer.gui.addSphere('world/breff',.03,[1.0,0.0,0.0, 1.])
        self.robot.viewer.gui.addSphere('world/flhip',.03,[1.0,0.0,0.0, 1.])
        self.robot.viewer.gui.addSphere('world/flknee',.03,[1.0,0.0,0.0, 1.])
        self.robot.viewer.gui.addSphere('world/fleff',.03,[1.0,0.0,0.0, 1.])
        self.robot.viewer.gui.addSphere('world/frhip',.03,[1.0,0.0,0.0, 1.])
        self.robot.viewer.gui.addSphere('world/frknee',.03,[1.0,0.0,0.0, 1.])
        self.robot.viewer.gui.addSphere('world/freff',.03,[1.0,0.0,0.0, 1.])

    def display(self,q):
        #RobotWrapper.display(self,q)
        self.robot.display(q)
        se3.updateFramePlacements(self.model,self.data)

        self.robot.viewer.gui.applyConfiguration('world/blhip', se3ToXYZQUAT(self.data.oMf[self.id_frame_bl_hfe]))
        self.robot.viewer.gui.applyConfiguration('world/blknee', se3ToXYZQUAT(self.data.oMf[self.id_frame_bl_kfe]))
        self.robot.viewer.gui.applyConfiguration('world/bleff', se3ToXYZQUAT(self.data.oMf[self.id_frame_bl_end]))
        self.robot.viewer.gui.applyConfiguration('world/brhip', se3ToXYZQUAT(self.data.oMf[self.id_frame_br_hfe]))
        self.robot.viewer.gui.applyConfiguration('world/brknee', se3ToXYZQUAT(self.data.oMf[self.id_frame_br_kfe]))
        self.robot.viewer.gui.applyConfiguration('world/breff', se3ToXYZQUAT(self.data.oMf[self.id_frame_br_end]))
        self.robot.viewer.gui.applyConfiguration('world/flhip', se3ToXYZQUAT(self.data.oMf[self.id_frame_fl_hfe]))
        self.robot.viewer.gui.applyConfiguration('world/flknee', se3ToXYZQUAT(self.data.oMf[self.id_frame_fl_kfe]))
        self.robot.viewer.gui.applyConfiguration('world/fleff', se3ToXYZQUAT(self.data.oMf[self.id_frame_fl_end]))
        self.robot.viewer.gui.applyConfiguration('world/frhip', se3ToXYZQUAT(self.data.oMf[self.id_frame_fr_hfe]))
        self.robot.viewer.gui.applyConfiguration('world/frknee', se3ToXYZQUAT(self.data.oMf[self.id_frame_fr_kfe]))
        self.robot.viewer.gui.applyConfiguration('world/freff', se3ToXYZQUAT(self.data.oMf[self.id_frame_fr_end]))

        self.robot.viewer.gui.refresh()

############################ For debugging ##########################################

# urdf = str(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/src/urdf/quadruped.urdf')
# # # print("urdf_path:", urdf)
# robot = QuadrupedWrapper(urdf)
