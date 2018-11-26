import pybullet as p
import pinocchio as se3
import numpy as np
from time import sleep

from pinocchio.utils import zero


class Simulator(object):
    def __init__(self, robot_id, pinocchio_robot, joint_names, endeff_names):
        self.nq = pinocchio_robot.nq
        self.nv = pinocchio_robot.nv
        self.nj = len(joint_names)
        self.nf = len(endeff_names)
        self.robot_id = robot_id
        self.pinocchio_robot = pinocchio_robot

        self.joint_names = joint_names
        self.endeff_names = endeff_names

        bullet_joint_map = {}
        for ji in range(p.getNumJoints(robot_id)):
            bullet_joint_map[p.getJointInfo(robot_id, ji)[1].decode('UTF-8')] = ji

        self.joint_names = joint_names
        self.bullet_joint_ids = np.array([bullet_joint_map[name] for name in joint_names])
        self.pinocchio_joint_ids = np.array([pinocchio_robot.model.getJointId(name) for name in joint_names])

        self.pin2bullet_joint_only_array = []
        for i in range(2, self.nj + 2):
            self.pin2bullet_joint_only_array.append(np.where(self.pinocchio_joint_ids == i)[0][0])


        # Disable the velocity control on the joints as we use torque control.
        p.setJointMotorControlArray(robot_id, self.bullet_joint_ids, p.VELOCITY_CONTROL, forces=np.zeros(self.nj))

        # In pybullet, the contact wrench is measured at a joint. In our case
        # the joint is fixed joint. Pinocchio doesn't add fixed joints into the joint
        # list. Therefore, the computation is done wrt to the frame of the fixed joint.
        self.bullet_endeff_ids = [bullet_joint_map[name] for name in endeff_names]
        self.pinocchio_endeff_ids = [pinocchio_robot.model.getFrameId(name) for name in endeff_names]

    def _action(self, pos, rot):
        res = np.zeros((6, 6))
        res[:3, :3] = rot
        res[3:, 3:] = rot
        res[3:, :3] = se3.utils.skew(np.array(pos)).dot(rot)
        return res

    def get_force(self):
        """ Returns the force readings as well as the set of active contacts """
        active_contacts_frame_ids = []
        contact_forces = []

        # Get the contact model using the p.getContactPoints() api.
        def sign(x):
            if x >= 0:
                return 1.
            else:
                return -1.

        cp = p.getContactPoints()
        for ci in cp:
            i = np.where(np.array(self.bullet_endeff_ids) == ci[4])[0][0]
            active_contacts_frame_ids.append(self.pinocchio_endeff_ids[i])
            force = np.zeros(6)
            # force[2] = ci[9]

            force[0] += ci[11][0] * ci[10] + ci[13][0] * ci[12]
            force[1] += ci[11][1] * ci[10] + ci[13][1] * ci[12]
            force[2] += ci[11][2] * ci[10] + ci[13][2] * ci[12] + ci[9]

            contact_forces.append(force)

        return active_contacts_frame_ids, np.array(contact_forces).reshape(-1)

    def get_state(self):
        # Returns a pinocchio like representation of the q, dq matrixes
        q = zero(self.nq)
        dq = zero(self.nv)

        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        q[:3, 0] = np.array(pos).reshape(3, 1)
        q[3:7, 0] = np.array(orn).reshape(4, 1)

        vel, orn = p.getBaseVelocity(self.robot_id)
        dq[:3, 0] = np.array(vel).reshape(3, 1)
        dq[3:6, 0] = np.array(orn).reshape(3, 1)

        # Pinocchio assumes the base velocity to be in the body frame -> rotate.
        rot = np.matrix(p.getMatrixFromQuaternion(q[3:7])).reshape((3, 3))
        dq[0:3] = rot.T.dot(dq[0:3])
        dq[3:6] = rot.T.dot(dq[3:6])

        # Query the joint readings.
        joint_states = p.getJointStates(self.robot_id, self.bullet_joint_ids)

        for i in range(self.nj):
            q[5 + self.pinocchio_joint_ids[i], 0] = joint_states[i][0]
            dq[4 + self.pinocchio_joint_ids[i], 0] = joint_states[i][1]

        return q, dq

    def reset_state(self, q, dq):
        vec2list = lambda m: np.array(m.T).reshape(-1).tolist()
        p.resetBasePositionAndOrientation(self.robot_id, vec2list(q[:3]), vec2list(q[3:7]))

        # Pybullet assumes the base velocity to be aligned with the world frame.
        rot = np.matrix(p.getMatrixFromQuaternion(q[3:7])).reshape((3, 3))
        p.resetBaseVelocity(self.robot_id, vec2list(rot.dot(dq[:3])), vec2list(rot.dot(dq[3:6])))

        for i, bullet_joint_id in enumerate(self.bullet_joint_ids):
            p.resetJointState(self.robot_id, bullet_joint_id,
                q[5 + self.pinocchio_joint_ids[i]],
                dq[4 + self.pinocchio_joint_ids[i]])

    def send_joint_command(self, tau):
        # TODO: Apply the torques on the base towards the simulator as well.
        assert(tau.shape[0] == self.nv - 6)

        zeroGains = tau.shape[0] * (0.,)

        p.setJointMotorControlArray(self.robot_id, self.bullet_joint_ids, p.TORQUE_CONTROL,
                forces=tau[self.pin2bullet_joint_only_array],
                positionGains=zeroGains, velocityGains=zeroGains)

    def step(self):
        p.stepSimulation()
