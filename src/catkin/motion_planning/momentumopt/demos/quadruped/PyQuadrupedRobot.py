import pinocchio as pin
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper

class QuadrupedWrapper(RobotWrapper):

    def __init__(self, urdf):
        RobotWrapper.__init__(self, urdf, root_joint=pin.JointModelFreeFlyer())

        # Create data again after setting frames
        self.data = self.model.createData()

    def initDisplay(self,loadModel):
        print("InitializeDisplay")
        RobotWrapper.initDisplay(self,loadModel=loadModel)
        self.viewer.gui.addFloor('world/floor')
 
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
 
        frame_base = self.model.frames[self.id_frame_base]
        frame_bl_end = self.model.frames[self.id_frame_bl_end]
        frame_bl_hfe = self.model.frames[self.id_frame_bl_hfe]
        frame_bl_kfe = self.model.frames[self.id_frame_bl_kfe]
        frame_fl_end = self.model.frames[self.id_frame_fl_end]
        frame_fl_hfe = self.model.frames[self.id_frame_fl_hfe]
        frame_fl_kfe = self.model.frames[self.id_frame_fl_kfe]
        frame_br_end = self.model.frames[self.id_frame_br_end]
        frame_br_hfe = self.model.frames[self.id_frame_br_hfe]
        frame_br_kfe = self.model.frames[self.id_frame_br_kfe]
        frame_fr_end = self.model.frames[self.id_frame_fr_end]
        frame_fr_hfe = self.model.frames[self.id_frame_fr_hfe]
        frame_fr_kfe = self.model.frames[self.id_frame_fr_kfe]
  
        self.viewer.gui.addXYZaxis('world/framebasis',[1.,0.,0.,1.],.03,.1)
        self.viewer.gui.addXYZaxis('world/frameblend',[1.,0.,0.,1.],.03,.1)
        self.viewer.gui.addXYZaxis('world/frameblhfe',[1.,0.,0.,1.],.03,.1)
        self.viewer.gui.addXYZaxis('world/frameblkfe',[1.,0.,0.,1.],.03,.1)
        self.viewer.gui.addXYZaxis('world/frameflend',[1.,0.,0.,1.],.03,.1)
        self.viewer.gui.addXYZaxis('world/frameflhfe',[1.,0.,0.,1.],.03,.1)
        self.viewer.gui.addXYZaxis('world/frameflkfe',[1.,0.,0.,1.],.03,.1)
        self.viewer.gui.addXYZaxis('world/framebrend',[1.,0.,0.,1.],.03,.1)
        self.viewer.gui.addXYZaxis('world/framebrhfe',[1.,0.,0.,1.],.03,.1)
        self.viewer.gui.addXYZaxis('world/framebrkfe',[1.,0.,0.,1.],.03,.1)
        self.viewer.gui.addXYZaxis('world/framefrend',[1.,0.,0.,1.],.03,.1)
        self.viewer.gui.addXYZaxis('world/framefrhfe',[1.,0.,0.,1.],.03,.1)
        self.viewer.gui.addXYZaxis('world/framefrkfe',[1.,0.,0.,1.],.03,.1)
 
        # Add 3D modelling parts manually, because they are somehow not displayed
        self.viewer.gui.addBox('world/basis',.1,.2,.025,[1.0,1.0,1.0,1])
        self.viewer.gui.addCylinder('world/blfemoral',.02,.16,[0.4,0.4,0.4, 1.])
        self.viewer.gui.addCylinder('world/blshank',.02,.16,[0.4,0.4,0.4, 1.])
        self.viewer.gui.addCylinder('world/brfemoral',.02,.16,[0.4,0.4,0.4, 1.])
        self.viewer.gui.addCylinder('world/brshank',.02,.16,[0.4,0.4,0.4, 1.])
        self.viewer.gui.addCylinder('world/flfemoral',.02,.16,[0.4,0.4,0.4, 1.])
        self.viewer.gui.addCylinder('world/flshank',.02,.16,[0.4,0.4,0.4, 1.])
        self.viewer.gui.addCylinder('world/frfemoral',.02,.16,[0.4,0.4,0.4, 1.])
        self.viewer.gui.addCylinder('world/frshank',.02,.16,[0.4,0.4,0.4, 1.])
 
        self.viewer.gui.setStaticTransform('world/blfemoral',[ 0.00, 0.0,-0.08, 0.0,0.0,0.0,1.0])
        self.viewer.gui.setStaticTransform('world/blshank',[ 0.00, 0.0,-0.08, 0.0,0.0,0.0,1.0])
        self.viewer.gui.setStaticTransform('world/brfemoral',[ 0.00, 0.0,-0.08, 0.0,0.0,0.0,1.0])
        self.viewer.gui.setStaticTransform('world/brshank',[ 0.00, 0.0,-0.08, 0.0,0.0,0.0,1.0])
        self.viewer.gui.setStaticTransform('world/flfemoral',[ 0.00, 0.0,-0.08, 0.0,0.0,0.0,1.0])
        self.viewer.gui.setStaticTransform('world/flshank',[ 0.00, 0.0,-0.08, 0.0,0.0,0.0,1.0])
        self.viewer.gui.setStaticTransform('world/frfemoral',[ 0.00, 0.0,-0.08, 0.0,0.0,0.0,1.0])
        self.viewer.gui.setStaticTransform('world/frshank',[ 0.00, 0.0,-0.08, 0.0,0.0,0.0,1.0])

    def display(self,q):
        RobotWrapper.display(self,q)
        pin.updateFramePlacements(self.model,self.data)

        self.viewer.gui.applyConfiguration('world/basis', se3ToXYZQUAT(self.data.oMf[self.id_frame_base]))
        self.viewer.gui.applyConfiguration('world/blfemoral', se3ToXYZQUAT(self.data.oMf[self.id_frame_bl_hfe]))
        self.viewer.gui.applyConfiguration('world/blshank', se3ToXYZQUAT(self.data.oMf[self.id_frame_bl_kfe]))
        self.viewer.gui.applyConfiguration('world/brfemoral', se3ToXYZQUAT(self.data.oMf[self.id_frame_br_hfe]))
        self.viewer.gui.applyConfiguration('world/brshank', se3ToXYZQUAT(self.data.oMf[self.id_frame_br_kfe]))
        self.viewer.gui.applyConfiguration('world/flfemoral', se3ToXYZQUAT(self.data.oMf[self.id_frame_fl_hfe]))
        self.viewer.gui.applyConfiguration('world/flshank', se3ToXYZQUAT(self.data.oMf[self.id_frame_fl_kfe]))
        self.viewer.gui.applyConfiguration('world/frfemoral', se3ToXYZQUAT(self.data.oMf[self.id_frame_fr_hfe]))
        self.viewer.gui.applyConfiguration('world/frshank', se3ToXYZQUAT(self.data.oMf[self.id_frame_fr_kfe]))
          
        self.viewer.gui.applyConfiguration('world/framebasis',se3ToXYZQUAT(self.data.oMf[self.id_frame_base]))
        self.viewer.gui.applyConfiguration('world/frameblend',se3ToXYZQUAT(self.data.oMf[self.id_frame_bl_end]))  
        self.viewer.gui.applyConfiguration('world/frameblhfe',se3ToXYZQUAT(self.data.oMf[self.id_frame_bl_hfe]))
        self.viewer.gui.applyConfiguration('world/frameblkfe',se3ToXYZQUAT(self.data.oMf[self.id_frame_bl_kfe]))
        self.viewer.gui.applyConfiguration('world/frameflend',se3ToXYZQUAT(self.data.oMf[self.id_frame_fl_end]))
        self.viewer.gui.applyConfiguration('world/frameflhfe',se3ToXYZQUAT(self.data.oMf[self.id_frame_fl_hfe]))
        self.viewer.gui.applyConfiguration('world/frameflkfe',se3ToXYZQUAT(self.data.oMf[self.id_frame_fl_kfe]))
        self.viewer.gui.applyConfiguration('world/framebrend',se3ToXYZQUAT(self.data.oMf[self.id_frame_br_end]))
        self.viewer.gui.applyConfiguration('world/framebrhfe',se3ToXYZQUAT(self.data.oMf[self.id_frame_br_hfe]))
        self.viewer.gui.applyConfiguration('world/framebrkfe',se3ToXYZQUAT(self.data.oMf[self.id_frame_br_kfe]))
        self.viewer.gui.applyConfiguration('world/framefrend',se3ToXYZQUAT(self.data.oMf[self.id_frame_fr_end]))
        self.viewer.gui.applyConfiguration('world/framefrhfe',se3ToXYZQUAT(self.data.oMf[self.id_frame_fr_hfe]))
        self.viewer.gui.applyConfiguration('world/framefrkfe',se3ToXYZQUAT(self.data.oMf[self.id_frame_fr_kfe]))
          
        self.viewer.gui.refresh()
#