# PyMJCF
from dm_control import mjcf

# Composer high level imports
from dm_control import composer
from dm_control.composer.observation import observable
from dm_control.composer import variation

# MoMa Robot modules
from panther import Panther
from ur5e import UR5e

# Utils
import PIL.Image


class MoMaRobot(composer.Entity):
    def _build(self):
        self._model = mjcf.RootElement()
        self._model.compiler.angle = 'radian'
        panther = Panther()
        ur5e = UR5e()

        site = self._model.worldbody.add('site', size=[1e-6]*3, pos=[0, 0, 0.5])
        site.attach(panther.mjcf_model)
        site.attach(ur5e.mjcf_model)

    def _build_observables(self):
        return MoMaRobotObservables(self)
    
    @property
    def mjcf_model(self):
        return self._model
    
    @property
    def actuators(self):
        return tuple(self._model.find_all('actuator'))

class MoMaRobotObservables(composer.Observables):
    @composer.observable
    def joint_positions(self):
        all_joints = self._entity.mjcf_model.find_all('joint')
        return observable.MJCFFeature('qpos', all_joints)

    @composer.observable
    def joint_velocities(self):
        all_joints = self._entity.mjcf_model.find_all('joint')
        return observable.MJCFFeature('qvel', all_joints)
    

if __name__ == '__main__':
    arena = mjcf.RootElement()
    chequered = arena.asset.add('texture', type='2d', builtin='checker', width=300,
                                height=300, rgb1=[.2, .3, .4], rgb2=[.3, .4, .5])
    grid = arena.asset.add('material', name='grid', texture=chequered,
                        texrepeat=[5, 5], reflectance=.2)
    arena.worldbody.add('geom', type='plane', size=[2, 2, .1], material=grid)
    for x in [-2, 2]:
        arena.worldbody.add('light', pos=[x, -1, 3], dir=[-x, 1, -2])

    # Instantiate 6 creatures with 3 to 8 legs.
    moma_robot = MoMaRobot()
    xml = moma_robot.mjcf_model.to_xml_string()
    with open('moma_robot.xml', 'w+')as file:
        file.write(xml)
    