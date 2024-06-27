# PyMJCF
from dm_control import mjcf

# Composer high level imports
from dm_control import composer
from dm_control.composer.observation import observable
from dm_control.composer import variation

#Utils
import PIL.Image

MJCF_DIR = 'panther_robot/panther_mujoco.xml'

class Panther(composer.Entity):
    def _build(self):
        self._model = mjcf.from_path(MJCF_DIR)
    
    def _build_observables(self):
        return PantherObservables(self)
    
    @property
    def mjcf_model(self):
        return self._model
    
    @property
    def actuators(self):
        return tuple(self._model.find_all('actuator'))

class PantherObservables(composer.Observables):
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
    arena.worldbody.add('geom', type='plane', size=[10, 10, .1], material=grid)
    for x in [-2, 2]:
        arena.worldbody.add('light', pos=[x, -1, 3], dir=[-x, 1, -2])

    # Instantiate 6 creatures with 3 to 8 legs.
    panther = Panther().mjcf_model
    
    spawn_pos = (0,0,0)
    spawn_site = arena.worldbody.add('site', pos=spawn_pos, group=3)
    # Attach to the arena at the spawn sites, with a free joint.
    spawn_site.attach(panther).add('freejoint')

    # Instantiate the physics and render.
    physics = mjcf.Physics.from_mjcf_model(arena)
    image= PIL.Image.fromarray(physics.render())
    image.save('tests/panther_pymjcf_test.png')